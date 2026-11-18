#!/usr/bin/env python3
"""Print each decoded frame's capture time beside the latest `robot.state` `t_ns`.

Run it from a laptop against the robot's signalling server, with `robotd`'s socket forwarded:

    ssh -NL /tmp/robotd.sock:/run/robotd.sock root@<robot> &
    scripts/check-frame-dating.py --signaller ws://<robot>:8443 --robot-socket /tmp/robotd.sock

Both numbers are `CLOCK_MONOTONIC` on the robot, so the difference says whether the frame or the
sample beside it is the newer, and by how much — bounded by the 10 Hz sample period, not a
measurement of the network.
"""

import argparse
import json
import socket
import sys
import threading

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

ROBOT_SOCKET = "/run/robotd.sock"
API_VERSION = 44

# `GstWebRTCFECType`'s `none`, as the number: GI reads the property through the `GstWebRTC`
# typelib, which a laptop may not have.
FEC_NONE = 0


class State:
    """The newest `robot.state` sample, read off `robotd` in a thread of its own."""

    def __init__(self, path):
        self.latest = None
        self.error = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read, args=(path,), daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _read(self, path):
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(path)
            f = sock.makefile("rwb")
            self._send(f, 1, "hello", {"api_version": API_VERSION})
            self._send(f, 2, "robot.subscribe", {"hz": 10})
            while not self._stop.is_set():
                line = f.readline()
                if not line:
                    break
                message = json.loads(line)
                if message.get("method") == "robot.state":
                    self.latest = message.get("params")
        except OSError as exc:
            self.error = exc

    @staticmethod
    def _send(f, ident, method, params):
        request = {"jsonrpc": "2.0", "id": ident, "method": method, "params": params}
        f.write((json.dumps(request) + "\n").encode())
        f.flush()


def configure_rtpbin(_signaller, _peer_id, webrtcbin):
    rtpbin = webrtcbin.get_by_name("rtpbin")
    if rtpbin is None:
        print("no rtpbin under webrtcbin; frames carry no capture time", file=sys.stderr)
        return

    rtpbin.set_property("add-reference-timestamp-meta", True)

    def on_new_transceiver(_webrtcbin, transceiver):
        def enforce(_transceiver, _pspec):
            # Written only on a difference: without EXPLICIT_NOTIFY this notifies on every set.
            if int(transceiver.get_property("fec-type")) != FEC_NONE:
                transceiver.set_property("fec-type", FEC_NONE)

        enforce(transceiver, None)
        transceiver.connect("notify::fec-type", enforce)

    webrtcbin.connect("on-new-transceiver", on_new_transceiver)


class Report:
    """The probe's counters, and the line it prints per frame."""

    def __init__(self, state):
        self.state = state
        self.frames = 0
        self.stamped = 0
        self._header_printed = False

    def on_buffer(self, _pad, info):
        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK

        self.frames += 1
        meta = buf.get_reference_timestamp_meta(None)
        if meta is None:
            return Gst.PadProbeReturn.OK

        self.stamped += 1
        if not self._header_printed:
            print(f"{'capture (ns)':>20}  {'robot.state t_ns':>20}  {'difference':>19}")
            self._header_printed = True

        capture = int(meta.timestamp)
        sample = self.state.latest
        if not sample or not sample.get("t_ns"):
            print(f"{capture:>20}  {'(no sample yet)':>20}")
            return Gst.PadProbeReturn.OK

        # The sample is read when the frame is popped, not when it was captured, and a frame is
        # decoded after it was captured too — so either number can be the newer one.
        t_ns = int(sample["t_ns"])
        print(f"{capture:>20}  {t_ns:>20}  {capture - t_ns:>+19d}")
        return Gst.PadProbeReturn.OK


def on_pad_added(pipeline, pad, report):
    if not pad.get_name().startswith("video"):
        return

    decodebin = Gst.ElementFactory.make("decodebin", None)
    pipeline.add(decodebin)

    sink = Gst.ElementFactory.make("fakesink", None)
    sink.set_property("sync", False)
    sink.set_property("async", False)
    pipeline.add(sink)
    sink.get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, report.on_buffer)

    if pad.link(decodebin.get_static_pad("sink")) != Gst.PadLinkReturn.OK:
        print(f"could not link {pad.get_name()} to decodebin", file=sys.stderr)
        return

    def on_decodebin_pad(_decodebin, decodebin_pad):
        if decodebin_pad.get_name().startswith("video"):
            decodebin_pad.link(sink.get_static_pad("sink"))

    decodebin.connect("pad-added", on_decodebin_pad)
    decodebin.sync_state_with_parent()
    sink.sync_state_with_parent()


def main():
    parser = argparse.ArgumentParser(
        description="Print each frame's capture time beside the latest robot.state t_ns."
    )
    parser.add_argument(
        "peer_id",
        nargs="?",
        help="the producer's signalling peer id; omitted, connects to whichever registers first",
    )
    parser.add_argument("--signaller", default="ws://127.0.0.1:8443")
    parser.add_argument(
        "--seconds", type=float, default=30.0, help="how long to watch (default 30)"
    )
    parser.add_argument("--robot-socket", default=ROBOT_SOCKET)
    args = parser.parse_args()

    Gst.init(None)

    for element in ("webrtcsrc", "rtphdrextntp64", "rtpjitterbuffer"):
        if Gst.ElementFactory.find(element) is None:
            sys.exit(f"{element} is not registered")

    state = State(args.robot_socket)
    state.start()

    pipeline = Gst.Pipeline.new("frame-dating")
    webrtcsrc = Gst.ElementFactory.make("webrtcsrc", None)
    pipeline.add(webrtcsrc)

    report = Report(state)

    signaller = webrtcsrc.get_property("signaller")
    uri = args.signaller
    if args.peer_id:
        signaller.set_property("producer-peer-id", args.peer_id)
    else:
        uri += "?connect-to-first-producer=true"
    signaller.set_property("uri", uri)
    signaller.connect("webrtcbin-ready", configure_rtpbin)

    webrtcsrc.connect("pad-added", lambda _src, pad: on_pad_added(pipeline, pad, report))

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(_bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            print(f"\npipeline error: {error.message}", file=sys.stderr)
            if debug:
                print(debug, file=sys.stderr)
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    bus.connect("message", on_message)
    GLib.timeout_add(int(args.seconds * 1000), loop.quit)

    # `webrtcsrc` dials the signaller on READY→PAUSED, so nothing happens before this.
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except KeyboardInterrupt:
        pass

    pipeline.set_state(Gst.State.NULL)
    state.stop()

    if state.error is not None:
        print(f"\nrobotd: {state.error}", file=sys.stderr)
    print(f"\n{report.stamped} of {report.frames} frames carried a capture time")
    if report.frames and not report.stamped:
        print(
            "No frame was dated: either the robot is not running the ntp-64 extension, or they "
            "did not survive the jitterbuffer.",
            file=sys.stderr,
        )
    return 0 if report.stamped else 1


if __name__ == "__main__":
    sys.exit(main())
