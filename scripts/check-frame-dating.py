#!/usr/bin/env python3
"""Print, per decoded frame, its capture time beside the newest `robot.state` sample.

Run it from a laptop against a robot:

    ssh -NL /tmp/robotd.sock:/run/robotd.sock root@<robot> &
    scripts/check-frame-dating.py --signaller ws://<robot>:8443 --robot-socket /tmp/robotd.sock

The two columns should move together with a gap inside the sample period (100 ms at the 10 Hz
asked for below). About 1.8e18 ns means the frame is dated on the wall clock; no lines at all
means the frames carry no date. `webrtcsrc` is used rather than a hand-rolled receiver because
the reference timestamp comes from `rtpjitterbuffer` and the extension id from the SDP.
"""

import argparse
import json
import socket
import sys
import threading
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

# `robotd`'s socket, and the handshake its own clients use. `duck-ipc-proto` owns both; they are
# literals here because this script has to run without building the workspace.
ROBOT_SOCKET = "/run/robotd.sock"
API_VERSION = 34

# `GstWebRTCFECType`'s `none` member. Written as the number because GI reads the property through
# the `GstWebRTC` typelib's enum, which a board does not have installed — `webrtc_fwd.h` fixes the
# value at 0 for anything that is not a typelib, and the property accepts a plain int either way.
FEC_NONE = 0


class State:
    """The newest `robot.state` sample, read off `robotd` in a thread of its own.

    A thread rather than a GLib source: the socket is blocking and `robotd` pushes at the rate it
    was asked for, and a reader that falls behind is harmless — the comparison wants the *latest*
    sample, so an old one being overwritten is the ordinary case rather than a loss.
    """

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
            # The handshake first. A daemon that does not know this version still answers, and
            # the answer is not read: this is not the version check, it is what opens the duplex.
            self._send(f, 1, "hello", {"api_version": API_VERSION})
            self._send(f, 2, "robot.subscribe", {"hz": 10})
            while not self._stop.is_set():
                line = f.readline()
                if not line:
                    break
                message = json.loads(line)
                # Notifications carry no `id` and name their stream in `method`; that is how
                # `robot.subscribe` delivers, and why the reply to each call is skipped.
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
    """Ask this session's `rtpbin` for the frame dates, and keep FEC off its transceivers.

    `add-reference-timestamp-meta` is the whole of the first half: with it, `rtpjitterbuffer`
    attaches the RTP-to-NTP mapping to every frame it pops, which is a `GstReferenceTimestampMeta`
    on the buffer the decoder receives. The property belongs to `rtpbin`, not to `webrtcbin`, so
    the element is found by name — the same route upstream's own `webrtc-precise-sync-recv` takes.
    """
    rtpbin = webrtcbin.get_by_name("rtpbin")
    if rtpbin is None:
        print(
            "no rtpbin under webrtcbin; frames will carry no capture time. "
            "Is this an unpatched build?",
            file=sys.stderr,
        )
        return

    rtpbin.set_property("add-reference-timestamp-meta", True)

    def on_new_transceiver(_webrtcbin, transceiver):
        def enforce(_transceiver, _pspec):
            # Read before writing, and write only on a difference. A property without
            # G_PARAM_EXPLICIT_NOTIFY notifies on every set, including a set to the value it
            # already holds, so an unconditional write here calls itself forever.
            if int(transceiver.get_property("fec-type")) != FEC_NONE:
                transceiver.set_property("fec-type", FEC_NONE)

        # Once now, for the transceivers `webrtcsrc` has already made, and once for each one it
        # makes afterwards — which of the two orders this runs in is not something to depend on.
        enforce(transceiver, None)
        transceiver.connect("notify::fec-type", enforce)

    webrtcbin.connect("on-new-transceiver", on_new_transceiver)


class Report:
    """The probe's counter, and the line it prints per frame."""

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

        t_ns = int(sample["t_ns"])
        print(f"{capture:>20}  {t_ns:>20}  {capture - t_ns:>+19d}")

        # Left signed rather than absolute. The sample is read when the frame arrives, not when
        # the sample was taken, so it is usually the older of the two and the difference is
        # usually negative -- but a frame is decoded some 60 ms after capture, so a sample taken
        # in that window is the newer one. Either sign is meaningful and folding it away would
        # hide the direction that matters.
        return Gst.PadProbeReturn.OK


def on_pad_added(pipeline, pad, report):
    """Decode whatever `webrtcsrc` produced, and hand the frames to the probe.

    `webrtcsrc` names its pads `video_%u`/`audio_%u` (a ghost of `rtpptdemux`'s own naming), so
    the media kind is in the name and does not need reading out of the caps. Audio is ignored:
    its frames are dated by the same machinery, but they are 48 kHz of them and the question
    here is about the picture.
    """
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
        help="the producer's signalling peer id. Omit it to connect to whichever producer "
        "registers first, which is what a one-robot network wants: the id is per-session and "
        "changes on every restart of mediad.",
    )
    parser.add_argument(
        "--signaller",
        default="ws://127.0.0.1:8443",
        help="the robot's signalling server (default ws://127.0.0.1:8443)",
    )
    parser.add_argument(
        "--seconds", type=float, default=30.0, help="how long to watch (default 30)"
    )
    parser.add_argument(
        "--robot-socket", default=ROBOT_SOCKET, help=f"robotd's socket (default {ROBOT_SOCKET})"
    )
    args = parser.parse_args()

    Gst.init(None)

    for element in ("webrtcsrc", "rtphdrextntp64", "rtpjitterbuffer"):
        if Gst.ElementFactory.find(element) is None:
            sys.exit(
                f"{element} is not registered. webrtcsrc comes from gst-plugin-webrtc; "
                "rtphdrextntp64 and rtpjitterbuffer from gst-plugins-good's rtpmanager."
            )

    state = State(args.robot_socket)
    state.start()

    pipeline = Gst.Pipeline.new("frame-dating")
    webrtcsrc = Gst.ElementFactory.make("webrtcsrc", None)
    pipeline.add(webrtcsrc)

    report = Report(state)

    signaller = webrtcsrc.get_property("signaller")
    # `connect-to-first-producer` is a query parameter rather than a property, and it is the
    # only way to reach a robot without knowing the id it happened to register with this time.
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

    # PLAYING, not just PLAYING after a gap: `webrtcsrc` connects to the signaller when its state
    # machine reaches READY→PAUSED, so nothing at all happens until this call.
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
            "No frame was dated. Either the robot is not running the change that adds the "
            "ntp-64 extension, or they arrived without surviving the jitterbuffer.",
            file=sys.stderr,
        )
    return 0 if report.stamped else 1


if __name__ == "__main__":
    sys.exit(main())
