#!/usr/bin/env python3
"""Read control replies across arbitrary HTTP chunk boundaries, without a robot."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import requests
from wire import WsConsumer


MESSAGE = {"type": "peer", "rpc": {"id": 1, "result": {"label": "démo 🦆"}}}


def frame(ending, message=MESSAGE):
    return ("data: " + json.dumps(message, ensure_ascii=False) + ending * 2).encode("utf-8")


class Stream:
    def __init__(self, chunks):
        self.chunks = chunks

    def iter_content(self, chunk_size=None):
        return iter(self.chunks)

    def close(self):
        pass


class ControlStreamFraming(unittest.TestCase):
    def collect(self, response):
        rpc = Mock()
        consumer = WsConsumer("local-fixture", "duck-a", rpc)
        consumer._stream = response
        try:
            consumer._pump()
        finally:
            consumer.stop()
        return [json.loads(call.args[0]) for call in rpc.on_message.call_args_list]

    def test_whole_lf_frame(self):
        self.assertEqual(self.collect(Stream([frame("\n")])), [MESSAGE["rpc"]])

    def test_whole_crlf_frame(self):
        self.assertEqual(self.collect(Stream([frame("\r\n")])), [MESSAGE["rpc"]])

    def test_crlf_split_across_chunks(self):
        # Normalising each chunk independently misses both split CRLF pairs.
        raw = frame("\r\n")
        chunks = [raw[:-4], b"\r", b"\n", b"\r", b"\n"]
        self.assertEqual(self.collect(Stream(chunks)), [MESSAGE["rpc"]])

    def test_utf8_split_across_chunks(self):
        # Decoding each byte independently replaces otherwise valid characters.
        chunks = [bytes([byte]) for byte in frame("\n")]
        self.assertEqual(self.collect(Stream(chunks)), [MESSAGE["rpc"]])

    def test_all_two_chunk_splits(self):
        # Parsing must be invariant to where the HTTP layer cuts a reply.
        for ending in ("\n", "\r\n"):
            raw = frame(ending)
            for offset in range(1, len(raw)):
                with self.subTest(ending=repr(ending), offset=offset):
                    chunks = [raw[:offset], raw[offset:]]
                    self.assertEqual(self.collect(Stream(chunks)), [MESSAGE["rpc"]])

    def test_multiple_frames_with_partial_tail(self):
        # Dispatch complete replies in order and leave an unfinished final event alone.
        second = {"type": "peer", "rpc": {"id": 2, "result": "second reply"}}
        raw = frame("\r\n") + frame("\n", second) + frame("\n")[:-2]
        chunks = [bytes([byte]) for byte in raw]
        self.assertEqual(self.collect(Stream(chunks)), [MESSAGE["rpc"], second["rpc"]])

    def test_real_chunked_http_response(self):
        # Each HTTP chunk may end anywhere in the event's bytes. Exercise the
        # actual requests Response iterator as well as the synthetic cases above.
        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Transfer-Encoding", "chunked")
                self.end_headers()
                for byte in frame("\r\n"):
                    self.wfile.write(b"1\r\n" + bytes([byte]) + b"\r\n")
                self.wfile.write(b"0\r\n\r\n")
                self.wfile.flush()

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            with requests.get(
                f"http://127.0.0.1:{server.server_port}/events",
                stream=True,
                timeout=3,
                proxies={"http": "", "https": ""},
            ) as response:
                self.assertEqual(self.collect(response), [MESSAGE["rpc"]])
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main(verbosity=2)
