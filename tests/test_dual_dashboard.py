"""Real local HTTP tests with fake receivers; no robot or HEBI access."""

from contextlib import ExitStack
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import unittest
import uuid
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from dual_dashboard import DashboardServer


class Receiver(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(self.server.state).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.headers.get("Origin") != f"http://127.0.0.1:{self.server.server_port}":
            self.send_error(403)
            return
        self.server.commands.append(
            json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        )
        self.send_response(204)
        self.end_headers()

    def log_message(self, *args):
        pass


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.receivers = {}
        profiles = {}
        for n in (1, 2):
            robot = f"follower_{n}"
            server = ThreadingHTTPServer(("127.0.0.1", 0), Receiver)
            server.commands = []
            server.state = dict(
                robot_id=robot,
                phone_family="HEBI",
                phone_name=f"phone{n}",
                connected=True,
                tracking=True,
                b1=False,
                a3=0,
                delta_cm=[0, 0, 0],
            )
            profiles[robot] = dict(
                port=server.server_port, name=f"phone{n}", family="HEBI"
            )
            self.receivers[robot] = server
            self.start(server)
        self.dashboard = DashboardServer(("127.0.0.1", 0), profiles)
        self.start(self.dashboard)
        self.url = f"http://127.0.0.1:{self.dashboard.server_port}"

    def start(self, server):
        threading.Thread(
            target=server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True
        ).start()
        self.stack.callback(server.server_close)
        self.stack.callback(server.shutdown)

    def request(self, path, data=None, origin=None, host=None):
        headers = {"Origin": origin or self.url, "Content-Type": "application/json"}
        if host:
            headers["Host"] = host
        try:
            response = urlopen(
                Request(
                    self.url + path,
                    headers=headers,
                    data=None if data is None else json.dumps(data).encode(),
                ),
                timeout=3,
            )
        except HTTPError as error:
            response = error
        with response:
            body = response.read()
            return response.status, json.loads(
                body
            ) if response.headers.get_content_type() == "application/json" else body

    def test_both_channels_and_static_page(self):
        code, data = self.request("/api/state")
        self.assertEqual(code, 200)
        self.assertEqual(
            [a["robot_id"] for a in data["arms"]], ["follower_1", "follower_2"]
        )
        self.assertTrue(all(a["available"] for a in data["arms"]))
        for path in ("/", "/app.js", "/style.css"):
            self.assertEqual(self.request(path)[0], 200)
        self.assertEqual(self.request("/../../config.toml")[0], 404)

    def test_pause_goes_only_to_selected_arm(self):
        self.assertEqual(
            self.request("/api/arms/follower_1/control", {"action": "pause"})[0], 200
        )
        self.assertEqual(self.receivers["follower_1"].commands, [{"action": "pause"}])
        self.assertEqual(self.receivers["follower_2"].commands, [])

    def test_supported_release_is_arm_specific(self):
        path = "/api/arms/follower_2/control"
        for value in (None, False, "true", 1):
            self.assertEqual(
                self.request(path, {"action": "release", "supported": value})[0], 400
            )
        self.assertEqual(
            self.request(path, {"action": "release", "supported": True})[0], 200
        )
        self.assertEqual(self.receivers["follower_1"].commands, [])
        self.assertEqual(len(self.receivers["follower_2"].commands), 1)

    def test_external_origins_hosts_and_invalid_commands_rejected(self):
        path = "/api/arms/follower_1/control"
        self.assertEqual(
            self.request(path, {"action": "pause"}, origin="https://example.com")[0],
            403,
        )
        self.assertEqual(self.request("/api/state", host="example.com")[0], 403)
        for data in (
            [],
            None,
            {"action": "start"},
            {"action": "pause", "padding": "x" * 1100},
        ):
            if data is not None:
                self.assertEqual(self.request(path, data)[0], 400)
        self.assertEqual(
            self.request("/api/arms/follower_3/control", {"action": "pause"})[0], 404
        )
        self.assertEqual(self.receivers["follower_1"].commands, [])

    def test_disconnected_channel_does_not_hide_other_arm(self):
        self.receivers["follower_1"].shutdown()
        self.receivers["follower_1"].server_close()
        _, data = self.request("/api/state")
        self.assertFalse(data["arms"][0]["available"])
        self.assertNotIn("state", data["arms"][0])
        self.assertTrue(data["arms"][1]["available"])
        self.assertEqual(
            self.request("/api/arms/follower_1/control", {"action": "pause"})[0], 503
        )

    def test_reset_requires_support_and_idle_phone_and_routes_only_selected_arm(self):
        sessions = Mock()
        sessions.snapshot.return_value = dict(busy=False, reset_available=True)
        sessions.start.return_value = dict(
            busy=True, phase="starting", message="Starting session"
        )
        self.dashboard.sessions = sessions
        path = "/api/arms/follower_2/control"
        token = str(uuid.uuid4())
        request = dict(action="reset", supported=True, request_id=token)
        self.assertEqual(self.request(path, dict(action="reset"))[0], 400)
        self.receivers["follower_2"].state["b1"] = True
        self.assertEqual(self.request(path, request)[0], 409)
        self.receivers["follower_2"].state.update(b1=False, a3=0.4)
        self.assertEqual(self.request(path, request)[0], 409)
        sessions.start.assert_not_called()
        self.receivers["follower_2"].state["a3"] = 0
        self.assertEqual(self.request(path, request)[0], 202)
        sessions.start.assert_called_once_with("follower_2", token)
        self.assertFalse(self.receivers["follower_1"].commands)
        self.assertFalse(self.receivers["follower_2"].commands)
        sessions.snapshot.return_value = dict(busy=True, reset_available=True)
        self.assertEqual(
            self.request(path, dict(action="release", supported=True))[0], 409
        )

    def test_reset_not_configured(self):
        self.assertEqual(
            self.request(
                "/api/arms/follower_1/control",
                dict(action="reset", supported=True, request_id=str(uuid.uuid4())),
            )[0],
            409,
        )

    def test_identity_mismatch_blocks_commands(self):
        self.receivers["follower_1"].state["robot_id"] = "follower_2"
        self.assertEqual(
            self.request("/api/arms/follower_1/control", {"action": "pause"})[0], 503
        )
        self.receivers["follower_1"].state["robot_id"] = "follower_1"
        self.receivers["follower_1"].state["phone_name"] = "wrong-phone"
        self.assertEqual(
            self.request("/api/arms/follower_1/control", {"action": "pause"})[0], 503
        )
        self.assertEqual(self.receivers["follower_1"].commands, [])


if __name__ == "__main__":
    unittest.main()
