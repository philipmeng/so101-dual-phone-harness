"""One local dashboard for two existing receivers. No HEBI or serial access."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import sys
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler

from phone_profiles import profile
from session_reset import SessionReset

ASSETS = Path(__file__).with_name("dashboard")
ROBOTS = ("follower_1", "follower_2")


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def upstream(port, path, payload=None):
    """Only fixed loopback receivers; ignore environment proxy settings."""
    url = f"http://127.0.0.1:{port}"
    headers = {"Origin": url, "Content-Type": "application/json"}
    request = Request(url + path, data=payload, headers=headers)
    opener = build_opener(ProxyHandler({}), NoRedirect())
    with opener.open(request, timeout=1.2) as response:
        raw = response.read(65537)
        if len(raw) > 65536:
            raise ValueError("Receiver response too large")
        return json.loads(raw) if raw else None


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, profiles, sessions=None):
        self.profiles = profiles
        self.sessions = sessions
        self.command_locks = {robot: threading.Lock() for robot in ROBOTS}
        super().__init__(address, Handler)

    def read_arm(self, robot):
        p = self.profiles[robot]
        result = dict(
            robot_id=robot,
            phone_name=p["name"],
            phone_family=p["family"],
            port=p["port"],
            available=False,
            session=self.sessions.snapshot(robot)
            if self.sessions
            else dict(reset_available=False, busy=False),
        )
        try:
            state = upstream(p["port"], "/state")
            if not isinstance(state, dict) or state.get("robot_id") != robot:
                raise ValueError("Receiver identity does not match this arm")
            if (state.get("phone_family"), state.get("phone_name")) != (
                p["family"],
                p["name"],
            ):
                raise ValueError("Receiver phone identity does not match this arm")
            result.update(available=True, state=state)
        except ValueError as error:
            result["error"] = str(error)
        except (OSError, HTTPError):
            result["error"] = "Receiver unreachable. Check the Pi and its connection."
        return result


class Handler(BaseHTTPRequestHandler):
    def reply(self, status, value, kind="application/json"):
        body = json.dumps(value).encode() if kind == "application/json" else value
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'",
        )
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def local_host(self):
        return self.headers.get("Host") in (
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        )

    def do_GET(self):
        if not self.local_host():
            self.reply(403, {"error": "Use the local dashboard address"})
            return
        if self.path == "/api/state":
            with ThreadPoolExecutor(max_workers=2) as pool:
                arms = list(pool.map(self.server.read_arm, ROBOTS))
            self.reply(200, dict(arms=arms, checked_at=time.time()))
            return
        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/style.css": ("style.css", "text/css; charset=utf-8"),
        }
        if self.path not in assets:
            self.reply(404, {"error": "Not found"})
            return
        name, kind = assets[self.path]
        self.reply(200, (ASSETS / name).read_bytes(), kind)

    def do_POST(self):
        origins = (
            f"http://127.0.0.1:{self.server.server_port}",
            f"http://localhost:{self.server.server_port}",
        )
        if (
            not self.local_host()
            or self.headers.get("Origin") not in origins
            or self.headers.get("Content-Type") != "application/json"
        ):
            self.reply(403, {"error": "Use controls on the local dashboard"})
            return
        routes = {f"/api/arms/{robot}/control": robot for robot in ROBOTS}
        if self.path not in routes:
            self.reply(404, {"error": "Unknown arm"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size < 1024:
                raise ValueError("Invalid request size")
            self.connection.settimeout(2)
            command = json.loads(self.rfile.read(size))
            if not isinstance(command, dict) or command.get("action") not in (
                "pause",
                "release",
                "reset",
            ):
                raise ValueError("Choose pause, release or reset")
            if (
                command["action"] in ("release", "reset")
                and command.get("supported") is not True
            ):
                raise ValueError("Confirm that this arm is supported before releasing")
        except (ValueError, TypeError, OSError) as error:
            self.reply(400, {"error": str(error)})
            return
        robot = routes[self.path]
        # Serialize each arm's commands: the receiver atomically replaces one command file.
        with self.server.command_locks[robot]:
            arm = self.server.read_arm(robot)
            if not arm["available"]:
                self.reply(503, {"error": arm["error"]})
                return
            if command["action"] == "reset":
                if self.server.sessions is None:
                    self.reply(
                        409,
                        {"error": "Session reset must be enabled on the Pi dashboard"},
                    )
                    return
                state = arm["state"]
                if state.get("connected") and (
                    state.get("b1")
                    or not isinstance(state.get("a3"), (int, float))
                    or not abs(state["a3"]) < 0.15
                ):
                    self.reply(
                        409, {"error": "Release B1 and center A3 before resetting"}
                    )
                    return
                try:
                    job = self.server.sessions.start(robot, command.get("request_id"))
                except (ValueError, RuntimeError) as error:
                    self.reply(409, {"error": str(error)})
                    return
                self.reply(202, {"message": job["message"], "session": job})
                return
            if self.server.sessions and self.server.sessions.snapshot(robot)["busy"]:
                self.reply(
                    409, {"error": "This arm is resetting; wait for it to finish"}
                )
                return
            try:
                upstream(arm["port"], "/control", json.dumps(command).encode())
            except (OSError, ValueError, HTTPError):
                self.reply(
                    502,
                    {
                        "error": "Request not confirmed. Check the arm status before retrying."
                    },
                )
                return
        self.reply(
            200, {"message": "Request sent. Watch the arm status for confirmation."}
        )

    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        help="Use the harness configuration; defaults to receiver ports 8766/8765",
    )
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument(
        "--manage-sessions",
        action="store_true",
        help="Enable supported session resets; run on the Pi with the controller Python environment",
    )
    parser.add_argument(
        "--session-commands",
        type=Path,
        help="Optional local JSON mapping of arm IDs to controller command argument lists for an existing installation",
    )
    args = parser.parse_args()
    if args.config:
        from phone_profiles import load_config

        load_config(args.config)
    profiles = {robot: profile(robot) for robot in ROBOTS}
    if not 1024 <= args.port <= 65535 or args.port in {
        p["port"] for p in profiles.values()
    }:
        parser.error(
            "Choose a dashboard port from 1024 to 65535 distinct from both receivers"
        )
    sessions = None
    if args.manage_sessions:
        if not args.config and not args.session_commands:
            parser.error("Session resets require --config or --session-commands")
        commands = (
            json.loads(args.session_commands.read_text())
            if args.session_commands
            else {
                robot: [
                    sys.executable,
                    "-u",
                    str(Path(__file__).with_name("phone_control.py")),
                    "--robot-id",
                    robot,
                    "--config",
                    str(args.config.resolve()),
                    "--base-clamped-sweep-clear",
                ]
                for robot in ROBOTS
            }
        )
        if set(commands) != set(ROBOTS):
            parser.error("Configure session commands for exactly both arms")
        sessions = SessionReset(
            profiles,
            commands,
            Path(__file__).resolve().parents[1],
            lambda robot: upstream(
                profiles[robot]["port"],
                "/control",
                json.dumps(dict(action="release", supported=True)).encode(),
            ),
        )
    elif args.session_commands:
        parser.error("--session-commands requires --manage-sessions")
    with DashboardServer(("127.0.0.1", args.port), profiles, sessions) as server:
        print(f"Both arms: http://127.0.0.1:{server.server_port}/", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
