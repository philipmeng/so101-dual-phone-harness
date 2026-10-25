"""Explicit, per-arm controller restart without accessing the serial bus."""

import fcntl
import json
from pathlib import Path
import subprocess
import threading
import time
import uuid


def controller_lock(directory):
    handle = (directory / "controller.lock").open("a")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        return None
    return handle


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


class SessionReset:
    def __init__(
        self,
        profiles,
        commands,
        cwd,
        send_release,
        release_timeout=12,
        startup_timeout=30,
    ):
        self.profiles = profiles
        self.commands = commands
        self.cwd = Path(cwd)
        self.send_release = send_release
        self.release_timeout = release_timeout
        self.startup_timeout = startup_timeout
        self.guard = threading.Lock()
        self.jobs = {}
        self.requests = {}
        self.children = {}
        for robot, command in commands.items():
            if (
                robot not in profiles
                or not isinstance(command, list)
                or not command
                or not all(isinstance(part, str) and part for part in command)
            ):
                raise ValueError(
                    "Session commands must be nonempty argument lists for configured arms"
                )

    def snapshot(self, robot):
        with self.guard:
            return dict(
                self.jobs.get(robot, dict(busy=False, phase="idle", message="")),
                reset_available=robot in self.commands,
            )

    def update(self, robot, **values):
        with self.guard:
            self.jobs[robot].update(values, updated_at=time.time())

    def start(self, robot, request_id):
        try:
            uuid.UUID(request_id)
        except (ValueError, TypeError, AttributeError):
            raise ValueError("A valid reset request ID is required")
        with self.guard:
            key = (robot, request_id)
            if key in self.requests:
                return dict(self.requests[key])
            if robot not in self.commands:
                raise ValueError("Session reset is not configured on this dashboard")
            if self.jobs.get(robot, {}).get("busy"):
                raise RuntimeError("This arm is already resetting")
            directory = Path(self.profiles[robot]["directory"])
            directory.mkdir(parents=True, exist_ok=True)
            reset_lock = (directory / "dashboard-reset.lock").open("a")
            try:
                fcntl.flock(reset_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                reset_lock.close()
                raise RuntimeError("This arm is already resetting in another dashboard")
            job = dict(
                request_id=request_id,
                busy=True,
                phase="releasing",
                message="Releasing motors and waiting for the current session to close…",
                updated_at=time.time(),
            )
            self.jobs[robot] = self.requests[key] = job
            # Retain bounded idempotency history, including the current request.
            while len(self.requests) > 100:
                self.requests.pop(next(iter(self.requests)))
            threading.Thread(
                target=self.run, args=(robot, reset_lock), daemon=True
            ).start()
            return dict(job)

    def run(self, robot, reset_lock):
        directory = Path(self.profiles[robot]["directory"])
        status_path = directory / "gripper-control-status.json"
        command_path = directory / "operator-command.json"
        held_lock = None
        try:
            held_lock = controller_lock(directory)
            release_id = None
            if held_lock is None:
                requested_at = time.time()
                self.send_release(robot)
                release_command = read_json(command_path)
                if (
                    release_command.get("action") != "release"
                    or release_command.get("at", 0) < requested_at
                ):
                    raise RuntimeError(
                        "Release request could not be confirmed; reset stopped"
                    )
                release_id = release_command.get("id")
                deadline = time.monotonic() + self.release_timeout
                while time.monotonic() < deadline:
                    held_lock = controller_lock(directory)
                    if held_lock is not None:
                        break
                    time.sleep(0.1)
                if held_lock is None:
                    raise RuntimeError(
                        "Current controller has not closed. Reset stopped; no second controller was started."
                    )
                status = read_json(status_path)
                if (
                    status.get("robot_id") != robot
                    or status.get("mode") != "off"
                    or status.get("enabled") is not False
                    or status.get("updated_at", 0) < requested_at
                ):
                    raise RuntimeError(
                        "Motor release was not verified. Reset stopped; support the arm."
                    )
            elif read_json(status_path).get("enabled"):
                raise RuntimeError(
                    "Previous controller stopped while motors may still hold. Release must be resolved before restarting."
                )
            # Remove only our release or an expired command, under the controller lock.
            command = read_json(command_path)
            if command:
                if (
                    command.get("id") != release_id
                    and time.time() - command.get("at", 0) < 2
                ):
                    raise RuntimeError(
                        "A newer operator command arrived. Reset cancelled."
                    )
                command_path.unlink(missing_ok=True)
            self.update(
                robot,
                phase="starting",
                message="Starting a fresh session and checking the arm…",
            )
            started_at = time.time()
            # The controller takes this same lock before constructing/opening its bus.
            # A competing manual start can win; the losing controller exits before serial access.
            held_lock.close()
            held_lock = None
            previous = self.children.get(robot)
            if previous is not None:
                previous.poll()
            with (directory / "dashboard-session.log").open("ab", buffering=0) as log:
                child = subprocess.Popen(
                    self.commands[robot],
                    cwd=self.cwd,
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    close_fds=True,
                    start_new_session=True,
                )
            self.children[robot] = child
            deadline = time.monotonic() + self.startup_timeout
            while time.monotonic() < deadline:
                code = child.poll()
                if code is not None:
                    raise RuntimeError(
                        f"Controller startup stopped (exit {code}). See dashboard-session.log for the arm; reset did not complete."
                    )
                status = read_json(status_path)
                if (
                    status.get("robot_id") == robot
                    and status.get("controller_pid") == child.pid
                    and status.get("updated_at", 0) >= started_at
                ):
                    if status.get("fault") or status.get("mode") in ("fault", "off"):
                        raise RuntimeError(
                            "New session reported: " + status.get("message", "a fault")
                        )
                    if status.get("mode") in ("ready", "paused", "holding", "active"):
                        self.update(
                            robot,
                            phase="complete",
                            message="Session reset. "
                            + status.get("message", "Release B1, then hold to move."),
                        )
                        return
                time.sleep(0.1)
            raise RuntimeError(
                "Startup is still unconfirmed. A controller may be running; check its status before another reset."
            )
        except Exception as error:
            self.update(robot, phase="failed", message=str(error))
        finally:
            if held_lock is not None:
                held_lock.close()
            reset_lock.close()
            self.update(robot, busy=False)
