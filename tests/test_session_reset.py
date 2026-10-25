"""Restart sequencing against real file locks and fake controller subprocesses."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

from session_reset import SessionReset, controller_lock

# The fake owns the same lock the real controller acquires before any serial access.
CHILD = r"""
import fcntl, json, os, pathlib, sys, time
folder=pathlib.Path(sys.argv[1]); robot=sys.argv[2]
lock=(folder/'controller.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
def state(mode,enabled,message):
    p=folder/'gripper-control-status.json'
    tmp=p.with_suffix('.tmp')
    tmp.write_text(json.dumps(dict(robot_id=robot,controller_pid=os.getpid(),mode=mode,enabled=enabled,message=message,updated_at=time.time())))
    tmp.replace(p)
state('ready',False,'READY — HOLD B1 TO MOVE')
while True:
    try: command=json.loads((folder/'operator-command.json').read_text())
    except (OSError, ValueError): command={}
    if command.get('action')=='release':
        state('off',False,'MOTORS OFF — SESSION ENDED')
        break
    time.sleep(.02)
"""


class ResetTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.profiles = {
            r: {"directory": self.root / r} for r in ("follower_1", "follower_2")
        }
        for p in self.profiles.values():
            p["directory"].mkdir()
        self.script = self.root / "fake.py"
        self.script.write_text(CHILD)
        self.commands = {
            r: [sys.executable, str(self.script), str(p["directory"]), r]
            for r, p in self.profiles.items()
        }
        self.releases = []
        self.manager = SessionReset(
            self.profiles,
            self.commands,
            self.root,
            self.release,
            release_timeout=0.4,
            startup_timeout=1.5,
        )
        self.addCleanup(self.cleanup_children)

    def cleanup_children(self):
        for child in self.manager.children.values():
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=2)

    def release(self, robot):
        self.releases.append(robot)
        (self.profiles[robot]["directory"] / "operator-command.json").write_text(
            json.dumps(dict(id=time.time_ns(), at=time.time(), action="release"))
        )

    def wait(self, manager=None, robot="follower_1"):
        manager = manager or self.manager
        deadline = time.monotonic() + 3
        while manager.snapshot(robot)["busy"] and time.monotonic() < deadline:
            time.sleep(0.02)
        result = manager.snapshot(robot)
        self.assertFalse(result["busy"], result)
        return result

    def start(self, manager=None):
        manager = manager or self.manager
        return manager.start("follower_1", str(uuid.uuid4()))

    def test_start_ended_session_and_duplicate_request(self):
        token = str(uuid.uuid4())
        self.manager.start("follower_1", token)
        self.assertEqual(self.wait()["phase"], "complete")
        pid = self.manager.children["follower_1"].pid
        self.manager.start("follower_1", token)
        self.assertEqual(self.manager.children["follower_1"].pid, pid)
        self.assertEqual(self.releases, [])
        self.assertNotIn("follower_2", self.manager.children)

    def test_release_old_owner_before_new_controller_and_clear_release(self):
        old = subprocess.Popen(self.commands["follower_1"])
        self.addCleanup(lambda: old.poll() is None and old.terminate())
        deadline = time.monotonic() + 2
        path = self.profiles["follower_1"]["directory"] / "gripper-control-status.json"
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.start()
        self.assertEqual(self.wait()["phase"], "complete")
        old.wait(timeout=2)
        self.assertEqual(self.releases, ["follower_1"])
        self.assertFalse((path.parent / "operator-command.json").exists())
        self.assertNotEqual(self.manager.children["follower_1"].pid, old.pid)

    def test_stuck_owner_is_never_killed_or_replaced(self):
        held = controller_lock(self.profiles["follower_1"]["directory"])
        self.addCleanup(held.close)
        self.start()
        result = self.wait()
        self.assertEqual(result["phase"], "failed")
        self.assertIn("has not closed", result["message"])
        self.assertFalse(self.manager.children)

    def test_missing_release_confirmation_blocks_restart(self):
        folder = self.profiles["follower_1"]["directory"]
        held = controller_lock(folder)

        def no_confirmation(robot):
            self.release(robot)
            held.close()

        self.manager.send_release = no_confirmation
        self.start()
        self.assertEqual(self.wait()["phase"], "failed")
        self.assertFalse(self.manager.children)

    def test_uncertain_holding_state_and_fresh_command_block_start(self):
        folder = self.profiles["follower_1"]["directory"]
        (folder / "gripper-control-status.json").write_text(
            json.dumps(dict(enabled=True))
        )
        self.start()
        self.assertIn("may still hold", self.wait()["message"])
        (folder / "gripper-control-status.json").unlink()
        self.release("follower_1")
        self.start()
        self.assertIn("newer operator command", self.wait()["message"])
        self.assertFalse(self.manager.children)

    def test_start_failure_is_not_reported_ready_from_old_status(self):
        self.manager.commands["follower_1"] = [
            sys.executable,
            "-c",
            "raise SystemExit(7)",
        ]
        self.start()
        result = self.wait()
        self.assertEqual(result["phase"], "failed")
        self.assertIn("exit 7", result["message"])

    def test_fresh_status_from_another_process_cannot_confirm_startup(self):
        self.manager.commands["follower_1"] = [
            sys.executable,
            "-c",
            "import time; time.sleep(5)",
        ]
        self.manager.startup_timeout = 0.15
        folder = self.profiles["follower_1"]["directory"]
        (folder / "gripper-control-status.json").write_text(
            json.dumps(
                dict(
                    robot_id="follower_1",
                    controller_pid=0,
                    mode="ready",
                    enabled=False,
                    updated_at=time.time() + 1,
                )
            )
        )
        self.start()
        result = self.wait()
        self.assertEqual(result["phase"], "failed")
        self.assertIn("unconfirmed", result["message"])
        self.assertIsNone(self.manager.children["follower_1"].poll())

    def test_same_arm_concurrent_reset_rejected(self):
        held = controller_lock(self.profiles["follower_1"]["directory"])
        self.addCleanup(held.close)
        self.start()
        with self.assertRaises(RuntimeError):
            self.start()
        other = SessionReset(self.profiles, self.commands, self.root, self.release)
        with self.assertRaises(RuntimeError):
            self.start(other)
        self.wait()

    def test_invalid_request_id_or_command_rejected(self):
        with self.assertRaises(ValueError):
            self.manager.start("follower_1", "bad")
        with self.assertRaises(ValueError):
            SessionReset(
                self.profiles, {"follower_1": "shell command"}, self.root, self.release
            )


if __name__ == "__main__":
    unittest.main()
