"""Offline phone-to-arm isolation tests; no HEBI subscriptions or serial opens."""

import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import phone_control as control
import phone_input_preview as receiver
from phone_profiles import exclusive_file, phone_lock_path, profile, select_phone


class ProfileTest(unittest.TestCase):
    def entry(self, name="mobileIO", mac="02:00:00:00:00:01", stale=False):
        return SimpleNamespace(
            family="HEBI", name=name, mac_address=mac, is_stale=stale
        )

    def test_profiles_have_separate_control_channels(self):
        one, two = profile("follower_1"), profile("follower_2")
        for key in ("name", "directory", "port"):
            self.assertNotEqual(one[key], two[key])
        self.assertEqual(two["port"], 8765)
        self.assertEqual(two["name"], "mobileIO")

    def test_each_phone_is_selected_by_exact_identity(self):
        entries = [self.entry(), self.entry("mobileIO-follower1", "02:00:00:00:00:02")]
        self.assertEqual(select_phone(entries, "HEBI", "mobileIO"), "02:00:00:00:00:01")
        self.assertEqual(
            select_phone(entries, "HEBI", "mobileIO-follower1"), "02:00:00:00:00:02"
        )
        self.assertIsNone(select_phone(entries, "OTHER", "mobileIO"))
        self.assertIsNone(select_phone([self.entry(stale=True)], "HEBI", "mobileIO"))

    def test_ambiguous_names_and_phone_swaps_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "Multiple phones"):
            select_phone(
                [self.entry(), self.entry(mac="02:00:00:00:00:02")], "HEBI", "mobileIO"
            )
        with self.assertRaisesRegex(RuntimeError, "different phone"):
            select_phone([self.entry()], "HEBI", "mobileIO", "02:00:00:00:00:02")

    def test_same_device_cannot_be_owned_by_two_receivers(self):
        self.assertEqual(phone_lock_path("AA:BB"), phone_lock_path("aa:bb"))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "device.lock"
            owner = exclusive_file(path)
            try:
                with self.assertRaises(BlockingIOError):
                    exclusive_file(path)
            finally:
                owner.close()
            exclusive_file(path).close()

    def test_controller_rejects_other_arms_phone_even_when_fresh(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            sample = dict(
                robot_id="follower_2",
                phone_family="HEBI",
                phone_name="mobileIO",
                received_at=100,
                connected=True,
                tracking=True,
                b1=True,
                a3=0,
                delta_cm=[0, 0, 1],
            )
            (path / "input-status.json").write_text(json.dumps(sample))
            with (
                patch.object(control, "OUT", path),
                patch.object(control.time, "time", return_value=100.1),
            ):
                with patch.object(control, "ROBOT_ID", "follower_2"):
                    self.assertTrue(control.phone_input()["b1"])
                with patch.object(control, "ROBOT_ID", "follower_1"):
                    with self.assertRaisesRegex(RuntimeError, "different arm"):
                        control.phone_input()

    def handler(self, origin):
        h = receiver.Handler.__new__(receiver.Handler)
        body = json.dumps(dict(action="pause")).encode()
        h.path = "/control"
        h.server = SimpleNamespace(server_port=8766)
        h.headers = {
            "Origin": origin,
            "Content-Type": "application/json",
            "Content-Length": str(len(body)),
        }
        h.rfile = io.BytesIO(body)
        h.send_error = Mock()
        h.send_response = Mock()
        h.end_headers = Mock()
        return h

    def test_dashboard_controls_only_its_own_directory(self):
        with tempfile.TemporaryDirectory() as d:
            one = Path(d) / "one"
            two = Path(d) / "two"
            one.mkdir()
            two.mkdir()
            other = two / "operator-command.json"
            other.write_text("unchanged")
            with patch.object(receiver, "OUT", one / "input-status.json"):
                wrong = self.handler("http://127.0.0.1:8765")
                wrong.do_POST()
                wrong.send_error.assert_called_once_with(403)
                self.assertFalse((one / "operator-command.json").exists())
                right = self.handler("http://127.0.0.1:8766")
                right.do_POST()
                right.send_response.assert_called_once_with(204)
                self.assertEqual(
                    json.loads((one / "operator-command.json").read_text())["action"],
                    "pause",
                )
                self.assertEqual(other.read_text(), "unchanged")


if __name__ == "__main__":
    unittest.main()
