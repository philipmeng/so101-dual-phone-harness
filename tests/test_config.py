import copy
from pathlib import Path
import tempfile
import unittest
import phone_profiles as profiles


class ConfigTest(unittest.TestCase):
    def setUp(self):
        self.original = copy.deepcopy(profiles.PROFILES)
        self.state, self.model = profiles.STATE_ROOT, profiles.MODEL_PATH
        self.example = (profiles.ROOT / "config.example.toml").read_text()

    def tearDown(self):
        profiles.PROFILES.clear()
        profiles.PROFILES.update(self.original)
        profiles.STATE_ROOT, profiles.MODEL_PATH = self.state, self.model

    def load(self, text):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "config.toml"
            p.write_text(text)
            profiles.load_config(p)
            return Path(d).resolve()

    def test_paths_resolve_relative_to_config_not_current_directory(self):
        base = self.load(self.example)
        self.assertEqual(
            profiles.profile("follower_1")["directory"], base / ".state/follower_1"
        )
        self.assertEqual(
            profiles.profile("follower_2")["calibration"],
            base / "calibration/follower_2.json",
        )

    def test_duplicate_ports_phones_boards_and_calibration_are_rejected(self):
        for old, new in [
            ("8766", "8765"),
            ("mobileIO-follower1", "mobileIO"),
            ("REPLACE_WITH_FIRST_ARM_ADAPTER", "REPLACE_WITH_SECOND_ARM_ADAPTER"),
            ("calibration/follower_1.json", "calibration/follower_2.json"),
        ]:
            with self.subTest(old=old), self.assertRaises(ValueError):
                self.load(self.example.replace(old, new))
            self.assertEqual(profiles.PROFILES, self.original)

    def test_serial_aliases_cannot_assign_one_board_to_two_arms(self):
        with tempfile.TemporaryDirectory() as d:
            a = Path(d) / "a"
            b = Path(d) / "b"
            a.touch()
            b.symlink_to(a)
            text = self.example.replace(
                "/dev/serial/by-id/REPLACE_WITH_FIRST_ARM_ADAPTER", str(a)
            ).replace("/dev/serial/by-id/REPLACE_WITH_SECOND_ARM_ADAPTER", str(b))
            with self.assertRaises(ValueError):
                self.load(text)


if __name__ == "__main__":
    unittest.main()
