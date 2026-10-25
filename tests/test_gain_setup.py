import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from check_arm import set_lift_gain


class FakeSession:
    def __init__(self):
        self.values = {}
        self.writes = []
        self.powered = False
        self.fail_once = False

    def read(self, key, motor):
        if key == "Torque_Enable":
            return int(self.powered)
        return self.values.get((key, motor), 16 if key == "P_Coefficient" else 0)

    def write(self, key, motor, value):
        if (
            self.fail_once
            and motor == "elbow_flex"
            and key == "P_Coefficient"
            and value == 32
        ):
            self.fail_once = False
            raise RuntimeError("simulated communication failure")
        self.writes.append((key, motor, value))
        self.values[key, motor] = value


class GainTest(unittest.TestCase):
    def test_powered_arm_is_never_modified(self):
        s = FakeSession()
        s.powered = True
        with self.assertRaises(RuntimeError):
            set_lift_gain(s, 32)
        self.assertEqual(s.writes, [])

    def test_only_lift_gains_and_eprom_lock_are_written(self):
        s = FakeSession()
        result = set_lift_gain(s, 32)
        self.assertEqual(result["after"], {"shoulder_lift": 32, "elbow_flex": 32})
        self.assertTrue(all(k in ("Lock", "P_Coefficient") for k, n, v in s.writes))
        self.assertTrue(
            all(n in ("shoulder_lift", "elbow_flex") for k, n, v in s.writes)
        )

    def test_partial_failure_restores_original_gains(self):
        s = FakeSession()
        s.fail_once = True
        with self.assertRaisesRegex(RuntimeError, "simulated"):
            set_lift_gain(s, 32)
        for n in ("shoulder_lift", "elbow_flex"):
            self.assertEqual(s.read("P_Coefficient", n), 16)


if __name__ == "__main__":
    unittest.main()
