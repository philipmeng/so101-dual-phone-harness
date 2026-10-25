"""Offline control-transition tests: no serial connection or motor commands."""

import unittest
import numpy as np
from types import SimpleNamespace
from unittest.mock import Mock, patch
import phone_control as control


def sample(held):
    return dict(b1=held, a3=0, delta_cm=[1, 0, 0])


class ControlTest(unittest.TestCase):
    def test_exclusive_port_lock_precedes_use_and_reapplies_on_open(self):
        port = SimpleNamespace(
            openPort=Mock(return_value=True),
            closePort=Mock(),
            ser=SimpleNamespace(fileno=lambda: 17),
        )
        with (
            patch.object(control.sys, "platform", "linux"),
            patch.object(control.fcntl, "ioctl") as ioctl,
        ):
            control.protect_serial_port(port)
            self.assertTrue(port.openPort())
            self.assertTrue(port.openPort())
            self.assertEqual(ioctl.call_count, 2)
            ioctl.assert_called_with(17, control.termios.TIOCEXCL)
        port.closePort.assert_not_called()

    def test_exclusive_lock_failure_closes_port_and_aborts(self):
        port = SimpleNamespace(
            openPort=Mock(return_value=True),
            closePort=Mock(),
            ser=SimpleNamespace(fileno=lambda: 17),
        )
        with (
            patch.object(control.sys, "platform", "linux"),
            patch.object(control.fcntl, "ioctl", side_effect=OSError("lock failed")),
        ):
            control.protect_serial_port(port)
            with self.assertRaisesRegex(OSError, "lock failed"):
                port.openPort()
        port.closePort.assert_called_once()

    def test_failed_hold_is_never_reported_as_verified_holding(self):
        s = self.make_session([None, "release"])
        s.check = Mock(side_effect=control.FeedbackUnstable("no replies"))
        s.freeze = Mock(side_effect=RuntimeError("no position replies"))
        with patch.object(control.time, "sleep"):
            s.run()
        self.assertTrue(
            any("HOLD UNVERIFIED" in c.args[0] for c in s.state.call_args_list)
        )
        self.assertFalse(
            any("FEEDBACK PAUSE — HOLDING" in c.args[0] for c in s.state.call_args_list)
        )
        s.enable.assert_not_called()
        s.write.assert_not_called()

    def test_corrupt_feedback_is_recoverable_but_never_used(self):
        s = self.make_session([])
        s.bus = SimpleNamespace(
            packet_handler=SimpleNamespace(readTxRx=Mock(return_value=([], -6, 0))),
            port_handler=None,
            motors={"shoulder_pan": SimpleNamespace(id=1)},
        )
        with self.assertRaises(control.FeedbackUnstable):
            s.feedback("shoulder_pan")
        s.write.assert_not_called()

    def test_failed_handshake_is_bounded_and_never_releases_or_writes(self):
        s = control.Session.__new__(control.Session)
        s.bus = Mock(is_connected=True)
        s.bus.connect.side_effect = RuntimeError("No response")
        with (
            patch.object(control.time, "sleep"),
            self.assertRaisesRegex(RuntimeError, "No response"),
        ):
            s.connect()
        self.assertEqual(s.bus.connect.call_count, 3)
        self.assertEqual(s.bus.disconnect.call_count, 3)
        for call in s.bus.disconnect.call_args_list:
            self.assertEqual(call.kwargs, {"disable_torque": False})
        s.bus.write.assert_not_called()
        s.bus.enable_torque.assert_not_called()

    def make_session(self, commands):
        s = control.Session.__new__(control.Session)
        s.powered = False
        s.touched = False
        s.released = False
        s.previous_b1 = True
        s.pause_requested = True
        s.fault = None
        s.recoverable = False
        s.good_feedback_checks = 0
        s.hard_fault = False
        s.reach_limited = False
        s.goals = {n: 2000 for n in control.CAPS}
        s.robot = SimpleNamespace(
            calibration={
                n: SimpleNamespace(range_min=1000, range_max=3000) for n in control.CAPS
            }
        )
        s.mapper = SimpleNamespace(
            latch=Mock(), target=Mock(return_value={n: 2100 for n in control.BODY})
        )
        s.connect = Mock()
        s.command = Mock(side_effect=commands)
        s.check = Mock(side_effect=lambda: dict(s.goals))
        s.state = Mock()
        s.event = Mock()
        s.freeze = Mock()
        s.write = Mock()

        def enable(_):
            s.powered = True

        def release():
            s.powered = False
            s.released = True

        s.enable = Mock(side_effect=enable)
        s.release = Mock(side_effect=release)
        return s

    def test_release_button_holds_then_operator_release_ends(self):
        s = self.make_session([None, None, None, "release"])
        with (
            patch.object(
                control,
                "phone_input",
                side_effect=[
                    sample(False),
                    sample(True),
                    sample(True),
                    sample(True),
                    sample(False),
                ],
            ),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        s.enable.assert_called_once()
        s.release.assert_called_once()
        self.assertTrue(any(c.args[1] == "holding" for c in s.state.call_args_list))
        self.assertGreaterEqual(s.freeze.call_count, 2)

    def test_startup_held_button_does_not_enable(self):
        s = self.make_session([None, None, "release"])
        with (
            patch.object(control, "phone_input", return_value=sample(True)),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        s.enable.assert_not_called()
        s.write.assert_not_called()

    def test_phone_loss_pauses_and_requires_button_release(self):
        s = self.make_session([None, None, None, None, None, "release"])
        inputs = [
            sample(False),
            sample(True),
            sample(True),
            sample(True),
            RuntimeError("stale"),
            sample(True),
            sample(False),
        ]
        with (
            patch.object(control, "phone_input", side_effect=inputs),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        s.enable.assert_called_once()
        s.mapper.target.assert_called_once()
        self.assertTrue(any("PHONE LOST" in c.args[0] for c in s.state.call_args_list))

    def test_health_fault_releases_without_rearming(self):
        s = self.make_session([None, None, "release"])
        s.check = Mock(side_effect=control.HealthFault("overheat"))
        with patch.object(control.time, "sleep"):
            s.run()
        s.release.assert_called_once()
        s.enable.assert_not_called()
        self.assertEqual(s.check.call_count, 1)

    def test_unstable_feedback_holds_and_rearms_only_after_good_checks_and_release(
        self,
    ):
        s = self.make_session([None] * 7 + ["release"])
        calls = 0

        def check():
            nonlocal calls
            calls += 1
            if calls == 4:
                raise control.FeedbackUnstable("inconsistent readings")
            return dict(s.goals)

        s.check = Mock(side_effect=check)
        inputs = [
            sample(False),
            sample(True),
            sample(True),
            sample(True),
            sample(True),
            sample(False),
            sample(False),
            sample(True),
            sample(True),
            sample(True),
        ]
        with (
            patch.object(control, "phone_input", side_effect=inputs),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        s.enable.assert_called_once()
        s.release.assert_called_once()
        self.assertEqual(s.mapper.target.call_count, 2)
        self.assertTrue(
            any(c.args[0] == "feedback_recovered" for c in s.event.call_args_list)
        )

    def test_partial_enable_failure_calls_release(self):
        s = self.make_session([None, None, "release"])
        s.enable = Mock(side_effect=RuntimeError("partial write"))
        with (
            patch.object(
                control, "phone_input", side_effect=[sample(False), sample(True)]
            ),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        self.assertGreaterEqual(s.release.call_count, 1)
        s.write.assert_not_called()

    def test_speed_limit_has_no_demo_travel_radius(self):
        goals = {"a": 1000, "b": 2000}
        target = {"a": 3000, "b": 2500}
        for _ in range(400):
            step = control.step_toward(goals, target, 0.1)
            self.assertTrue(all(abs(step[n] - goals[n]) <= 8 for n in goals))
            goals = step
        self.assertEqual(goals, target)
        self.assertEqual(control.step_toward(goals, {"a": 2000, "b": 2000}, 0), goals)

    def test_lagging_motor_cannot_accumulate_distant_goals(self):
        goals = {"a": 1000, "b": 2000}
        positions = dict(goals)
        target = {"a": 3000, "b": 1000}
        for _ in range(100):
            new = control.step_with_feedback(goals, target, positions, 0.1)
            self.assertTrue(all(abs(new[n] - goals[n]) <= 8 for n in goals))
            self.assertTrue(all(abs(new[n] - positions[n]) <= 32 for n in goals))
            goals = new
        self.assertEqual(goals["a"], 1032)
        self.assertEqual(
            control.step_with_feedback(goals, target, positions, 0.1), goals
        )

    def test_feedback_pacing_continues_when_motor_catches_up(self):
        goals = {"a": 1000, "b": 2000}
        target = {"a": 1200, "b": 1800}
        positions = dict(goals)
        for _ in range(250):
            goals = control.step_with_feedback(goals, target, positions, 0.1)
            self.assertTrue(all(abs(goals[n] - positions[n]) <= 32 for n in goals))
            positions = {
                n: p + max(-2, min(2, goals[n] - p)) for n, p in positions.items()
            }
        self.assertEqual(goals, target)
        self.assertEqual(positions, target)

    def test_feedback_pacing_allows_reversal_without_jumping(self):
        goals = {"a": 1035}
        positions = {"a": 1000}
        self.assertEqual(
            control.step_with_feedback(goals, {"a": 1100}, positions, 0.1), goals
        )
        self.assertEqual(
            control.step_with_feedback(goals, {"a": 900}, positions, 0.1), {"a": 1027}
        )
        self.assertEqual(
            control.step_with_feedback(goals, {"a": 900}, positions, 0), goals
        )

    def test_stalled_body_does_not_block_gripper(self):
        goals = {"shoulder_lift": 968, "gripper": 2000}
        positions = {"shoulder_lift": 1000, "gripper": 2000}
        result = control.step_with_feedback(
            goals, {"shoulder_lift": 900, "gripper": 2200}, positions, 0.1
        )
        self.assertEqual(result, {"shoulder_lift": 968, "gripper": 2008})

    def test_twenty_tick_following_error_does_not_deadlock(self):
        goals = {"shoulder_lift": 1000}
        positions = {"shoulder_lift": 1020}
        for _ in range(30):
            goals = control.step_with_feedback(
                goals, {"shoulder_lift": 900}, positions, 0.1
            )
            positions = {"shoulder_lift": goals["shoulder_lift"] + 20}
        self.assertEqual(goals["shoulder_lift"], 900)

    def test_path_uses_commanded_pose_when_measured_pose_lags(self):
        s = self.make_session([None, None, "release"])
        s.powered = True
        s.pause_requested = False
        s.previous_b1 = True
        s.check = Mock(return_value={n: 2020 for n in control.CAPS})
        seeds = []

        def target(seed, delta):
            seeds.append(dict(seed))
            return {n: 2100 for n in control.BODY}

        s.mapper.target = Mock(side_effect=target)
        with (
            patch.object(control, "phone_input", return_value=sample(True)),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        self.assertEqual(seeds[0], {n: 2000 for n in control.CAPS})

    def test_paused_hold_is_not_relatched_each_loop(self):
        s = self.make_session([None, None, None, "release"])
        s.powered = True
        with (
            patch.object(control, "phone_input", return_value=sample(True)),
            patch.object(control.time, "sleep"),
        ):
            s.run()
        s.freeze.assert_called_once()

    def test_exact_hardware_fault_details_are_recorded(self):
        for temperature, status, expected in [
            (70, 0, "temperature=70"),
            (35, 32, "status=0x20"),
        ]:
            s = self.make_session([])
            data = [0] * 15
            data[0] = 208
            data[1] = 7
            data[6] = 123
            data[7] = temperature
            data[9] = status
            s.bus = SimpleNamespace(
                packet_handler=SimpleNamespace(
                    readTxRx=Mock(return_value=(data, 0, 0))
                ),
                port_handler=None,
                motors={"elbow_flex": SimpleNamespace(id=3)},
            )
            with self.assertRaisesRegex(control.HealthFault, expected):
                control.Session.feedback(s, "elbow_flex")
            self.assertEqual(s.event.call_args.args[0], "motor_fault_snapshot")
            self.assertEqual(s.event.call_args.kwargs["temperature"], temperature)

    def test_voltage_window_and_current_protection(self):
        for voltage, current, valid in [
            (109, 0, True),
            (90, 0, True),
            (126, 0, True),
            (89, 0, False),
            (127, 0, False),
            (123, 100, False),
        ]:
            s = self.make_session([])
            data = [0] * 15
            data[0] = 208
            data[1] = 7
            data[6] = voltage
            data[7] = 35
            data[13] = current
            s.bus = SimpleNamespace(
                packet_handler=SimpleNamespace(
                    readTxRx=Mock(return_value=(data, 0, 0))
                ),
                port_handler=None,
                motors={"gripper": SimpleNamespace(id=6)},
            )
            s.read = Mock(return_value=0)
            if valid:
                self.assertEqual(
                    control.Session.feedback(s, "gripper")["voltage"], voltage
                )
            else:
                with self.assertRaises(control.HealthFault):
                    control.Session.feedback(s, "gripper")

    def test_health_uses_full_packets_without_extra_temperature_read(self):
        s = self.make_session([])
        s.feedback = Mock(return_value=dict(position=2000, temperature=30))
        s.read = Mock(side_effect=AssertionError("Unexpected separate register read"))
        self.assertEqual(control.Session.check(s), {n: 2000 for n in control.CAPS})
        self.assertEqual(s.feedback.call_count, len(control.CAPS))

    def test_warm_full_packet_gets_two_fresh_health_checks(self):
        s = self.make_session([])
        s.feedback = Mock(
            side_effect=[
                dict(position=2000, temperature=55),
                dict(position=2001, temperature=34),
                dict(position=2002, temperature=34),
            ]
            + [dict(position=2000, temperature=30)] * 5
        )
        s.read = Mock(side_effect=AssertionError("Unexpected separate register read"))
        result = control.Session.check(s)
        self.assertEqual(result["shoulder_pan"], 2002)
        self.assertEqual(s.feedback.call_count, 8)

    def test_sustained_hot_full_packets_still_stop(self):
        s = self.make_session([])
        s.feedback = Mock(return_value=dict(position=2000, temperature=61))
        with self.assertRaises(control.HealthFault):
            control.Session.check(s)


class ReachTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Synthetic calibration: no user's motor calibration is published.
        calibration = {
            n: SimpleNamespace(range_min=0, range_max=4095) for n in control.BODY
        }
        cls.mapper = control.Mapper(calibration)
        angles = np.array([6.2, 53.71, -16.97, -3.47, 79.87])
        cls.start = dict(
            zip(control.BODY, np.rint(2047.5 + angles * 4095 / 360).astype(int))
        )

    def test_large_reachable_targets_converge(self):
        # With the current calibration, +10 cm phone travel asks this starting
        # pose to extend beyond its IK reach. +8 cm remains reachable.
        for delta in [[-20, 0, 0], [8, 0, 0], [0, 20, 0], [0, -20, 0], [0, 0, 20]]:
            m = self.mapper
            m.latch(self.start)
            p = self.start.copy()
            destination = m.reference[:3, 3] + np.asarray(delta) * 0.005
            for _ in range(200):
                target = m.target(p, delta)
                self.assertIsNotNone(target)
                new = control.step_toward(p, target, 0.1)
                self.assertLessEqual(max(abs(new[n] - p[n]) for n in p), 8)
                if new == p:
                    break
                p = new
            actual = m.kin.forward_kinematics(m.angles(p))[:3, 3]
            self.assertLess(np.linalg.norm(actual - destination), 0.002)
            self.assertGreater(max(abs(p[n] - self.start[n]) for n in p), 100)

    def test_unreachable_target_stops_within_calibration(self):
        m = self.mapper
        m.latch(self.start)
        p = self.start.copy()
        for _ in range(200):
            target = m.target(p, [200, 0, 0])
            if target is None:
                break
            raw = np.array([target[n] for n in control.BODY])
            self.assertTrue(np.all(raw >= m.low) and np.all(raw <= m.high))
            p = control.step_toward(p, target, 0.1)
        else:
            self.fail("Unreachable target was never rejected")

    def test_upward_path_does_not_accumulate_shoulder_sag(self):
        m = self.mapper
        angles = np.array([-0.923, 0.703, 3.868, 0.483, 2.154])
        goals = dict(
            zip(control.BODY, np.rint(2047.5 + angles * 4095 / 360).astype(int))
        )
        positions = dict(goals)
        m.latch(positions)
        start = m.reference[:3, 3].copy()
        for _ in range(100):
            target = m.target(goals, [0, 0, 10])
            self.assertIsNotNone(target)
            goals = control.step_with_feedback(goals, target, positions, 0.1)
            positions = dict(goals)
            positions["shoulder_lift"] += 20
        delta = m.kin.forward_kinematics(m.angles(positions))[:3, 3] - start
        self.assertGreater(delta[2], 0.03)
        self.assertLess(abs(delta[0]), 0.01)

    def test_neutral_and_invalid_inputs(self):
        m = self.mapper
        m.latch(self.start)
        self.assertEqual(m.target(self.start, [0, 0, 0]), self.start)
        self.assertIsNone(m.target(self.start, [float("nan"), 0, 0]))
        self.assertIsNone(m.target(self.start, [0, 0]))


if __name__ == "__main__":
    unittest.main()
