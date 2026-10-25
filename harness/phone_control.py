"""Continuous operator-controlled SO101 session; no demonstration travel radius."""

import argparse
import fcntl
import json
import math
import os
import signal
import sys
import termios
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from so101_temperature_guard import (
    confirm_temperature_sample,
    TemperatureLimitError,
    TemperatureDataError,
)
from phone_profiles import PROFILES, load_config, model_path, profile

ROOT = Path(__file__).resolve().parents[1]
OUT = profile("follower_2")["directory"]
ROBOT_ID = "follower_2"
BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CAPS = dict(
    shoulder_pan=350,
    shoulder_lift=500,
    elbow_flex=500,
    wrist_flex=350,
    wrist_roll=250,
    gripper=150,
)
ROBOT_IDS = tuple(PROFILES)
# Conservative operating window for this verified 12 V kit, in tenths of a volt.
VOLTAGE_MIN = 90
VOLTAGE_MAX = 126


class HealthFault(RuntimeError):
    pass


class FeedbackUnstable(RuntimeError):
    pass


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def atomic(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def protect_serial_port(port_handler):
    """Block other Linux processes from opening this tty during the session."""
    if not sys.platform.startswith("linux"):
        return
    original_open = port_handler.openPort

    def exclusive_open():
        opened = original_open()
        if opened:
            try:
                fcntl.ioctl(port_handler.ser.fileno(), termios.TIOCEXCL)
            except Exception:
                port_handler.closePort()
                raise
        return opened

    port_handler.openPort = exclusive_open


def phone_input():
    s = json.loads((OUT / "input-status.json").read_text())
    expected = profile(ROBOT_ID)
    require(
        s.get("robot_id") == ROBOT_ID
        and s.get("phone_family") == expected["family"]
        and s.get("phone_name") == expected["name"],
        "Phone is assigned to a different arm",
    )
    require(
        0 <= time.time() - s.get("received_at", 0) < 0.3
        and s["connected"]
        and s["tracking"],
        "Phone input or tracking lost",
    )
    require(isinstance(s["b1"], bool), "Invalid B1 input")
    require(math.isfinite(s["a3"]) and -1.01 <= s["a3"] <= 1.01, "Invalid A3 input")
    require(
        np.asarray(s["delta_cm"]).shape == (3,) and np.isfinite(s["delta_cm"]).all(),
        "Invalid phone motion",
    )
    return s


def step_toward(goals, target, dt):
    """Limit speed, not overall travel. Preserve the joint-space direction."""
    maximum = max(abs(target[n] - goals[n]) for n in goals)
    allowance = min(8.0, max(0.0, dt) * 120.0)
    factor = min(1.0, allowance / maximum) if maximum else 1.0
    return {n: round(goals[n] + factor * (target[n] - goals[n])) for n in goals}


def step_with_feedback(goals, target, positions, dt):
    """Pace the body and gripper separately, below the 40-tick fault guard.

    Allow 32 ticks of command lead so a steady 20-tick following error does
    not deadlock the trajectory. Keep eight ticks of tracking-fault margin.
    """
    if "gripper" in goals and len(goals) > 1:
        body = {n: v for n, v in goals.items() if n != "gripper"}
        return {
            **step_with_feedback(body, target, positions, dt),
            **step_with_feedback({"gripper": goals["gripper"]}, target, positions, dt),
        }
    stepped = step_toward(goals, target, dt)
    factor = 1.0
    for n in goals:
        delta = stepped[n] - goals[n]
        if delta > 0:
            factor = min(factor, max(0.0, (positions[n] + 32 - goals[n]) / delta))
        elif delta < 0:
            factor = min(factor, max(0.0, (goals[n] - positions[n] + 32) / (-delta)))
    return {n: round(goals[n] + factor * (stepped[n] - goals[n])) for n in goals}


class Mapper:
    def __init__(self, calibration):
        self.calibration = calibration
        self.mid = np.array(
            [(calibration[n].range_min + calibration[n].range_max) / 2 for n in BODY]
        )
        self.low = np.array([calibration[n].range_min + 40 for n in BODY])
        self.high = np.array([calibration[n].range_max - 40 for n in BODY])
        self.kin = RobotKinematics(str(model_path()), joint_names=BODY)

    def angles(self, positions):
        return (np.array([positions[n] for n in BODY]) - self.mid) * 360 / 4095

    def latch(self, positions):
        self.reference = self.kin.forward_kinematics(self.angles(positions)).copy()

    def target(self, positions, delta_cm):
        delta_cm = np.asarray(delta_cm, dtype=float)
        if delta_cm.shape != (3,) or not np.isfinite(delta_cm).all():
            return None
        desired = self.reference.copy()
        # Half-scale translation; release/repress B1 to reposition the phone.
        desired[:3, 3] += delta_cm * 0.005
        q = self.angles(positions)
        current = self.kin.forward_kinematics(q)
        offset = desired[:3, 3] - current[:3, 3]
        distance = np.linalg.norm(offset)
        # Follow a distant target incrementally; this is not a total reach cap.
        if distance > 0.01:
            desired[:3, 3] = current[:3, 3] + offset * (0.01 / distance)
        # Let the wrist bend to reach the requested position.
        for _ in range(12):
            q = self.kin.inverse_kinematics(q, desired, orientation_weight=0.0)
        actual = self.kin.forward_kinematics(q)
        raw = q * 4095 / 360 + self.mid
        if (
            not np.isfinite(raw).all()
            or np.linalg.norm(actual[:3, 3] - desired[:3, 3]) > 0.002
            or np.any(raw < self.low)
            or np.any(raw > self.high)
        ):
            return None
        return dict(zip(BODY, np.rint(raw).astype(int).tolist()))


class Session:
    def __init__(self, robot_id="follower_2", port=None):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.robot_id = robot_id
        self.port = port or profile(robot_id)["serial_port"]
        require(bool(self.port), "Set this arm’s serial_port in config.toml")
        calibration = {
            n: MotorCalibration(**value)
            for n, value in json.loads(
                profile(robot_id)["calibration"].read_text()
            ).items()
        }
        # Use the same bus/motor configuration as SOFollower without importing
        # camera and training dependencies or invoking automatic calibration.
        self.bus = FeetechMotorsBus(
            port=self.port,
            calibration=calibration,
            motors={
                n: Motor(
                    i,
                    "sts3215",
                    MotorNormMode.RANGE_0_100
                    if n == "gripper"
                    else MotorNormMode.DEGREES,
                )
                for i, n in enumerate(CAPS, 1)
            },
        )
        self.robot = SimpleNamespace(calibration=calibration, bus=self.bus)
        protect_serial_port(self.bus.port_handler)
        self.mapper = Mapper(self.robot.calibration)
        self.powered = False
        self.touched = False
        self.saved = {}
        self.goals = {}
        self.caps = {}
        self.log = open(
            OUT / f"continuous-events-{time.time_ns()}.jsonl", "a", buffering=1
        )
        self.pause_requested = True
        self.fault = None
        self.last_command = None
        self.released = False
        self.previous_b1 = True
        self.recoverable = False
        self.good_feedback_checks = 0
        self.hard_fault = False
        self.reach_limited = False

    def event(self, kind, **data):
        self.log.write(json.dumps(dict(time=time.time(), kind=kind, **data)) + "\n")

    def state(self, message, mode):
        atomic(
            OUT / "gripper-control-status.json",
            dict(
                message=message,
                enabled=self.powered or (self.touched and not self.released),
                mode=mode,
                continuous=True,
                controller_pid=os.getpid(),
                updated_at=time.time(),
                fault=self.fault,
                robot_id=self.robot_id,
            ),
        )

    def read(self, key, n):
        return int(self.bus.read(key, n, normalize=False))

    def write(self, key, n, value):
        self.bus.write(key, n, int(value), normalize=False)

    def feedback(self, n):
        d, c, e = self.bus.packet_handler.readTxRx(
            self.bus.port_handler, self.bus.motors[n].id, 56, 15
        )
        if c != 0 or len(d) != 15:
            raise FeedbackUnstable(
                f"{n}: invalid feedback (communication={c}, length={len(d)})"
            )
        f = dict(
            position=d[0] + 256 * d[1],
            temperature=d[7],
            voltage=d[6],
            status=d[9],
            current=d[13] + 256 * d[14],
            speed_raw=d[2] + 256 * d[3],
            load_raw=d[4] + 256 * d[5],
        )
        if e or f["status"] or f["temperature"] >= 70:
            self.event("motor_fault_snapshot", motor=n, packet_error=e, **f)
            if e or f["status"]:
                raise HealthFault(
                    f"{n}: motor status=0x{f['status']:02x}, packet error=0x{e:02x}, temperature={f['temperature']} C"
                )
            raise HealthFault(
                f"{n}: reported temperature={f['temperature']} C reached stop threshold 70 C"
            )
        if not VOLTAGE_MIN <= f["voltage"] <= VOLTAGE_MAX or f["current"] >= (
            160 if n in ("shoulder_lift", "elbow_flex") else 100
        ):
            self.event("motor_fault_snapshot", motor=n, packet_error=e, **f)
            raise HealthFault(
                f"{n}: voltage={f['voltage'] / 10:.1f} V, current={f['current']} raw outside limits"
            )
        c = self.robot.calibration[n]
        require(
            c.range_min <= f["position"] <= c.range_max, f"{n}: outside calibration"
        )
        require(
            self.read("Torque_Enable", n) == int(self.powered),
            f"{n}: unexpected torque state",
        )
        if self.powered:
            require(
                self.read("Torque_Limit", n) == self.caps[n], f"{n}: output cap changed"
            )
            require(
                self.read("Goal_Position", n) == self.goals[n], f"{n}: goal mismatch"
            )
            if abs(f["position"] - self.goals[n]) > 40:
                self.event("tracking_error_snapshot", motor=n, goal=self.goals[n], **f)
                raise RuntimeError(
                    f"{n}: tracking error (position={f['position']}, goal={self.goals[n]})"
                )
        return f

    def check(self):
        positions = {}
        self.latest_feedback = {}
        for n in CAPS:
            f = self.feedback(n)
            self.latest_feedback[n] = f
            seen = []

            def reread():
                fresh = self.feedback(n)
                self.latest_feedback[n] = fresh
                positions[n] = fresh["position"]
                seen.append(fresh["temperature"])
                return fresh["temperature"]

            # Temperature comes from the same checked packet as position,
            # voltage and motor status. Do not add a separate one-byte read.
            first = f["temperature"]
            seen.append(first)
            try:
                confirmations = confirm_temperature_sample(
                    first, reread, hardware_limit=70, software_limit=60
                )
            except TemperatureLimitError as e:
                self.event("temperature_limit_snapshot", motor=n, readings=seen)
                raise HealthFault(f"{n}: {e}") from e
            except TemperatureDataError as e:
                self.event("temperature_data_snapshot", motor=n, readings=seen)
                raise FeedbackUnstable(f"{n}: {e}") from e
            if confirmations:
                self.event("temperature_confirmed", motor=n, readings=confirmations)
            positions.setdefault(n, f["position"])
        return positions

    def connect(self):
        # Some USB adapters miss the first handshake after opening. Retry only
        # the read-only handshake; never retry enable, goals or health failures.
        for attempt in range(3):
            try:
                self.bus.connect()
                break
            except Exception:
                if self.bus.is_connected:
                    self.bus.disconnect(disable_torque=False)
                if attempt == 2:
                    raise
                time.sleep(0.25)
        require(
            not any(self.bus.sync_read("Torque_Enable", normalize=False).values()),
            "Motors already powered; not taking over",
        )
        require(
            self.bus.read_calibration() == self.robot.calibration,
            "Calibration mismatch",
        )
        for n in CAPS:
            require(self.read("Operating_Mode", n) == 0, "Wrong motor mode")
            require(
                self.read("Max_Temperature_Limit", n) == 70
                and self.read("Unloading_Condition", n) & 4,
                "Motor protection mismatch",
            )
            self.saved[n] = {
                k: self.read(k, n)
                for k in ("Torque_Limit", "Goal_Velocity", "Acceleration")
            }
            self.caps[n] = min(CAPS[n], self.saved[n]["Torque_Limit"])
            require(self.caps[n] > 0, "Motor output disabled")
        self.check()
        self.event("connected_motors_off", robot_id=self.robot_id, port=self.port)

    def enable(self, positions):
        self.touched = True
        for n in CAPS:
            self.write("Torque_Limit", n, 0)
            require(self.read("Torque_Limit", n) == 0, "Cannot zero motor output")
        for n in CAPS:
            self.write("Acceleration", n, 10)
            self.write("Goal_Velocity", n, 120 if self.read("Phase", n) & 4 else 1)
            self.write("Goal_Position", n, positions[n])
            require(self.read("Goal_Position", n) == positions[n], "Stale motor goal")
        require(
            all(
                abs(self.read("Present_Position", n) - positions[n]) <= 5 for n in CAPS
            ),
            "Arm shifted while enabling",
        )
        require(phone_input()["b1"], "B1 released while enabling")
        self.goals = dict(positions)
        for n in CAPS:
            self.write("Torque_Limit", n, self.caps[n])
            self.bus.enable_torque(n)
        self.powered = True
        for _ in range(3):
            actual = self.check()
            require(
                all(abs(actual[n] - positions[n]) <= 10 for n in CAPS),
                "Arm failed initial hold",
            )
            time.sleep(0.05)
        self.event("enabled", positions=positions)

    def freeze(self, positions=None):
        if not self.powered:
            return
        if positions is None:
            positions = self.bus.sync_read("Present_Position", normalize=False)
        for n in CAPS:
            c = self.robot.calibration[n]
            require(
                c.range_min <= positions[n] <= c.range_max,
                "Cannot hold invalid position",
            )
            require(
                abs(positions[n] - self.goals[n]) <= 40,
                "Cannot safely relatch divergent joint",
            )
        for n in CAPS:
            self.write("Goal_Position", n, positions[n])
            self.goals[n] = positions[n]
        self.event("paused_holding", positions=positions)

    def release(self):
        errors = []
        for n in CAPS:
            try:
                self.bus.disable_torque(n, num_retry=2)
            except Exception as e:
                errors.append(str(e))
        if self.touched:
            for n in CAPS:
                try:
                    self.write("Torque_Limit", n, 0)
                    require(self.read("Torque_Limit", n) == 0, "Cleanup zero failed")
                    for k in ("Goal_Velocity", "Acceleration"):
                        self.write(k, n, self.saved[n][k])
                    self.bus.disable_torque(n, num_retry=2)
                    self.write("Torque_Limit", n, self.saved[n]["Torque_Limit"])
                    self.bus.disable_torque(n, num_retry=2)
                except Exception as e:
                    errors.append(str(e))
        torque = self.bus.sync_read("Torque_Enable", normalize=False)
        self.powered = any(torque.values())
        require(
            not any(torque.values()) and not errors,
            f"Motor release could not be verified: {errors}",
        )
        self.released = True
        self.event("released", torque=torque)

    def command(self):
        try:
            c = json.loads((OUT / "operator-command.json").read_text())
            if c["id"] != self.last_command:
                self.last_command = c["id"]
                if 0 <= time.time() - c["at"] < 2:
                    return c["action"]
        except (OSError, ValueError, KeyError):
            pass
        return None

    def run(self):
        self.connect()
        print(
            "READY: continuous phone control, motors off until B1 is pressed.",
            flush=True,
        )
        last = time.monotonic()
        while True:
            cmd = self.command()
            if cmd == "release":
                if not self.released:
                    self.release()
                self.state("MOTORS OFF — SESSION ENDED", "off")
                break
            if cmd == "pause":
                self.pause_requested = True
            try:
                if self.hard_fault:
                    self.state("FAULT — MOTORS RELEASED: " + self.fault, "fault")
                    time.sleep(0.1)
                    continue
                positions = self.check()
                if self.fault:
                    if self.recoverable:
                        self.good_feedback_checks += 1
                        try:
                            released = not phone_input()["b1"]
                        except Exception:
                            released = False
                        if self.good_feedback_checks >= 2 and released:
                            self.event("feedback_recovered")
                            self.fault = None
                            self.recoverable = False
                            self.previous_b1 = False
                            self.pause_requested = True
                        self.state(
                            "FEEDBACK PAUSE — RELEASE B1; PRESS AGAIN WHEN READY",
                            "paused",
                        )
                    else:
                        self.state(
                            "FAULT: "
                            + self.fault
                            + " — support arm, then release motors",
                            "fault",
                        )
                    time.sleep(0.1)
                    continue
                try:
                    s = phone_input()
                except Exception:
                    if self.previous_b1:
                        self.freeze(positions)
                    self.previous_b1 = False
                    self.pause_requested = True
                    self.state(
                        "PHONE LOST — HOLDING POSITION; RELEASE B1 TO RESET"
                        if self.powered
                        else "WAITING FOR PHONE — MOTORS OFF",
                        "paused",
                    )
                    time.sleep(0.05)
                    continue
                if self.pause_requested:
                    if self.previous_b1:
                        self.freeze(positions)
                    self.previous_b1 = False
                    if not s["b1"]:
                        self.pause_requested = False
                    self.state("PAUSED — RELEASE B1, THEN HOLD TO CONTINUE", "paused")
                    time.sleep(0.05)
                    continue
                if not s["b1"]:
                    if self.previous_b1:
                        self.freeze(positions)
                    self.previous_b1 = False
                    self.state(
                        "HOLDING — PRESS B1 TO MOVE"
                        if self.powered
                        else "READY — HOLD B1 TO MOVE",
                        "holding" if self.powered else "ready",
                    )
                    time.sleep(0.05)
                    continue
                if not self.previous_b1:
                    if not self.powered:
                        require(abs(s["a3"]) < 0.15, "Center A3 before first enabling")
                        try:
                            self.enable(positions)
                        except Exception:
                            self.release()
                            raise
                    positions = self.check()
                    self.mapper.latch(positions)
                    self.previous_b1 = True
                    self.reach_limited = False
                    last = time.monotonic()
                # Advance the commanded path; measured sag must not become a
                # new path origin every cycle. Feedback still bounds each step.
                desired = self.mapper.target(self.goals, s["delta_cm"])
                current = phone_input()
                if not current["b1"]:
                    self.freeze(positions)
                    self.previous_b1 = False
                    continue
                if desired is None:
                    if not self.reach_limited:
                        self.freeze(positions)
                    self.reach_limited = True
                    self.state(
                        "REACH LIMIT — MOVE PHONE BACK OR RELEASE B1 TO RECENTER",
                        "active",
                    )
                    time.sleep(0.05)
                    continue
                self.reach_limited = False
                now = time.monotonic()
                dt = min(0.1, now - last)
                last = now
                c = self.robot.calibration["gripper"]
                desired["gripper"] = max(
                    c.range_min + 5,
                    min(
                        c.range_max - 5,
                        round(self.goals["gripper"] + s["a3"] * 120 * dt),
                    ),
                )
                target = step_with_feedback(self.goals, desired, positions, dt)
                # Recheck the enabling input immediately before the command batch.
                if not phone_input()["b1"]:
                    continue
                for n in CAPS:
                    if target[n] != self.goals[n]:
                        self.write("Goal_Position", n, target[n])
                        self.goals[n] = target[n]
                if now - getattr(self, "last_motion_log", 0) >= 0.25:
                    self.event(
                        "motion_sample",
                        phone_delta_cm=s["delta_cm"],
                        a3=s["a3"],
                        positions=positions,
                        goals=dict(self.goals),
                        feedback=getattr(self, "latest_feedback", {}),
                    )
                    self.last_motion_log = now
                self.state("LIVE — B1 MOVE · RELEASE B1 HOLD · A3 GRIPPER", "active")
                time.sleep(0.03)
            except FeedbackUnstable as e:
                self.fault = str(e)
                self.recoverable = True
                self.good_feedback_checks = 0
                self.event("feedback_paused", error=str(e))
                try:
                    self.freeze()
                except Exception as hold_error:
                    self.recoverable = False
                    self.event("hold_unverified", error=str(hold_error))
                self.previous_b1 = False
                self.pause_requested = True
                if self.recoverable:
                    self.state(
                        "FEEDBACK PAUSE — HOLDING; RELEASE B1 TO RESET", "paused"
                    )
                else:
                    self.state("FEEDBACK LOST — HOLD UNVERIFIED; SUPPORT ARM", "fault")
                time.sleep(0.1)
            except HealthFault as e:
                self.recoverable = False
                self.hard_fault = True
                self.fault = str(e)
                self.event("health_fault", error=str(e))
                self.release()
                self.state("FAULT — MOTORS RELEASED: " + str(e), "fault")
                time.sleep(0.1)
            except Exception as e:
                self.recoverable = False
                self.fault = str(e)
                self.event("paused_fault", error=str(e))
                try:
                    self.freeze()
                except Exception as hold_error:
                    self.event("hold_unverified", error=str(hold_error))
                self.state(
                    "PAUSED: " + str(e) + " — SUPPORT ARM BEFORE RELEASE", "fault"
                )
                time.sleep(0.1)


def main():
    global OUT, ROBOT_ID
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-clamped-sweep-clear", action="store_true")
    parser.add_argument("--robot-id", choices=ROBOT_IDS, required=True)
    parser.add_argument("--config", type=Path, default=Path("config.toml"))
    args = parser.parse_args()
    load_config(args.config)
    ROBOT_ID = args.robot_id
    OUT = profile(ROBOT_ID)["directory"]
    OUT.mkdir(parents=True, exist_ok=True)
    require(
        args.base_clamped_sweep_clear,
        "Base mounting and sweep clearance must be confirmed",
    )
    lock = open(OUT / "controller.lock", "w")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    session = Session(args.robot_id)

    def pause(*_):
        session.pause_requested = True

    signal.signal(signal.SIGTERM, pause)
    signal.signal(signal.SIGINT, pause)
    try:
        session.run()
    finally:
        # Normal pause/phone loss never drops a suspended arm. Hardware faults and
        # an explicit supported release use release(); unexpected exit keeps holding.
        if session.bus.is_connected:
            session.bus.disconnect(disable_torque=False)
        session.log.close()
        if session.powered:
            session.state(
                "CONTROLLER STOPPED — MOTORS MAY STILL HOLD; SUPPORT ARM", "fault"
            )
            print("Controller stopped while motors may still be holding.", flush=True)


if __name__ == "__main__":
    main()
