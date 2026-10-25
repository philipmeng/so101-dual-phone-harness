"""Check one powered-off follower; optionally set a verified lift response gain."""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "harness"))
import phone_control as control
from phone_profiles import exclusive_file, load_config, profile


def set_lift_gain(session, gain):
    """Only change P, with torque off and EEPROM writes locked; roll back on error."""
    if gain not in (16, 32):
        raise ValueError("Supported response gains are 16 and 32")
    names = ("shoulder_lift", "elbow_flex")
    if any(session.read("Torque_Enable", n) != 0 for n in control.CAPS):
        raise RuntimeError("All motors must be off before adjusting response")
    before = {n: session.read("P_Coefficient", n) for n in names}
    changed = []
    try:
        for n in names:
            session.write("Lock", n, 1)
            if session.read("Lock", n) != 1:
                raise RuntimeError("Could not lock EEPROM writes")
            changed.append(n)
            session.write("P_Coefficient", n, gain)
            if session.read("P_Coefficient", n) != gain:
                raise RuntimeError("Gain verification failed")
    except Exception:
        for n in changed:
            session.write("Lock", n, 1)
            session.write("P_Coefficient", n, before[n])
            if session.read("P_Coefficient", n) != before[n]:
                raise RuntimeError("Gain rollback verification failed")
        raise
    return {"before": before, "after": {n: gain for n in names}, "volatile": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.toml"))
    parser.add_argument("--robot-id", choices=control.ROBOT_IDS, required=True)
    parser.add_argument(
        "--set-lift-gain",
        type=int,
        choices=(16, 32),
        help="Optional explicit motor-setting change; does not move the arm",
    )
    args = parser.parse_args()
    load_config(args.config)
    control.ROBOT_ID = args.robot_id
    control.OUT = profile(args.robot_id)["directory"]
    control.OUT.mkdir(parents=True, exist_ok=True)
    with exclusive_file(control.OUT / "controller.lock"):
        session = control.Session(args.robot_id)
        try:
            session.connect()
            samples = []
            for _ in range(20):
                positions = session.check()
                samples.append(dict(session.latest_feedback))
                time.sleep(0.1)
            session.mapper.latch(positions)
            neutral = session.mapper.target(positions, [0, 0, 0]) is not None
            result = dict(
                robot_id=args.robot_id,
                positions=positions,
                neutral_reachable=neutral,
                temperatures={
                    n: [
                        min(r[n]["temperature"] for r in samples),
                        max(r[n]["temperature"] for r in samples),
                    ]
                    for n in control.CAPS
                },
                lift_gains={
                    n: session.read("P_Coefficient", n)
                    for n in ("shoulder_lift", "elbow_flex")
                },
            )
            if args.set_lift_gain is not None:
                result["gain_adjustment"] = set_lift_gain(session, args.set_lift_gain)
            control.atomic(control.OUT / "preflight.json", result)
            print(json.dumps(result, indent=2))
            if not neutral:
                raise RuntimeError(
                    "Reposition the supported, unpowered arm away from its travel limits"
                )
        finally:
            if session.bus.is_connected:
                session.bus.disconnect(disable_torque=False)
            session.log.close()


if __name__ == "__main__":
    main()
