"""Fixed, separate phone/control channels for the two followers."""

import fcntl
import hashlib
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE_ROOT = ROOT / ".state"
MODEL_PATH = ROOT / "models/SO101/so101_new_calib.urdf"
PROFILES = {
    "follower_2": dict(
        port=8765,
        family="HEBI",
        name="mobileIO",
        serial_port=None,
        calibration=ROOT / "calibration/follower_2.json",
    ),
    "follower_1": dict(
        port=8766,
        family="HEBI",
        name="mobileIO-follower1",
        serial_port=None,
        calibration=ROOT / "calibration/follower_1.json",
    ),
}


def profile(robot_id):
    p = dict(PROFILES[robot_id])
    p["robot_id"] = robot_id
    p["directory"] = STATE_ROOT / robot_id
    return p


def model_path():
    return MODEL_PATH


def load_config(path):
    """Validate the whole installation before replacing any active settings."""
    global STATE_ROOT, MODEL_PATH
    path = Path(path).resolve()
    data = tomllib.loads(path.read_text())

    def resolve(value):
        p = Path(value).expanduser()
        return (path.parent / p).resolve() if not p.is_absolute() else p.resolve()

    state_root = resolve(data.get("state_dir", ".state"))
    model = resolve(data.get("urdf", "models/SO101/so101_new_calib.urdf"))
    arms = data["arms"]
    if set(arms) != set(PROFILES):
        raise ValueError("Configure exactly follower_1 and follower_2")
    proposed = {}
    for robot, values in arms.items():
        family, name = values["family"], values["name"]
        port, serial = values["dashboard_port"], values["serial_port"]
        if not all(isinstance(v, str) and v.strip() for v in (family, name, serial)):
            raise ValueError("Family, name and serial_port must be nonempty strings")
        if type(port) is not int or not 1024 <= port <= 65535:
            raise ValueError("Dashboard ports must be integers from 1024 to 65535")
        if not Path(serial).is_absolute():
            raise ValueError("Use an absolute serial device path")
        proposed[robot] = dict(
            family=family,
            name=name,
            port=port,
            serial_port=serial,
            calibration=resolve(values["calibration"]),
        )
    a, b = proposed.values()
    if (a["family"], a["name"]) == (b["family"], b["name"]):
        raise ValueError("Each arm needs its own phone family/name identity")
    if a["port"] == b["port"]:
        raise ValueError("Each arm needs its own dashboard port")
    if Path(a["serial_port"]).resolve() == Path(b["serial_port"]).resolve():
        raise ValueError("Each arm needs its own USB motor-controller board")
    if a["calibration"] == b["calibration"]:
        raise ValueError("Each arm needs its own calibration file")
    PROFILES.clear()
    PROFILES.update(proposed)
    STATE_ROOT, MODEL_PATH = state_root, model


def exclusive_file(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("a")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        handle.close()
        raise
    return handle


def select_phone(entries, family, name, paired_mac=None):
    matches = [
        e
        for e in entries
        if e is not None and not e.is_stale and e.family == family and e.name == name
    ]
    if len(matches) > 1:
        raise RuntimeError(f"Multiple phones named {family}/{name}; use distinct names")
    if not matches:
        return None
    mac = str(matches[0].mac_address).lower()
    if paired_mac and mac != paired_mac.lower():
        raise RuntimeError(
            "A different phone has this name; original phone pairing retained"
        )
    return mac


def phone_lock_path(mac):
    key = hashlib.sha256(mac.lower().encode()).hexdigest()
    return STATE_ROOT / "phone-device-locks" / f"{key}.lock"
