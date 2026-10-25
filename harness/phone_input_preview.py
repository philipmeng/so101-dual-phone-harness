"""Local HEBI input preview. No robot/serial imports, connections, or motor commands."""

import argparse
import json
import html
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import hebi
import numpy as np
from scipy.spatial.transform import Rotation
from phone_profiles import (
    PROFILES,
    load_config,
    exclusive_file,
    phone_lock_path,
    profile,
    select_phone,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = profile("follower_2")["directory"] / "input-status.json"
CONTROL = OUT.parent / "gripper-control-status.json"
lock = threading.Lock()
stop = threading.Event()
state = {
    "connected": False,
    "tracking": False,
    "b1": False,
    "a3": 0,
    "delta_cm": [0, 0, 0],
    "presses": 0,
    "releases": 0,
    "packets": 0,
    "max_travel_cm": 0,
    "max_slider": 0,
    "arm_control_enabled": False,
}
last_packet = 0.0
PROFILE = profile("follower_2")

HTML = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Phone control setup</title><style>
body{background:#10151c;color:#e7edf5;font:17px system-ui;max-width:780px;margin:45px auto;padding:20px}
h1{font-size:32px;margin-bottom:8px}.muted{color:#9eafc2}.card{background:#1b2531;border-radius:18px;padding:24px;margin:18px 0}
.badge{background:#24463d;color:#94efbd;padding:8px 12px;border-radius:8px;display:inline-block}
.row{display:flex;gap:18px;flex-wrap:wrap}.metric{flex:1;min-width:150px}.value{font-size:30px;margin:10px 0}
li{margin:13px 0}#dot{font-size:22px}progress{width:100%}small{color:#9eafc2}
</style><h1>Phone control setup</h1><p class="muted">HEBI Mobile I/O · Follower 2 preparation</p>
<div class="badge" id="control">INPUT PREVIEW — ARM CONTROL OFF</div>
<div class="card"><span id="dot">●</span> <strong id="status">Connecting…</strong>
<p id="tracking" class="muted"></p><div class="row"><div class="metric">B1 enable button<div id="b1" class="value">—</div></div>
<div class="metric">A3 gripper slider<div id="a3" class="value">—</div></div></div>
<div class="row"><div class="metric">Forward / X<div id="x" class="value">0 cm</div></div>
<div class="metric">Sideways / Y<div id="y" class="value">0 cm</div></div>
<div class="metric">Up / Z<div id="z" class="value">0 cm</div></div></div><small id="stats"></small></div>
<div class="card"><strong>Test the phone controls</strong><ol>
<li>Hold the phone flat, screen up, with its top pointing forward in the same direction as the arm.</li>
<li>Hold <b>B1</b>, move the phone a few centimetres, then release B1.</li>
<li>Move <b>A3</b> slightly each way and return it to the centre.</li></ol>
<p class="muted" id="instructions">This page shows phone inputs. The status above indicates whether motor control is running.</p></div>
<div class="card"><strong>Arm controls</strong><p>Pause holds the current pose. Support the arm before releasing the motors.</p>
<button onclick="controlArm('pause')">Pause movement</button>
<p><label><input type="checkbox" id="supported"> The arm is supported and ready to release</label></p>
<button onclick="controlArm('release')">Release motors and end session</button><p id="command-result"></p></div>
<script>async function controlArm(action){let supported=document.getElementById('supported').checked;
if(action==='release'&&!supported){document.getElementById('command-result').textContent='Support the arm, then tick the checkbox.';return;}
try{let response=await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,supported})});
document.getElementById('command-result').textContent=response.ok?'Request sent — watch the status above.':'Request failed';}
catch(e){document.getElementById('command-result').textContent='Could not reach controller';}}</script>
<script>async function tick(){try{let s=await(await fetch('/state',{cache:'no-store'})).json();
document.getElementById('control').textContent=s.control_message||'INPUT PREVIEW — ARM CONTROL OFF';
if(s.continuous)document.getElementById('instructions').textContent='Hold B1 to move the arm. Release B1 to hold its pose and reposition your phone. A3 controls the gripper while B1 is held.';
document.getElementById('status').textContent=s.connected?'Phone connected':'Waiting for HEBI Mobile I/O';
document.getElementById('dot').style.color=s.connected?'#94efbd':'#f6ba71';
document.getElementById('tracking').textContent=s.tracking?'3D tracking ready':(s.error||'Waiting for stable camera tracking');
document.getElementById('b1').textContent=s.b1?'Held':'Released';
document.getElementById('a3').textContent=s.a3.toFixed(2);
['x','y','z'].forEach((k,i)=>document.getElementById(k).textContent=s.delta_cm[i].toFixed(1)+' cm');
document.getElementById('stats').textContent=s.presses+' presses · '+s.releases+' releases · '+s.max_travel_cm.toFixed(1)+' cm largest motion';
}catch(e){document.getElementById('status').textContent='Preview stopped'}setTimeout(tick,150)}tick()</script>"""


def snapshot():
    with lock:
        result = dict(state)
        if time.monotonic() - last_packet > 0.5:
            result.update(connected=False, tracking=False, b1=False)
    return result


def configure(robot_id):
    global PROFILE, OUT, CONTROL
    PROFILE = profile(robot_id)
    OUT = PROFILE["directory"] / "input-status.json"
    CONTROL = OUT.parent / "gripper-control-status.json"
    state.update(
        robot_id=robot_id, phone_family=PROFILE["family"], phone_name=PROFILE["name"]
    )


def save_snapshot():
    temporary = OUT.with_suffix(".tmp")
    temporary.write_text(json.dumps(snapshot(), indent=2) + "\n")
    temporary.replace(OUT)


def receiver():
    global last_packet
    lookup = hebi.Lookup()
    group = None
    device_lock = None
    pair_path = OUT.parent / "phone-pairing.json"
    pairing = json.loads(pair_path.read_text()) if pair_path.exists() else None
    if pairing and (pairing["family"], pairing["name"]) != (
        PROFILE["family"],
        PROFILE["name"],
    ):
        raise RuntimeError("Saved phone pairing does not match this arm profile")
    paired_mac = pairing["mac"] if pairing else None
    misses = 0
    reference_position = reference_rotation = None
    held = False
    saved = 0.0
    while not stop.is_set():
        try:
            if group is None:
                mac = select_phone(
                    lookup.entrylist, PROFILE["family"], PROFILE["name"], paired_mac
                )
                if mac is None:
                    save_snapshot()
                    stop.wait(0.5)
                    continue
                if device_lock is None:
                    device_lock = exclusive_file(phone_lock_path(mac))
                group = lookup.get_group_from_macs([mac])
                if group is None:
                    device_lock.close()
                    device_lock = None
                    stop.wait(0.5)
                    continue
                if group.size != 1:
                    raise RuntimeError(
                        "A phone channel must contain exactly one device"
                    )
                if paired_mac is None:
                    paired_mac = mac
                    temporary = pair_path.with_suffix(".tmp")
                    temporary.write_text(
                        json.dumps(
                            dict(
                                family=PROFILE["family"], name=PROFILE["name"], mac=mac
                            )
                        )
                    )
                    temporary.replace(pair_path)
                with lock:
                    state["phone_mac"] = mac
                group.feedback_frequency = 30
            fb = group.get_next_feedback(timeout_ms=250)
            if fb is None:
                misses += 1
                if held:
                    with lock:
                        state["releases"] += 1
                        state.update(b1=False, tracking=False)
                    held = False
                if misses >= 4:
                    group = None
                    if device_lock:
                        device_lock.close()
                        device_lock = None
                save_snapshot()
                continue
            misses = 0
            f = fb[0]
            position = np.asarray(f.ar_position, dtype=float)
            quat = np.asarray(f.ar_orientation, dtype=float)
            valid = (
                position.shape == (3,)
                and quat.shape == (4,)
                and np.isfinite(position).all()
                and np.isfinite(quat).all()
                and np.linalg.norm(quat) > 0.5
            )
            quality = getattr(f, "ar_quality", None)
            if quality is not None:
                try:
                    valid = valid and int(quality) == 0
                except (ValueError, TypeError):
                    valid = False
            b1 = bool(f.io.b.get_int(1)) if f.io.b.has_int(1) else False
            a3 = float(f.io.a.get_float(3)) if f.io.a.has_float(3) else 0.0
            if not math.isfinite(a3):
                a3 = 0.0
            delta = [0.0, 0.0, 0.0]
            if valid:
                rotation = Rotation.from_quat(quat[[1, 2, 3, 0]])
                position = position - rotation.apply([0.0, -0.02, 0.04])
                if b1 and not held:
                    reference_position = position.copy()
                    reference_rotation = rotation.inv()
                if b1 and reference_position is not None:
                    p = reference_rotation.apply(position - reference_position)
                    delta = (100 * np.array([p[1], -p[0], p[2]])).tolist()
            with lock:
                state["presses"] += int(b1 and not held)
                state["releases"] += int(not b1 and held)
                state["packets"] += 1
                state.update(
                    connected=True,
                    tracking=bool(valid),
                    b1=b1,
                    a3=a3,
                    received_at=time.time(),
                    delta_cm=delta,
                    ar_quality=str(quality),
                )
                state["max_travel_cm"] = max(
                    state["max_travel_cm"], float(np.linalg.norm(delta))
                )
                state["max_slider"] = max(state["max_slider"], abs(a3))
                state.pop("error", None)
                last_packet = time.monotonic()
            held = b1
        except Exception as exc:
            with lock:
                state.update(connected=False, tracking=False, b1=False, error=str(exc))
            group = None
            if device_lock:
                device_lock.close()
                device_lock = None
            held = False
            save_snapshot()
            stop.wait(0.5)
        if time.monotonic() - saved >= 0.025:
            save_snapshot()
            saved = time.monotonic()
    if device_lock:
        device_lock.close()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/state":
            value = snapshot()
            try:
                control = json.loads(CONTROL.read_text())
                if time.time() - CONTROL.stat().st_mtime > 1 and control["enabled"]:
                    control["message"] = (
                        "CONTROLLER STATUS STALE — MOTORS MAY STILL HOLD; SUPPORT ARM"
                    )
                value.update(
                    control_message=control["message"],
                    arm_control_enabled=control["enabled"],
                    continuous=control.get("continuous", False),
                )
            except (OSError, ValueError, KeyError):
                pass
            body = json.dumps(value).encode()
            kind = "application/json"
        elif self.path == "/":
            label = PROFILE["robot_id"].replace("_", " ").title()
            body, kind = (
                HTML.replace(
                    "Follower 2 preparation",
                    html.escape(label + " · " + PROFILE["name"]),
                ).encode(),
                "text/html; charset=utf-8",
            )
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        port = self.server.server_port
        if (
            self.path != "/control"
            or self.headers.get("Origin")
            not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}")
            or self.headers.get("Content-Type") != "application/json"
        ):
            self.send_error(403)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size < 1024:
                raise ValueError("Invalid size")
            command = json.loads(self.rfile.read(size))
            action = command["action"]
            if action not in ("pause", "release"):
                raise ValueError("Invalid action")
            if action == "release" and command.get("supported") is not True:
                raise ValueError("Support required")
            path = OUT.parent / "operator-command.json"
            temporary = path.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(dict(id=time.time_ns(), at=time.time(), action=action))
            )
            temporary.replace(path)
        except (ValueError, KeyError, TypeError):
            self.send_error(400)
            return
        self.send_response(204)
        self.end_headers()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", choices=list(PROFILES), required=True)
    parser.add_argument("--config", type=Path, default=Path("config.toml"))
    args = parser.parse_args()
    load_config(args.config)
    configure(args.robot_id)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    receiver_lock = exclusive_file(OUT.parent / "receiver.lock")
    server = ThreadingHTTPServer(("127.0.0.1", PROFILE["port"]), Handler)
    thread = threading.Thread(target=receiver, daemon=True)
    thread.start()
    print(
        f"{args.robot_id}: http://127.0.0.1:{PROFILE['port']} — {PROFILE['family']}/{PROFILE['name']}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join(timeout=1)
