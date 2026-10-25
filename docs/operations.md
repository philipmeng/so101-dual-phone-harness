# Operation and troubleshooting

## Session lifecycle

1. Start a receiver for each phone; check the arm name on each dashboard.
2. Run the motors-off check before starting a controller. No second program may
   open its serial port while the controller is running.
3. Release B1 and center A3. A fresh B1 press enables the arm at its measured pose.
4. Hold B1 to translate the arm; release B1 to hold and recenter.
5. To finish, support that arm, tick the dashboard's support checkbox, and select
   **Release motors and end session**. Verify the motors-off status.

Phone loss requests a hold and requires a B1 reset. Missing/malformed feedback
pauses motion; if a hold cannot be verified, the dashboard says so. Hard health
faults release motors and do not automatically rearm. An unexpected process exit
does not guarantee torque off. SIGINT/SIGTERM pause instead of exiting a powered
controller; do not use a process supervisor that immediately escalates to SIGKILL.

For an already released hard-fault session, finish it through the dashboard,
inspect the event log, then run a new motors-off check before restarting.

## USB ownership

Use one USB board per arm and stable `/dev/serial/by-id/` paths. To inspect without
opening a port:

```bash
ls -l /dev/serial/by-id/
sudo fuser /dev/serial/by-id/YOUR_ARM_ADAPTER
journalctl -k --since '5 minutes ago'
```

A motor power supply does not make an absent USB adapter enumerate. If there is no
new USB connection event, check the data cable and the board-to-Pi USB connection.
If the adapter appears but no motors reply, check motor power and motor-bus wiring.

Some shared robotics services scan all serial ports. Exclude only the two arm
adapters from those services instead of stopping a service another person uses.
A Linux udev rule can grant only the teleoperation user access to each exact board:

```udev
SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_FIRST_BOARD_SERIAL", OWNER:="YOUR_USER", MODE:="0600"
SUBSYSTEM=="tty", ATTRS{serial}=="YOUR_SECOND_BOARD_SERIAL", OWNER:="YOUR_USER", MODE:="0600"
```

Install edited rules in `/etc/udev/rules.d/99-so101-phone-owner.rules`, reload
udev rules, then reconnect only the unpowered/released arm adapter. Verify actual
access as the other service's account; ACLs can matter in addition to mode bits.
Permissions and `TIOCEXCL` cannot evict a process that already owns an open file
descriptor, and privileged processes may bypass them. Resolve existing owners
before starting the harness. Do not reset a shared USB hub carrying other hardware.

## Phones and pairings

Both phones need the exact configured family/name, the same reachable LAN, and
HEBI foregrounded. On iOS, check Local Network permission if discovery fails.
AR/camera tracking must produce changing XYZ, not just button/slider packets.

The first unique matching device is paired in
`.state/<robot_id>/phone-pairing.json`. Subsequent connections must match its MAC.
To intentionally replace a phone: finish the arm session, stop that receiver,
back up/remove only that arm's pairing file, configure the replacement's unique
name, and restart the receiver. Verify its identity before starting the arm.
Never delete a pairing while its receiver is running.

If both devices have the same family/name, discovery is rejected. The configured
phone and dashboard ports must be different for each arm. Two receivers on
different computers are not covered by the local lock files; run them on one host.

## Motion and gripper

The robot's end-effector target uses phone translation in the frame captured at
each B1 press. There is no orientation/tilt control. Hold the phone flat at that
moment and make a small vertical translation to check the up axis.

The commanded path is seeded from the previous command, while measured positions
bound command lead. A slow joint can still pause coordinated body motion; gripper
pacing is independent. An unreachable IK target pauses both body and gripper
commands until it becomes reachable or you release/recenter B1.

If lift stalls but the gripper works, inspect `motion_sample` events for measured
positions, goals, raw load, and current. In the tested setup, P=16 on shoulder and
elbow produced a persistent following error and poor lift. The documented P=32
adjustment is explicit and volatile. It is not a substitute for checking assembly,
power, obstructions, and load. Do not blindly increase thresholds to hide a fault.

## Implemented operating limits

These values describe the code, not independent hardware guarantees:

| Check | Value |
|---|---|
| Phone freshness at command use | under 0.3 s |
| Goal speed | 120 ticks/s, at most 8 ticks per update |
| Command lead | 32 ticks |
| Tracking fault | more than 40 ticks |
| Body calibration margin | 40 ticks |
| Temperature | immediate stop at reported 70°C; warm-sample confirmation from 45°C; software cutoff 60°C |
| Voltage | 9.0–12.6 V for the tested 12 V kit |
| Current cutoff | raw 160 shoulder_lift/elbow_flex; raw 100 other joints |
| Torque-limit caps | pan 350, lift 500, elbow 500, wrist flex 350, wrist roll 250, gripper 150; never above saved limits |

Readings come from full feedback blocks and include motor status. Inconsistent
temperature confirmations pause without issuing new goals. Values at the hardware
threshold are not dismissed as glitches. Recurrent implausible temperatures remain
an open diagnostic issue; check bus integrity and log evidence before continuing.

Each state directory contains `input-status.json`, `gripper-control-status.json`,
`operator-command.json`, pairing/lock files, and timestamped JSONL events. Keep
these private: they can contain phone addresses, calibration-related positions,
and local device paths.
