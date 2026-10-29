# Inspect and test motors

Raw maintenance gives a local operator direct position, selective torque and RAM PID access to
all 15 motors, including the mouth. It bypasses policies, pickup/fall reactions and anatomical
clamps for that session. Native servo limits/shutdown are unchanged. Normal robot configuration
is not changed. [The design](../design/diagnostics-design.md) owns lifecycle and failure semantics.

Requires a complete daemon release with API 47; installing robotctl alone does not add robotd
routes. The shipped robotd unit grants the local `microduck` account maintenance access via
`--maintenance-user microduck`; root and the daemon owner are also permitted. Other named users
can be granted explicitly by the service administrator. BLE/WebRTC relay accounts cannot acquire
raw sessions. Existing relax remains available on its normal paths.

## Read first

```sh
robotctl diagnose capabilities
robotctl diagnose inspect
robotctl diagnose status
robotctl diagnose refresh
robotctl diagnose watch > motor-states.jsonl
```

All outputs are JSON/JSONL. inspect/status return the cached complete snapshot, with per-motor
read timestamps and error/unknown values. refresh schedules a bounded motor sweep, limited to
one request/second; poll inspect to see newer timestamps. A sweep spans multiple ticks; the 15
motors are not measured simultaneously. Goal readback confirms a register, not physical arrival.

## One persistent raw session

Choose support and clearance yourself. Entry does not move or relax the robot and requires normal
control disabled, no pending actuator sequence and all 15 torque registers freshly read off.
Entry also requires known drive modes with Torque On by Goal Update disabled; otherwise it
reports `unsupported_drive_mode` and the affected motors. This prevents goal writes from
implicitly energizing motors; entry does not change EEPROM settings.
If entry reports torque_on, cut torque explicitly with the robot supported:

```sh
robotctl robot relax --yes
robotctl diagnose session --mode maintenance-raw --record motor-states.jsonl
```

The record path must be new; it is not overwritten. The session prints responses to stdout and
owns its connection/heartbeat. Type these commands **inside that process**, not as separate
robotctl invocations. Angles below illustrate syntax, not a suitable target for every assembly:

```text
torque head_roll on
gains head_roll p=100 i=0 d=0
position head_roll rad=0.05 valid_for_ms=500
stop
release
```

Any canonical joint name from capabilities uses the same syntax. Torque-on first seeds the goal
from the measured position and verifies it; position commands never enable torque implicitly.
PID values are raw register integers (0..16383), complete P/I/D triplets, RAM only. Position mode
is required; this interface does not switch modes or change EEPROM offsets/limits. Raw position
commands are not smoothed by software; inspect native profile velocity/acceleration too.

A torque-on hold lasts while the session lease is alive. Once a position command is sent, its
valid_for_ms expiry cuts torque only for motors still controlled by that command. A newer command
replaces that deadline for its selected motors. Other torque-on holds remain intact. Keepalive
does not extend a position deadline. Stop cuts the entire session-controlled set without homing;
release also closes ownership. Neither returns to a prior pose or restarts a policy.

EOF/Ctrl-C/disconnection ends the lease. Ctrl-Z suspends the owner process, so its lease expires;
there is no detached heartbeat keeping motors powered. The proposed default lease is 300 ms,
renewed by the CLI every 100 ms; it is not a servo watchdog or a stop-on-process-crash guarantee.

## Cancel and recover

From another local terminal:

```sh
robotctl diagnose cancel
robotctl diagnose status
```

Cancel uses independent maintenance authorization. Existing `robotctl robot relax --yes` attempts
all-joint torque-off and terminates a raw session. If off cannot be read back, status retains
stop_unconfirmed and refuses normal writes/new acquisition. Fix communication/power as needed,
then cancel or relax to retry readback; no daemon restart is needed merely to reclaim a lost
session. Never interpret a sent packet or successful RPC as proof that torque is off.

Status exposes `exit_pending` until fresh normal goals and gains are restored with torque off;
new acquisition remains blocked through that handoff.

After confirmed release, use normal enable explicitly to return to the policy. Normal gains may
be restored at the first normal tick with torque off; torque is not restored automatically.
For a native latched hardware fault: release/cancel and confirm off, use the existing
`robotctl robot reboot-motors ID`, then relax and acquire a new session. No recovery reboot is
performed silently by diagnostics.
