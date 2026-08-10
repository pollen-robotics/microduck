# microduck daemon

The software that runs on the robot, and the machinery that ships it there.

`robotctl` is how you talk to a robot. It runs on the robot itself.

## Is it alive

```bash
robotctl health
```

```
robot     healthy
  loop      50.1 of 50.0 Hz · 2834 ticks · 0 missed · last 13 ms ago
  bus       ok
  imu       ready
  battery   7.62 V (64%)
  motors    41 °C max (left_knee) · 36 °C mean
  cpu       52 °C

software
  updaterd  0.1.4 (rev abc1234)
  robotd    0.1.5 (rev def5678)
  daemon    0.1.5 installed
            last update 0.1.4 → 0.1.5: applied
```

## Drive it

`padd` reads a gamepad and sends intents over the socket. It has no privileged access — it is
an ordinary client, sending exactly what the app and the SDK will send.

The good way to run it is **from your laptop**, with the socket forwarded. Pad in your hands,
robot on the bench, nothing cross-compiled and nothing installed:

```bash
ssh -L /tmp/robotd.sock:/run/robotd.sock radxa@192.168.1.42
```

Leave that open, and in another terminal from this clone:

```bash
cargo run -p padd -- --socket /tmp/robotd.sock
```

On the robot itself it ships with the release, but unlike `robotctl` it is not on `PATH` — so
give the full path. The default socket is already the right one there:

```bash
/opt/robot/daemon/current/bin/padd
```

The controls:

| | |
|---|---|
| **Start** | enable / disable the policy — nothing moves until this is on |
| **Y** / triangle | switch between driving the **body** and posing the **head** |
| **B** / circle | stop |
| left stick | body: forward/back and strafe · head: neck pitch and roll |
| right stick | body: turn · head: head pitch and yaw |

Two things worth knowing before the robot surprises you. Sticks drive the body or the head,
never both, so switching to head mode zeroes the body velocity rather than leaving it walking.
And if the pad disconnects, `padd` sends nothing at all — `robotd`'s deadman stops the robot on
its own, which is the wanted behaviour and the reason `padd` does not invent a zero command.

Speeds are conservative by default. `--max-linear` (m/s), `--max-angular` (rad/s) and
`--max-head` (radians) raise them; `--deadzone` is there because analogue sticks rarely rest at
exactly zero and the robot creeps without it.

## Watch what it is doing

```bash
robotctl monitor
```

The one window into the control loop. It shows what a client asked for beside what was actually
applied, and names the reason when they differ — safety clamps things constantly, and "the stick
is forward and the robot is still" is unreadable without that. A limit is spelled out rather than
named: `deadman — no intent arrived recently, velocity zeroed`.

Also on the frame: every joint measured against what it was commanded, the IMU's projected
gravity and the fall verdict drawn from it, and the achieved loop rate as a trace so a stutter
that has already recovered is still visible. The bottom border names the policy that is loaded,
because `walk` is a mode two releases with different gaits both report — and "which network is
this?" is the first question when comparing them.

`q` quits, `↑`/`↓` scroll the joint list. Redirected or piped it prints one line per tick
instead, so `> run.log` and `| grep FALLEN` behave. The joint vectors are in `--json`:

```bash
robotctl monitor --json --hz 50 > run.jsonl
```

## Run your own policy

You do not need to cut a release to try a network. Point `robotd` at your own `.onnx` on the
board, in `/etc/robot/robotd.toml`:

```toml
[policy]
walk = "/home/radxa/my_walking.onnx"
stand = "/home/radxa/my_stand.onnx"
```

```bash
sudo systemctl restart robotd
```

Your paths survive updates. Delete the lines to go back to the policy the release ships.

A policy that could not be loaded reports **unhealthy** — `robotctl health` and the bottom of
`monitor` both name the reason. The shape a policy has to have, and what else is checked at
load, are in [`docs/robotd-design.md`](docs/robotd-design.md) §5.3.

## Keep it up to date

What each daemon is running, and what is installed:

```bash
robotctl version
```

Install the latest release:

```bash
sudo robotctl update apply daemon
```

And go back if it misbehaves:

```bash
sudo robotctl update rollback daemon
```

`daemon` is the component name — one component covering every binary.

`apply daemon` means "install what the stable channel offers", not "install the newest thing".
That is what you want on a robot. On a **dev board** it is usually a downgrade, because the
branch builds you have been testing are newer than the last stable release — there, name what you
want with `--ref`, below.

## Put your branch on the robot

Make sure the board has been through the [dev install](docs/install-dev.md) first — a board that
has not will refuse branch builds.

Push your branch, then wait for CI to build it:

```bash
gh run list --branch my-branch
```

Once it is green, on the robot:

```bash
sudo robotctl update apply --ref my-branch daemon
```

```bash
robotctl version
```

Go back:

```bash
sudo robotctl update rollback daemon
```

Every push needs the apply again.

## Where next

| | |
|---|---|
| [`docs/install-dev.md`](docs/install-dev.md) | Setting up a board for development, from nothing — and the fix when `--ref` is refused. |
| [`docs/cheatsheet.md`](docs/cheatsheet.md) | Every `robotctl` and `btctl` command on one page: wifi, updates, pinning, rollback, logs. |
| [`docs/robotd-design.md`](docs/robotd-design.md) | How the control side works. §3.1 is the fastest way in — who talks to `robotd` and where the crate boundary sits — with the per-tick dataflow in §5.10. |
| [`docs/architecture.md`](docs/architecture.md) | The services, the IPC contract, safety and authority. |
| [`docs/roadmap.md`](docs/roadmap.md) | What works today versus what is designed. |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | Building, testing, and the conventions this repo holds to. |
