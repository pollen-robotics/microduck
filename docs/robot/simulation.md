# The simulated duck

A duck in MuJoCo, driven by the real daemons. You develop against it exactly as you develop against
a robot on the desk: the same `robotd`, the same policies, the same 50 Hz loop, the same `robotctl`,
the same console. Only the body is different.

This page is how to use it. [`design/simulation.md`](../design/simulation.md) is what it is and is
not a twin of, and why it is built the way it is.

## What it is

`robotd --sim host:port` runs the daemon with `duck_control::sim::RemoteIo` in place of the servo
bus: every tick, joint positions, velocities and the IMU come in over a TCP socket from a MuJoCo
process, and the policy's targets go back out. Everything above that seam — the control loop, the
policy, safety, fall detection, kinematics, odometry, the whole IPC surface — is the code a robot
runs, unchanged and unable to tell. `tofd --sim` gets its 8×8 depth frames from the same simulator;
`mediad --sim-camera` gets a rendered head-camera image, mounted a quarter turn off like the real one.
`configd` and `updaterd` run too, unchanged, which is what gives a simulated duck a serial, a name
and a Hugging Face account it can be reached through from off this network.

The MuJoCo half lives in [`microduck_rl`](https://github.com/pollen-robotics/microduck_rl) as
`duck-body`: one process, one window, N duck bodies in one scene, with the BAM actuator models the
policies were trained against.

**What it is good for:** anything in the daemons and their clients — IPC, `robotctl`, the console,
the updater, the chorale, policies standing and walking, mapping. **What it cannot tell you:**
anything in a driver. The Dynamixel bus, the BLE radio, the camera ISP, the NPU and the hardware
encoder are absent, not modelled; a bug in one of those is only visible on a robot.

## What you need

- This repo, built for your machine (`cargo build` happens on its own).
- A checkout of `microduck_rl` with its venv — beside this repo, or wherever `DUCK_SIM_RL` points.
  It provides `duck-body`, the scenes, and the `libonnxruntime` a laptop otherwise lacks. The
  camera lives on its `develop` branch.
- For a duck with a camera off Linux: `brew install gstreamer libnice-gstreamer`. The first is one
  merged formula carrying `webrtcsink`, srtp and x264. The second is the ICE agent `webrtcbin`
  needs, and the first does not depend on it — without it the duck comes up and streams to nothing,
  failing only when a browser asks for the video. `scripts/duck-sim` checks for both.
- For the container form only: `sudo`, `systemd-nspawn` (package `systemd-container`) and
  `mmdebstrap`. The script says which one is missing and prints the line to install it.

## `up` or `boot`?

Both drive the same MuJoCo body with the same `robotd`. They differ in what the daemons run *under*.

| | `scripts/duck-sim` (`up`) | `scripts/duck-sim boot N` |
|---|---|---|
| The daemons run as | plain processes, your user, a pidfile each | systemd services inside a `systemd-nspawn` container per duck, from the real unit files |
| Needs | nothing beyond the repo and `microduck_rl` | `sudo`, a one-time Debian rootfs build |
| Starts in | seconds | a minute the first time, seconds after |
| Identity | your laptop's; several ducks are one process tree | a machine-id, a voice and a socket per duck, and `duck-ether` between them |
| Exercises | the control loop, policies, IPC, `robotctl`, the console | all of that, plus `User=`/groups/`RuntimeDirectory=`/hardening, the updater's apply, health gate, rollback and restart order, `journalctl` |

Rule of thumb: **`up`** when you are working on the control loop, a policy, IPC or a client.
**`boot`** when you are working on anything that touches systemd, the updater or provisioning, or on
more than one duck talking to another. Two of the bugs this simulator has caught were a daemon whose
user did not exist, so its unit never started — a class `up` cannot see at all.

## One duck, no container

```sh
scripts/duck-sim                # a MuJoCo window opens, the duck stands up, and it is yours
scripts/duck-sim status         # health, and whether it is standing
scripts/duck-sim drive          # walk forward for 8 s (args: vx vyaw, default 0.3 0)
scripts/duck-sim keys           # drive it from the keyboard: arrows, space, u, r, k/l, q
scripts/duck-race               # time it round scene_challenge (needs DUCK_SIM_SCENE=challenge)
scripts/duck-sim ctl health     # anything robotctl does, aimed at this duck
scripts/duck-sim monitor        # robotctl monitor: joints, IMU, ToF, sticks
scripts/duck-sim log            # robotd's log
scripts/duck-sim simlog         # the MuJoCo side
scripts/duck-sim realtime       # how fast the world is running (see below)
scripts/duck-sim down
```

Without an argument the script builds the daemons from the branch you are on, writes a params file
naming the policies in this repo, starts `duck-body`, `tofd --sim` and `robotd --sim` under
`~/.cache/duck-sim`, and enables the standing policy. `ctl` is `robotctl` pointed at that duck's
sockets, which is the only thing that is different from a robot: on a board the sockets are under
`/run`, here they are under the state directory.

To talk to it from your own tools, the sockets are `~/.cache/duck-sim/duck-a.sock` (robotd; `duck.sock`
is a link to whichever duck `ctl` talks to) and `~/.cache/duck-sim/duck-a-tof.sock`, and the body is
on TCP port 7801.

`DUCK_SIM_SCENE=challenge scripts/duck-sim` swaps the bare floor for an obstacle course — a gate,
a slalom, a corridor, a ramp and steps, a curb, rubble, a roll wall, and a ball with a goal — and
`scripts/duck-race` times a run round it: nine checkpoints, +3 s a fall, −5 s for going over the
wall instead of round it, personal best in `~/.cache/duck-sim/race-best.json`. The scenes live in
`microduck_rl` (`src/mjlab_microduck/robot/microduck/scene_challenge.xml`,
`scene_playground.xml`); the clock only reads the body's own position report, so it cannot affect
the run it times. The clock is wall-clock: if the simulator is running below real time (check with
`scripts/duck-sim realtime`), the slowness counts against the run.

## Several ducks, each a machine you log into

```sh
scripts/duck-sim boot 4         # four ducks in one world, each in its own container (sudo)
scripts/duck-sim shell          # you are on duck-a
scripts/duck-sim shell duck-c   # or any other
duck-a # robotctl health
duck-a # journalctl -u robotd -f
scripts/duck-sim down
```

`boot` runs each duck's daemons under real systemd in a `systemd-nspawn` container, booted from one
Debian 13 rootfs (built once, about three minutes, kept under the state directory) with an overlay
per duck. That buys the part the plain form cannot fake: the real unit files with their `User=`,
groups, `RuntimeDirectory=` and hardening, under a real init — so `robotctl update apply`, the
health gate and the restart order behave as they do on a robot. Ducks are `duck-a`, `duck-b`, ... and
each has its own machine-id and its own voice.

With more than one duck the script also starts `duck-ether`, a fake radio that carries the chorale's
BLE beacons between containers. It is deliberately a *bad* radio — dropped and delayed beacons —
because a perfect one hides the bugs the real one shows.

The first time, `boot` builds the rootfs and may ask you to install `mmdebstrap`. Every duck is a
systemd unit, so `down` is a `systemctl stop`, never a key combination.

## A room to look at, and eyes to look with

```sh
DUCK_SIM_SCENE=apartment DUCK_SIM_CAMERAS=a scripts/duck-sim boot 2
```

The default world is a bare floor. `apartment` is six rooms in 7×6 m with doorways off centre on
purpose, so a pose is recognisable from the duck's 45° forward view; anything with a slash is a path
to your own scene, and `microduck_rl`'s `scene_*.xml` files are the built-in ones.

Cameras are opt-in per duck (`a`, `a,c`, or `all`) because a rendered frame costs 12 ms against
0.3 ms to step four ducks' physics: one camera is a third of a core, four is most of one. Each duck
with a camera gets its own `mediad`, and its console is served at `http://127.0.0.1:8080`, `8081`,
... by index, exactly the page a robot serves.

## Reaching it from anywhere

A simulated duck signs in to a Hugging Face account and appears in that account's robot list, the
same as a robot on a desk. The console, an app or a Space then reaches it over WebRTC without being
on this network.

```bash
DUCK_SIM_CAMERAS=a scripts/duck-sim
```

```bash
scripts/duck-sim ctl account login
```

Open `hf.co/oauth/device`, type the code it prints, and the duck is listed within a few seconds.
`scripts/duck-sim ctl account status` says which account it belongs to, and
`scripts/duck-sim ctl system info` prints the serial and the line `body MuJoCo`.

In the listing it carries a `simulated` flag and a name derived from its serial (`duck-eb55`), so it
does not read as the robot on the shelf. It is a duck like any other otherwise: same control lane,
same H.264, same policies.

Two rules follow from the rendezvous keying a peer by token:

- **One duck per account at a time.** A second `login` on the same account supersedes the first and
  neither looks broken — they take turns being listed. A second duck wants a second account.
- **A Space consuming this duck needs its own token**, not the duck's; a published Space uses the
  visitor's own login.

`scripts/duck-sim ctl account logout` takes it back off the listing.

## Knobs

Environment variables, all optional:

| Variable | Default | What it does |
|---|---|---|
| `DUCK_SIM_RL` | `~/Pollen/microduck_rl` | Where `duck-body`, the scenes and the ONNX runtime are. |
| `DUCK_SIM_STATE` | `~/.cache/duck-sim` | Sockets, logs, params, the rootfs and the ducks' overlays. Short on purpose: a unix socket path is capped at about 108 bytes. |
| `DUCK_SIM_DUCKS` | `1` | How many ducks; `boot N` sets it too. |
| `DUCK_SIM_SCENE` | bare floor | A scene name (`apartment`, `challenge`, `playground`) or a path. |
| `DUCK_SIM_CAMERAS` | none | Which ducks render a camera: `a`, `a,c`, `all`. |
| `DUCK_SIM_DUCK` | `duck-a` | Which duck `ctl` and `monitor` talk to. |
| `DUCK_SIM_KEYFRAME` | `SIT` | Where a duck starts: `SIT` folded on the floor (the standing policy rises from it), `HOME`, `STAND`, `FOLD`. |
| `DUCK_SIM_VIEWER` | `1` | `0` runs MuJoCo headless. |
| `DUCK_SIM_PORT` | `7801` | The first duck's body port; +1 per duck. |
| `DUCK_SIM_FRAME_PORT` | `7901` | The first camera's frame port; +1 per camera. |

## Things worth knowing

**Real time matters.** The daemons' loops are wall-clock. A simulator running below 1.0× real time
is not merely slow to watch: the policies are driving a robot that moves less than they expect, and
they cannot balance it. `scripts/duck-sim realtime` reports the factor, and `boot` prints it. Too
many ducks, or too many cameras, and the ducks do not get slow — they go *unhealthy* at the 45 Hz
gate, and in the container form the updater starts rolling releases back. Fewer ducks, fewer
cameras, or a headless viewer are the fixes, in that order.

**A walking command below about 0.25 does not walk.** The standing threshold is 0.05, so a smaller
command still selects the walking policy — `monitor` says `walk`, and the full command comes back as
applied — but the policy's own output is near flat down there and the duck shuffles on the spot.
Measured over 4 s: 0.15 moves it 6 mm, 0.25 moves it 0.32 m, 0.3 moves it 0.47 m. If a duck looks
like it is refusing to walk, ask for more speed before suspecting the simulator.

**A duck outlives its body.** Closing the MuJoCo window stops the simulator, and the daemons keep
running against a port with nothing behind it. Every failed read counts toward the update gate's
limit (ten consecutive failures by default), so `robotctl health` first keeps answering *healthy*
while the count climbs and then flips to *unhealthy: … consecutive bus read failures* — either way
commands are accepted and nothing moves. The other evidence is `bus write failed … Connection
refused` at 50 Hz in the log. `drive` and `keys` check for the body first and say so; everything
else does not.

**Ducks do not hot-join.** MuJoCo compiles its model, so changing the number of ducks restarts the
simulator. The daemons survive that: `RemoteIo` reconnects on the next tick, and a duck whose body
is briefly gone reports unhealthy rather than dying, the same as a robot with no power on the bus.

**`--sim` is not `--fake`.** `robotd --fake` is a robot made of nothing: no physics, positions echo
back perfectly, nothing falls over. It is for unit tests and for laptop work that needs no body at
all. `--sim` is the twin. The two flags are mutually exclusive.

**A duck should not think it is a laptop.** A duck's voice and its chorale identity are derived from
the hardware serial, which several ducks on one machine would share. The script sets
`DUCK_IDENTITY` per duck so they differ; nothing on a robot sets it.

## By hand, without the script

Each half takes a `host:port` and can be run alone, which is useful when something is wrong:

```sh
# the body, from the RL repo's venv
duck-body --ducks 1 --port 7801 --keyframe SIT
# the daemons, from this repo
target/debug/tofd --sim 127.0.0.1:7801 --socket /tmp/d/duck-a-tof.sock
DUCK_RUNTIME_DIR=/tmp/d ORT_DYLIB_PATH=<libonnxruntime.so> \
    target/debug/robotd --sim 127.0.0.1:7801 --params <params.toml> --socket /tmp/d/duck-a.sock
# who it is, and which account it belongs to
target/debug/configd --socket /tmp/d/duck-a-config.sock --state-dir /tmp/d/duck-a \
    --simulated sim-duck-a --fake-net --fake-pads
target/debug/updaterd --config /tmp/d/updater.toml --socket /tmp/d/duck-a-updater.sock \
    --token /tmp/d/duck-a/hf-token
# a camera, if duck-body was started with --cameras a
printf '[media]\nquality = "360p30"\n' > /tmp/d/mediad.toml
target/debug/mediad --sim-camera 127.0.0.1:7901 --config /tmp/d/mediad.toml \
    --robot-socket /tmp/d/duck-a.sock --tof-socket /tmp/d/duck-a-tof.sock \
    --config-socket /tmp/d/duck-a-config.sock --updater-socket /tmp/d/duck-a-updater.sock \
    --token /tmp/d/duck-a/hf-token
```

Off Linux, `mediad` wants building with its pipeline: `cargo build -p mediad --features gstreamer`.
Without the feature it starts, says so, and exits.

`--token` names the same file on both daemons — `updaterd` writes the credential there and `mediad`
reads it. Point them at different files and the duck signs in and never registers.

The camera's geometry has to match on both sides — `mediad` streams the `[media] quality` rung
(`360p30` is 640×360), and the body must render at the same size, because frames arrive raw with no
handshake and `mediad` refuses one of the wrong size rather than showing a picture nobody can read.
`scripts/duck-sim` is the record of the rest of the arguments; read it before improvising.
