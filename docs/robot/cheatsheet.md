# Cheat sheet

`robotctl`, which runs on the robot. Every command here was taken from `--help` on the branch that
ships it, not from memory.

Read-only commands need no privilege. Anything that **changes** the robot needs `sudo` (or a user in
`--allow-user`/`--allow-group` for `configd`, `allow_uids`/`allow_gids` in `updater.toml` for
`updaterd`).

Branch builds, release candidates and the restart traps after an update are in
[`cheatsheet-dev.md`](cheatsheet-dev.md) — they need a dev board. The same robot over Bluetooth from
a laptop, with no network and no ssh, is [`duckctl.md`](duckctl.md).

## On the robot — `robotctl`

### The first thing to run

```
robotctl version
```

What every daemon is *running* against what is *installed*, plus warnings when they disagree. Run
this before believing any other diagnosis — a daemon serving old code after an update looks exactly
like a bug in the fix you just shipped. See "After an update" below.

```
robotctl health
```

Hardware and software in one report. Exits non-zero when the robot is unhealthy or unreachable, so
it can gate a script — a hot motor or a pinned component is reported, not judged, and does not
affect the exit code. `--json` for a support bundle.

### Watching the loop

```
robotctl monitor
```

What a client asked for beside what was actually applied, with the reason named when they differ —
safety clamps things constantly, and "the stick is forward and the robot is still" is unreadable
without that. A limit is spelled out rather than named: `deadman — no intent arrived recently,
velocity zeroed`.

Also on the frame: every joint measured against what it was commanded, the IMU's projected gravity
and the fall verdict drawn from it, and the achieved loop rate as a trace so a stutter that has
already recovered is still visible. Projected gravity is the only IMU quantity on this stream —
upright is about `[0, 0, -1]`, and it is what `fallen` is decided from. The stale-read counters and
the ratios they mean anything against live in `robotctl health`.

The last row of the header is the robot's condition rather than its behaviour: the pack's charge in
volts and as a fraction, the hottest servo and the board's own temperature. It comes from
`robot.health`, polled every two seconds, because none of it is on the state stream — and it is
where anything wrong gets named, whether that is `unhealthy: control loop at 43.9 Hz`, `degraded:
no robot on the motor bus after 3 attempts` or `orientation frozen — 25 stale reads`. That last one
is on this row and nowhere else on the frame: a board that has stopped fusing keeps answering the
bus, so nothing errors and the gravity vector above holds a plausible attitude indefinitely.

0% is `BATTERY_EMPTY_V`, which is where `robotd` sits the robot down and cuts power, so the figure
is a countdown rather than a gauge — yellow at 30%, red at 15%. A reading that has not been taken
says `batt not read yet` instead of `0.00 V`, which is what the first second of an uptime and a bus
that cannot answer both look like. The row is drawn even when there is no state at all, and that is
the case it matters most in: a board whose servo power is off never completes a control tick, so
nothing arrives on the stream and the reason is only on the health answer.

The bottom border names the policy that is loaded — the `.onnx` files, and whether a standing
network is configured at all — because `walk` is a mode two releases with different gaits both
report. A robot with no policy says so, and one whose policy would not load says that instead,
which the stream's `held` cannot distinguish.

Down the right-hand side, **the robot drawn as it is standing** — the same visual model the
policies were trained against, posed by the measured joint angles and tilted by the IMU's gravity
vector. A leg folded the wrong way, a head pitched into the floor and a duck lying on its side are
all just numbers in the joints table; every one of them is obvious here. It is on by default and
appears whenever the terminal is wide enough (about 110 columns — the tables come first, and the
robot takes what is spare). **`d`** turns it off; **`[`** and **`]`**, or `←`/`→`, orbit it.

Whenever the ToF is delivering frames, what it sees is drawn into the same scene — yellow for a
hit, green for floor — depth-tested against the robot's own body, so a point behind the beak is
hidden by it. That is what makes "is it seeing my hand, or is it seeing itself" answerable. It
needs no key: the points appear when frames arrive and go when they stop.

Under it, when the column is tall enough, **a map of where the robot has been**: odometry's track
from the foot contacts and the IMU, in braille. The panel never grows — the world zooms out as the
track does, so the whole path stays in frame. `+` is where it started, `●` is the robot with a
short ray for its heading, screen-up is the heading it booted with. There is no magnetometer, so
this is relative motion and it drifts; it answers "did it walk in a circle" and not "where is it".

`q` quits; `↑`/`↓` scroll the joint list on a window too short for all of it; `u` switches the
angles between degrees and radians; `t` opens the [ToF matrix](#the-tof-sensor-tofd); `d` toggles
the robot view and `[` / `]` orbit it; `p` opens the pad's raw input stream — every evdev report
from the gamepad, with the gaps between them, which is the only place a stalled radio is visible
([pair a gamepad](pair-a-gamepad.md#when-it-drops-while-you-are-driving)). Angles are degrees on screen — joints, head and the yaw rate.
Redirected or piped it prints one line per tick instead, so `> run.log` and `| grep FALLEN`
behave, and those numbers stay radians whatever the screen is set to. The joint vectors are in
`--json`, which carries the whole state, one object per line:

```
robotctl monitor --json --hz 50 > run.jsonl
```

### Configuring the robot

```
sudo robotctl configure
```

What has been changed on this robot, and nothing else:

```
robotctl configure --list
```

A robot nobody has touched prints nothing — that is the answer. Add `--json` for a support
bundle. It needs no root and no terminal, which is the point: it is the first question to ask
about a robot behaving oddly, and until now answering it meant a full-screen editor over ssh.

An interactive editor over `/etc/robot/robotd.toml`: every key the daemons know, the feature
switches first (policy on/off, walk/roller, limp-fall, audio, pet detection, battery
shutdown, camera and video quality…), current value against default, one line of doc. SPACE toggles, ENTER types a
value, `u` reverts a key to its default. Values in yellow (marked `•`) are the keys where
this robot diverges from the defaults; everything else is the built-in default, and `unset`
optionals show what they resolve to `(auto)`.

Three properties worth trusting:

- **It cannot disagree with the daemon.** The schema, the defaults and the validation come
  from the same crate `robotd` parses the file with, and the key list is pinned complete by a
  test — a new `[section]` in the daemon shows up here or the build fails.
- **It cannot eat your file.** Comments, ordering and keys from other releases survive
  untouched; only the keys you change are written. Reverting a key removes it (and the
  comment attached to it) rather than pinning the default, so the file stays a list of
  *decisions*, not a copy of the defaults.
- **It cannot write a file robotd refuses to start on.** Every save is validated through the
  daemon's own loader first, atomically (temp file + rename), and rejected with the reason.

The daemons read the file once at startup, so saving offers a restart — of the ones that read
what you changed: `[media]` is `mediad`, everything else is `robotd`. `sudo`, because the file
is root-owned — without it the editor opens read-only and says so on the first write.
`--file` points it elsewhere for a bench copy. The shipped `deploy/robotd.toml` stays the
reference for *why* each knob exists; this is for flipping them.

#### Video quality

```
sudo robotctl configure
```

Set `media.quality` — `1080p30`, `720p30`, `720p15` or `360p30` — and take the restart it
offers. `media.camera` off streams a test pattern instead, which is what a board with no camera
wants: the WebRTC *control* channel rides on the video track, so a pipeline that cannot start
costs both. `media.bitrate` follows the quality unless you set it; the unit is bits per second.

`media.congestion_control` is the other knob in that section, and it is the one that moves CPU:
`disabled` drops the bandwidth estimator, which is the largest single consumer in `mediad` (7.6% of
a core against capture's 0.3%), and makes `media.bitrate` the rate rather than a starting point. It
costs adaptivity — on a link that degrades, the picture stalls instead of the rate falling.

720p30 is the rung the pipeline was measured at; a rung that does not hold runs slower rather
than failing. `robotctl monitor` reports the achieved rate on the bottom border, in yellow with
`of <target>` beside it when it is under 90% of what was asked for. What was applied:

```
journalctl -u mediad -b | grep streaming
```

### Policies and skills

A **slot** is what the robot runs by default — the walking gait, the standing network. A
**skill** is what it runs when asked: a kick, the roulade, a bow. Both are files on the Hub, both
change without a daemon release, and nothing below needs a restart.

What this robot is running right now:

```
robotctl policy list
```

Two tables: the seven slots, then the skills — what runs by default, and what runs when asked.

```text
   SKILL        RUNS FOR  POLICY
   kick_left       0.5 s  ball_kick_left.onnx
   roulade           1 s  roulade.onnx
 * polite-bow        4 s  fffiloni/microduck-polite-bow-b1d864/main/policy.onnx
   ground_pick         —  driven by the robot itself
   sit_toggle          —  driven by the robot itself
```

A `*` marks every row config has an opinion about, with a count at the bottom. A slot you switched
off says `switched off` rather than looking like one the robot never had. The directory comes off
the path because the `ORIGIN` column already says which one it was.

`ground_pick` and `sit_toggle` have no length because the robot drives them itself — they answer
to `robot do` like the rest, but they are not entries you can change.

#### A newer official set

The set the robot walks with lives on the Hub and versions on its own line:

```
robotctl policy check
```

```
sudo robotctl policy update
```

`check` prints what is installed, what is newest, and what else the repo offers; it changes
nothing and says so plainly when the Hub cannot be reached. `update` takes the newest unless you
name one — `--version v1` is how to go back. The robot returns to its home pose, re-reads every
slot and drives again, and **a slot you loaded yourself is left alone**, because it points
somewhere else entirely.

#### Trying your own file

No release, no file to edit, no restart:

```
sudo robotctl policy load walk /home/radxa/my_walking.onnx
```

If the robot is walking, it goes to its home pose, loads it, and drives again. If it is doing
something else — sitting, standing still — nothing moves: the network being replaced is not the
one running, and the swap happens underneath. `sudo robotctl policy reset walk` puts that slot
back; with no slot, it puts all seven back.

#### Trying somebody else's

Other people publish policies for this robot:

```
robotctl policy search microduck
```

```
sudo robotctl policy load walk RemiFabre/microduck-flamingo-cycle
```

The repo name is enough — it is fetched and loaded in one step. Add `@v2` for a revision and
`:policy.onnx` for a repo carrying more than one. `policy list` marks it `community`.

**Nothing a stranger publishes is verified by anybody.** What makes it safe to try is the joint
clamps, the fall reflex and the shape gate — not the description. Have the robot on its stand the
first time.

A policy whose manifest says it will not run here — wrong observation width, a newer daemon, a
different robot — is refused before it downloads. One with no manifest is accepted and checked
the usual way, at load.

#### Adding a skill

A policy in the walk slot replaces the gait. A policy added as a *skill* sits alongside it and
runs when asked, which is what most published one-shots want:

```
sudo robotctl policy add polite-bow fffiloni/microduck-polite-bow-b1d864
```

```
robotctl robot do polite-bow
```

The length comes from the repo's manifest. A policy that holds until told otherwise — a
one-footed stand, say — has no length of its own, so give it one, and the twist it reads:

```
sudo robotctl policy add flamingo RemiFabre/microduck-flamingo-cycle --hold 5 --command 1,1,0
```

`--command` is what the network is fed while it runs. Most skills need none: they are trained on
an all-zero command and being selected *is* the trigger. A policy that reads its twist as
something else — flamingo's is `[flag, side, 0]` — needs it spelled out, and its README says
what the slots mean.

`sudo robotctl policy remove <name>` takes one out. A skill this robot's release ships comes back
when you do, since removing the entry only removes the override.

The robot must be driving for a skill to run — press **Start** on the pad first, or the request
is refused saying so.

#### Putting a skill on a button

```
robotctl pad bindings
```
```text
a           ground_pick
x           roulade
lb          kick_left
rb          kick_right
dpad_down   sit_toggle

`robotctl pad bind <button> <skill>` changes one; padd picks it up within a second.
```

```
sudo robotctl pad bind x polite-bow
```
```text
a           ground_pick
x           polite-bow
lb          kick_left
rb          kick_right
dpad_down   sit_toggle
```

That writes one line, and only the button you named:

```toml
[pad]
x = "polite-bow"
```

Put it back:

```
sudo robotctl pad reset
```

**Nothing restarts** — `padd` notices within a second.

A binding naming a skill this robot does not have is marked in the listing rather than
silently doing nothing when you press it.

Five buttons are bindable: `a`, `x`, `lb`, `rb`, `dpad_down`. **`lb`/`rb` are the bumpers**, not
the analog triggers, which are the mouth and the quack. An empty name switches a button off, and
`pad reset <button>` puts one back. The defaults are the mapping the prototype had, so a robot
with no `[pad]` section behaves exactly as it always has.

The rest of the pad is not bindable: Start toggles the policy, Y and B change what the sticks
mean, and held Select powers the robot off — the button that stops a robot is the one worth not
being able to lose to a config edit. A name is checked against what the robot actually has, so a
typo is refused with the list rather than becoming a dead button.

#### Putting it all back

```
sudo robotctl policy reset
```
```
sudo robotctl pad reset
```
```
robotctl configure --list
```

The last one prints what this robot changes from the defaults and nothing else — a robot nobody
has touched prints one line saying so. It is the first thing to run when a robot is behaving
oddly and you are not sure what was left set.

#### The slots, and four things worth knowing

The slots are `walk`, `stand`, `sitstand`, `ground_pick`, `kick_left`, `kick_right` and
`roulade`. `load` writes the choice into `/etc/robot/robotd.toml`, so it survives a reboot and
survives updates — a release replaces the binaries and the policies it ships, not the line that
points elsewhere.

- **Resetting something already reset does nothing, and says so.** No homing, no reload, and no
  `sudo` needed when there is also nothing to write.
- **A policy that is not `obs[1,61] -> actions[1,14]` is refused before anything changes.** The
  file is opened and checked while the robot is still running the old one.
- **A load that fails anyway keeps the policy that was running.** Trying a gait cannot cost you
  the one you had.
- **A file that has gone missing by the next boot costs its slot, not the robot.** The slot falls
  back to this robot's own policy, `robotctl health` reports *degraded* and names the file, and
  `policy reset <slot>` clears it. An official policy that will not load is still **unhealthy** —
  that is a broken release, and the updater rolls it back.

`none` switches a slot off — every slot except `walk`, which is what the others fall back to and
cannot be empty. Some policies need that: one that does its own standing wants the standing
network out of the way, or the robot hands itself to that whenever the command is zero.

#### Publishing a policy for every robot

A policy in the official set reaches every robot, and adding one is four steps with no daemon
release:

1. Upload the `.onnx` to `pollen-robotics/microduck-policies`.
2. Add an entry to its `manifest.json`:
   ```json
   { "file": "polite-bow.onnx", "kind": "episodic", "duration_s": 4.0 }
   ```
   The set's manifest is `schema_version: 2`; a plain one-shot needs no more than those three.
3. Tag it — `hf repos tag create pollen-robotics/microduck-policies v4`.
4. On a robot: `sudo robotctl policy update`.

That entry is what a one-shot needs and nothing more: **`episodic` with a `duration_s`, on the
all-zero command it was trained against, becomes a skill the robot answers to by name** — ready
for `robot do` and a button, with nothing else edited. A **`perpetual`** one is a gait and needs
a slot pointed at it instead.

A policy the daemon has to *drive* — writing a phase over time, or flipping a posture flag —
declares that under `command.encoding`, and its numbers become that arm's timing rather than a
new skill. The ground pick and the sit↔stand are the two, and getting one of those entries wrong
is the one mistake here worth being careful about.

The manifest in full, every field, and what each one changes on the robot:
[`../policy-manifest.md`](../policy-manifest.md).

The shape a policy has to have and what else is checked at load are in
[`../design/robotd-design.md`](../design/robotd-design.md) §2.3; where policies come from, what
`official` means, and how a skill declares itself are in
[`../design/policy-channel-design.md`](../design/policy-channel-design.md).

### Power to the joints (`robotd`)

```
sudo robotctl robot init
```

```
sudo robotctl robot relax --yes
```

`init` powers the joints and ramps to the home pose over about two seconds — **it moves every joint**,
so have the robot on its stand. It needs no policy, and it is what the gamepad's Start does on its way
to driving, so by hand it is a bench thing.

`relax` cuts power and **the robot collapses** if nothing holds it, which is why it wants `--yes`. It
is the only way back to limp short of pulling the plug: pressing Start again stops the policy and
keeps the robot standing, and `robot.stop` zeroes the velocity while still standing.

Both go through `robotd`, which owns the motor bus. `robotd init` — the subcommand — still exists for
a robot whose daemon is not running, and it needs the daemon stopped, because two writers on one UART
corrupt each other's replies:

```
sudo systemctl stop robotd && sudo /opt/robot/daemon/current/bin/robotd init && sudo systemctl start robotd
```

`init` works whether or not the robot has fallen — by default a fall is a *report* (visible in
`robotctl monitor`), not a gate, matching the prototype. A board that sets `[safety] fall_limp`
or `fall_recover` in `robotd.toml` arms the gate: there a fallen robot goes limp and refuses
`init`/`enable`/skills until it is stood up.

### Gamepad (`configd`)

What each button *does* — and how to change it — is under **Policies and skills** above
(`robotctl pad bindings`). This section is about getting a pad connected at all.

```
robotctl pad status
```

```
sudo robotctl pad pair
```

```
sudo robotctl pad pair 78:86:2E:BB:13:28
```

```
sudo robotctl pad forget 78:86:2E:BB:13:28
```

Pairing is once per pad and has a page of its own —
[`pair-a-gamepad.md`](pair-a-gamepad.md): which button puts a pad in pairing mode, adding a second
pad without forgetting the first, and what to do when it will not bond (the `Privacy` setting in
`/etc/bluetooth/main.conf` is the answer more often than anything else).

`padd.service` runs from boot and drives whatever pad connects, so pairing is the only step. The
mapping is the prototype's, so muscle memory carries over:

| control | does |
| --- | --- |
| left stick | drive: forward/back and strafe · head: head yaw and pitch · body pose: up and crouch |
| right stick | drive: turn · head: neck pitch and head roll · body pose: pitch and roll |
| **Start** | toggle the policy — nothing moves until it is on |
| **Y** / triangle | head mode: sticks pose the head (body holds still) |
| **B** / circle | body-pose mode: sticks lean and crouch the standing robot |
| **A** / cross | ground pick |
| **X** / square | roulade — one forward roll; hold to chain rolls |
| **LB / RB** | left / right kick |
| **DPad-Down** | sit ↔ stand |
| **RT / LT** | mouth (either trigger) — RT also quacks; LT rides the "wheee" while held |
| **DPad-Up**, held 3 s | switch drive mode, walk ⇄ roller |
| **Select**, held 2 s | sit down, then power off |

There is no stop button: release the sticks and the robot stands, and `robotd`'s deadman stops it
if `padd` dies. On a roller robot (`mode = "roller"` in `robotd.toml`) the sticks take the roller
shaping automatically — asymmetric push/brake, no strafe — and A triggers the crouch. The
other skills ride along: sit, kicks and the roulade work on wheels too, as the prototype has it.

**Holding DPad-Up switches between the two**, for when you have just put wheels on the duck or
taken them off: the robot quacks once for walking or twice for roller, returns to its home pose,
loads that mode's policies there and drives again — a few seconds, torque on throughout, no
restart. `robotd.toml` is not touched, so a reboot comes back in the configured mode; make it
stick with `robotctl configure` (or `[policy] mode`). It is a hold rather than a press because
D-pad up is easy to lean on while driving.

`pad status` answers two questions separately, because a connected pad and a dead driver look
identical from the outside:

```
pad     Xbox Wireless Controller 78:86:2E:BB:13:28  connected
padd    active — driving whatever pad connects
```

To drive with non-default limits, stop the service first or two processes fight over the sticks:

```
sudo systemctl stop padd
```

```
sudo -u padd /opt/robot/daemon/current/bin/padd --max-linear 0.25
```

When the link itself is the suspect, watch it live — `robotctl monitor`, then `p`. That works with
no robot too: on a board whose servos are unpowered or whose `robotd` is stopped, the monitor opens
on the pad block instead of refusing. For a verdict over a window instead, copy the measurement
over from a clone of this repo:

```
scp scripts/pad-link-test.sh radxa@<board>:/tmp/
```

Drops already in `padd`'s journal — no pad needed, and it answers immediately:

```
sudo sh /tmp/pad-link-test.sh --history
```

Or measure it now, keeping the sticks moving for the whole two minutes:

```
sudo sh /tmp/pad-link-test.sh
```

It counts drops against the kernel's own reason for each, and times the gaps between the pad's
input reports — the failure `padd` cannot see, where the link stays up and the robot walks on a
stale command. [`pair-a-gamepad.md`](pair-a-gamepad.md#when-it-drops-while-you-are-driving) reads
the numbers.

When two boards behave differently with the same pad, the difference is in the stack under it:

```
scp scripts/pad-stack-report.sh radxa@<board>:/tmp/
```

```
sudo sh /tmp/pad-stack-report.sh
```

Kernel, BlueZ, controller firmware, LE or BR/EDR, and the pad's own firmware revision — printed and
saved to `/tmp/pad-stack-<host>-<when>.log`. `--fingerprint` prints only the values that must match
between two boards, for `diff`.
[`pair-a-gamepad.md`](pair-a-gamepad.md#is-this-board-running-the-same-stack-as-that-one) has the
comparison.

### The voice

```
robotctl quack
```

The loudest way to tell ducks apart: every robot's voice bank is generated from its SoC
serial (`sounds ensure-bank`, run by every release install), so the robot that answers — in
a voice that is only its own — is the one you're SSH'd into. A robot with no voice — audio
off, or no bank — says so instead of printing 🦆, so silence always means the wrong duck.
The robot also greets when `robotd` comes up, pecks goodbye before powering off, and — if you
ask it to — coos when the mic hears its head being scratched. That one is off by default in
both modes (`audio.pet_detect = true` turns it on; the classifier ships in the release): the
always-on version cooed at every incidental brush and wore thin. The startup greet has its own switch, for anyone restarting the daemon all day:

```
sudo robotctl configure
```

Set `audio.greet = false` and take the restart it offers on save. That silences the one
quack and leaves the triggers and the mic alone, which `audio.enabled = false` does not.
Audio hardware bring-up — codec driver, overlays, mixer — is `setup-board.sh`'s audio
section, once per board.

To audition a voice or regenerate the bank by hand, the release carries the generator:

```
/opt/robot/daemon/current/bin/sounds show
sudo /opt/robot/daemon/current/bin/sounds ensure-bank --force
```

`sounds theremin` auditions the *live* synth — the voice the theremin plays in, driven by a
scripted hand sweep at the ToF's own frame rate. `--out sweep.wav` writes it instead of
playing it, which is how you hear a voice change without a robot in front of you.

### The duck chorale

```
robotctl chorale
```

Two ducks in a room sing a four-part piece together; more join what they find already going. Runs
until Ctrl-C. `--off` stops one.

**Off by default** — `[chorale] accept` in `robotd.toml`, and it has to be set on every duck that
should take part. A chorale moves the mouth and the head, so a duck that started animating because
another duck walked in would be doing motion nobody asked for. Off also means *invisible*: a duck
that has not opted in puts nothing on the air, rather than politely declining.

How it works, in the order the questions come up:

- **Nobody is in charge.** Both ducks see the same beacons and the lower id conducts, so there is no
  election to lose and no message that has to arrive.
- **There is no shared clock.** The boards have no NTP and no RTC agreement, so the conductor's beat
  counter *is* the timebase: it bumps a byte in a BLE advertisement once per beat, and the arrival of
  a new value is the downbeat. Followers average the phase over about 25 beats, which brings the
  radio's jitter inside the ±20 ms an ensemble needs.
- **Parts are worked out, not assigned.** The lowest duck sings bass. The conductor broadcasts the
  roster and everyone replays the same seating over it — which is what stops two ducks singing the
  same line when they can each see a different subset of the room.
- **Joining changes nobody's part.** A duck arriving takes the part that is free. A duck that leaves
  keeps its *seat* — its line simply goes unsung, exactly as in a choir somebody walked out of —
  because reseating the survivors is the one thing worth avoiding mid-piece.

The readout names the part as soon as it is settled, so what a duck ended up singing survives in the
scrollback:

```
listening for other ducks — Ctrl-C to stop
  singing tenor    with 3 voices
  tenor    bar   12  beat  45.2  3 voices
```

The conductor picks the piece per performance, and a performance *ends* — after the last
note everyone goes back to listening, re-settles, and sings something else after a breath.
`robotctl chorale --piece 2` pins what this robot picks **if it conducts** (a follower sings
what the beacon names, so set it on every duck to guarantee the song); unknown ids are
refused with the robot's catalogue. Ids: 1 wistful, 2 duck-strut, 3 outer-wilds (test asset,
not for release). `DUCK_CHORALE_PIECE=<id>` in `robotd`'s environment is the standing
fallback — note it must be on **robotd**, not on the `robotctl` command line.

To hear the arrangement without any ducks, one machine can render the whole ensemble:

```
sounds chorale --voices 4                 # or --seeds 100,7,42 for particular ducks
sounds chorale --score my-piece.mid       # anything a notation editor exported
sounds chorale --rolloff 0                # for a full-range speaker, not a duck's
```

Scores come from either `sounds/scores/*.duckscore` — a line-oriented text format, documented in
`wistful.duckscore`, which is also the shipped piece — or a MIDI file, which is the path worth using:
**MuseScore is the score editor.** One instrument per voice rather than one piano staff, name the
parts, and export MIDI. Parts are matched by mean pitch, so a score written top-staff-first still
puts the bass on the bass; a track *named* "Soprano" is believed over its pitches.

### Play the duck (the ToF theremin)

```
robotctl theremin
```

The head's depth sensor becomes an instrument: a hand in front of the beak is the pitch —
closer is higher — and the mouth opens with the note, wide at the top of the range. Runs
until Ctrl-C and puts the instrument down on the way out. `--off` puts down one a client left
up.

An explicit mode with nothing clever inside it: while it is up, the nearest return inside the
playable band is the hand. Point the duck at open space and it is silent; point it at a wall
40 cm away and it plays a steady note. It plays sitting, standing or walking — the mouth is
not part of any policy.

The readout's last column is **what the sensor said about that frame**, and it is the answer
to every "why did it stop playing":

```
  0.34 m    438.1 Hz   60% ██████    14 usable · 255:38 4*:9 5*:5 1:12
```

How many zones carry a status the robot believes, then the count per ST status code with a
`*` on the believed ones. A `~` before the note means it is a *held* note bridging a sensor
dropout rather than something measured right now.

That column exists because of the bug it would have found in a minute: ST documents 5 and 9
as "range valid", and a build believing only those **stops seeing a hand at about 30 cm** —
past that a moving hand comes back as 4 or 13 (*consistency failed*, sigma too high) carrying
a distance that is perfectly good for a pitch. If the reach is short, add codes to
`[theremin] statuses` in `robotd.toml`; if it plays phantom notes at empty air, remove some.
`hold_ms` is the anti-chop: it rides over a flickering zone.

Note that `robotctl monitor`'s ToF grid is stricter than the theremin — it marks anything
outside 5/9 as `x`, *could not measure*. A grid full of `x` does not mean the sensor is
broken; it means it is being pessimistic about numbers it does have.

### The ToF sensor (`tofd`)

An 8×8 depth matrix from the head sensor. `robotctl monitor`, then **`t`**:

```
┌ tof VL53L8CX · 15 Hz · 8×8 · 48/64 ranged · 0.12–3.54 m ─────────────┐
│ 0.12 0.15    x 1.44 1.86    · 2.70 3.12                              │
└ · nothing in range · x could not measure · near→far ── seq 412 · 6 ms ┘
```

Distances in metres, coloured near-warm to far-cool. The two marks matter: `·` is
*measured, nothing in range* — free space, which is information — and `x` is
*could not measure*, which says nothing at all about what is out there. A grid
that showed both as blank would hide the difference.

This is the sensor's own frame, not the robot's: there is no reprojection until
the kinematics exist, which is also what makes the block the right place to check
a mounting angle.

`tofd` owns the sensor and nothing else reads the bus. It is an ordinary service —
`sudo systemctl stop tofd` is safe, nothing depends on it, and `monitor` says
"no depth stream" and carries on. Three things it distinguishes, because they need
different fixes:

| the block says | what it means |
| --- | --- |
| `connecting to tofd…` / `no depth stream` | the daemon is not running |
| `no sensor: …` | `tofd` is up; nothing answered on the bus (most ducks) |
| `waiting for the first frame…` | a sensor is ranging; its first scan is ~66 ms away |

To see what is on the bus by hand, or to watch frames without a terminal UI:

```
sudo i2cdetect -y -r 3
journalctl -u tofd -b
```

The sensor shares the codec's I²C bus, so `setup-board.sh`'s audio section already
provisions the bus itself; the ToF step only adds the stable `/dev/i2c-pihat`
name. Both sensor generations are supported — a VL53L5CX and a VL53L8CX are
interchangeable on the board, and the daemon picks the driver from an ID read.

### Wifi (`configd`)

```
robotctl net status
```

```
robotctl net scan
```

```
sudo robotctl net connect <ssid> --psk <passphrase>
```

```
sudo robotctl net connect <ssid> --psk-stdin
```

```
sudo robotctl net forget <ssid>
```

`--psk-stdin` keeps the passphrase out of `ps`, which shows a `--psk` argument to every user on the
box for the lifetime of the command. Prefer it on anything shared.

Joining a network **disconnects the robot from the one it is on**, so an ssh session over wifi will
drop. That is the operation working. A scan takes a few seconds — it waits for the radio to sweep
rather than returning the previous scan's results.

### The Hugging Face account (`updaterd`)

Signing the robot in is what will let you reach it from outside its own network. Nothing else
needs it yet.

```
sudo robotctl account login
```

That prints a code. Open <https://hf.co/oauth/device> on any device, type the code, and the
command finishes:

```
Open https://hf.co/oauth/device and enter this code:

    A6MY-0314

Waiting for approval…
Signed in as PierreRouanet.
```

```
robotctl account status
```

```
sudo robotctl account logout
```

Ctrl-C while it is waiting costs nothing — the robot keeps polling, and `account status` says
whether it was approved. The code is good for five minutes; after that, run `login` again.

A robot that is already signed in refuses, and names the account:

```
sudo robotctl account login --force
```

The same flag abandons a code that is still waiting for approval — the case where somebody started
a login and walked away. The old code stops working; approving it after that does nothing.

The token lasts thirty days and the robot renews it on its own. A robot switched off for longer
than that comes back needing `login` again, which `account status` says in as many words.

### Identity and power (`configd`)

```
robotctl system info
```

```
robotctl system pin
```

```
sudo robotctl system set-name <name>
```

```
sudo robotctl system set-pin <six-digits>
```

```
sudo robotctl system reboot
```

Out of the box a robot calls itself `duck-` plus four characters derived from its own serial, so two
boards flashed from the same image still look different in a phone's Bluetooth list. Renaming takes
effect over Bluetooth within a few seconds — no restart — but a phone has to scan again to see it.

The PIN is what a phone authenticates with over Bluetooth. The factory default is `000000`, which
authenticates anyone who has read this repository.

### Updates (`updaterd`)

```
robotctl update status
```

```
robotctl update check daemon
```

```
sudo robotctl update apply daemon
```

```
sudo robotctl update rollback daemon
```

```
robotctl update log
```

```
robotctl update show
```

```
robotctl update watch
```

`log` lists attempts, one line each, newest first; the first column is the run number. `show`
takes one of those numbers — or nothing, for the most recent — and prints everything that run
did, then the journal for the same window:

```
run 42 · daemon · 2025-08-27 13:06:40 UTC
  applied 0.1.3 → 0.1.4
  asked for latest, from github.com/pollen-robotics/microduck, onto 0.1.3
  requested by uid=1000 gid=1000 pid=2317

  13:06:41      +1s  manifest     0.1.4 · 184.2 MB · sha256 3f9a1c2b… · signed by release.pub · rev 88efc03
  13:06:41           downloading
  13:07:58   +1m17s  note         downloaded 184.2 MB to /opt/robot/daemon/staging/0.1.4/dl/…
  13:08:02      +4s  note         hash matches; signature verifies against release.pub
  13:08:20     +18s  pre-hook
  13:10:12   +1m52s  hook         hooks/preinstall
                                 │ onnxruntime 1.20.1 already present
                                 │ gstreamer: h264 encode ok
  13:10:12           swapping     0.1.3 → 0.1.4
  13:10:14      +1s  unit         robotd: restart
  13:10:23      +8s  health       the robot reported healthy
  13:10:24           ended        applied 0.1.3 → 0.1.4

  ── journal · 2025-08-27 13:06:40 to 2025-08-27 13:11:24 UTC ──
```

Times are UTC, and so is the journal underneath. The `+` column is the gap since the line above,
which is how you find the two minutes.

Reading the journal needs privileges the `robot` group does not carry, so the second half comes
back empty unless you are root. It prints the `journalctl` line when that happens; `sudo` in front
of it is the fix. `--no-journal` prints that line without trying, and `--json` gives the transcript
alone.

The component is `daemon` — one component covering every binary. `apply daemon` installs what the
stable channel offers; branch builds and release candidates need
[`cheatsheet-dev.md`](cheatsheet-dev.md).

### Switching without a download

To something the board already has unpacked. No network involved:

```
sudo robotctl update select daemon 0.1.4
```

```
sudo robotctl update rollback daemon
```

```
sudo robotctl update reset-to-golden daemon
```

`select` activates an installed release, `rollback` goes to the previously installed one, and
`reset-to-golden` goes to the never-pruned known-good one.

And refusing to move at all:

```
sudo robotctl update pin daemon 0.1.4
```

```
sudo robotctl update pin daemon
```

The second form unpins.

### When `updaterd` itself will not start

Everything above goes through `updaterd`, so none of it works when `updaterd` is the daemon that is
down. Check which one it is:

```
systemctl status updaterd robotd btd configd
```

Then go back to golden without it:

```
sudo robot-rescue --dry-run
```

```
sudo robot-rescue --reboot
```

`--dry-run` says what it would do and changes nothing. Without `--reboot` it swaps the release and
prints the reboot command rather than running it: every daemon execs through `current`, so nothing
picks up the swap until it restarts, and a robot that is standing should be caught first.

It declines, and says why, when no golden is configured or when `current` is already golden — if the
daemons are failing on golden itself, a rollback is not the answer and the journal is:

```
journalctl -b -u robotd -u updaterd -u btd -u configd
```

### The robot may have done this already

Three minutes into every boot, a timer asks whether the release brought its daemons up, and falls back
to golden if it did not. So a robot that rebooted on its own and is running an older release than you
installed has probably rescued itself. What it did:

```
robotctl update log
```

The entry reads as a rollback, with the daemon that failed named in its reason. To see the decision
being made rather than its result:

```
journalctl -b -u robot-boot-check
```

```
sudo robot-boot-check --dry-run
```

It acts once. A second rescue is refused while the first is still on record — `updaterd` clears that
when it next starts, so being refused means the daemons did not come up on golden either, and the
answer is the journal rather than another reboot. Past it, if you have read the journal and decided:

```
sudo robot-rescue --force --reboot
```

### Three things that are easy to get wrong

**`rollback` needs a predecessor, but an update creates one.** A freshly provisioned board has
exactly one release, so `rollback` right then has nothing older to go to and says so. Auto-rollback
is *not* affected: applying a release unpacks it alongside the current one and only then moves
`current`, so by the time the health gate runs there are two, and the release you came from is the
target. `rollback_target` picks the highest installed version below `current` that the journal has
not already recorded as bad — so a board with one release is fully protected from the moment it
takes its first update.

The one genuinely unprotected install is the bootstrap itself, which has nothing before it by
definition. `golden` would cover that, and it is deliberately unset until 1.0.0 exists — so
`reset-to-golden` reports honestly that none is configured rather than doing something surprising.

**`version` shows the live release per component, not the release store.** It will never list two
versions, however many are unpacked. Ask the store directly:

```
ls -l /opt/robot/daemon/releases/ /opt/robot/daemon/current
```

**`apply --version` needs the release to still exist upstream; `select` does not.** Releases
carrying known-bad builds get deleted from GitHub, so `apply --version 0.1.3` fails on purpose,
while `select 0.1.3` still works on a board that already unpacked it. The asymmetry is deliberate:
no new board can acquire a broken release, and a board that has one keeps its escape hatch.

### Installing with no network

Sideloading, factory install, or rescuing a board whose `updaterd` is too old to accept the release
that fixes being too old. See [`install-dev.md`](install-dev.md) — it is `updaterd install --from`,
and the `--force` variant has conditions worth reading before you use it.

### Logs

```
journalctl -u configd -b --no-pager | tail -40
```

```
journalctl -u btd -f
```

Swap in `robotd` or `updaterd`. `-f` follows; `-b` is this boot only.

The startup line carries version, git revision and the release directory the process was launched
from, at `warn`, so it survives any log level.

The update history is separate from the journal on purpose — `fsync`ed per entry under
`/var/lib/robot/updater/` — so it survives a robot whose logs were volatile:

```
robotctl update log
```

The last twenty runs keep a full transcript there too, under `runs/`, written as they happened:

```
robotctl update show 42
```

Both outlive the swap, the rollback, and a power cut, which the journal on this board does not —
`/var/log` is zram. If `robotctl` itself is what is broken, the files are newline-delimited JSON
and read fine with `cat`:

```
sudo cat /var/lib/robot/updater/runs/000042.jsonl
```

### Tab completion

`install.sh` sets this up in `/etc/bash_completion.d/`, as a loader that asks the binary for its own
completions — so they follow the installed release instead of going stale when an update adds a
command. For a shell it did not cover, or for a build you are running straight out of `target/`:

```
eval "$(robotctl completions bash)"
```

`zsh`, `fish`, `elvish` and `powershell` work in place of `bash`.

