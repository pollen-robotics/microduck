# `autod` — the autonomous brain

Status: draft · Date: 2026-10-01 · Owner: antoine

The runtime's `autonomous.rs`, ported as a daemon of its own. It owns three things: what the duck
does when nobody is driving it, how it gets out of the way of a pad that is, and how it avoids
walking into things with the head ToF. [`ideas/autonomous_behavior.md`](../ideas/autonomous_behavior.md)
stays the holding pen for what comes after the port (the social and musical behaviours); this
page is what exists.

## 1. Shape

`autod` is an unprivileged socket client, like `padd`. It reads `robot.state` from `robotd` and
`tof.frame` from `tofd`, and it sends ordinary intents back. It has no access the pad or the app
does not have, and it is safe to kill: the deadman stops the robot within `[safety] deadman_ms`.

```text
  tofd ── tof.frame (15 Hz) ──┐
                              ▼
  robotd ── robot.state ──▶ autod ──▶ robot.move · robot.head · robot.mouth · robot.sound
     ▲          (50 Hz)       │        robot.do (ground_pick, sit_toggle) · robot.enable
     └────────────────────────┘
```

**It is off by default and runs only when switched on.** `[autonomous] enabled = true` in
`robotd.toml` (`robotctl configure`, "features"). The unit ships and is enabled like every other
daemon, because no unit in this repository is started or stopped by a setting
([`tof-on-demand.md`](../project/tof-on-demand.md) §"Why not start and stop the unit"). Switched
off, `autod` parks: it connects nothing and sends nothing. `configure` restarts it when the key
changes.

The brain itself (`autod/src/brain.rs`) is pure: it reads a snapshot of the world each tick and
returns targets and one-shot events. It never touches a socket, which is what lets the ten-minute
soak test run it in a loop without a robot.

## 2. Boot without a pad

A robot boots limp with its policy off, and until now only the pad's Start brought it up. With
`autod` enabled, `autod` does it: `robot.init`, wait until homed, then `robot.enable {on: true}`.
A robot booted sitting rises through `sitstand`, the same way Start does it.

**It does this once per boot.** Once the robot is up, `autod` writes the kernel's `boot_id` to
`/var/lib/autod/started`, its `StateDirectory`, and checks it against the running boot on every
start. (It is not under `/run/autod`: that directory goes away with the unit, so that a stopped
daemon leaves no identity behind.) When `autod` restarts later (it crashed, `robotd` restarted under an update, `configure` restarted it), it
does not bring the robot back up on its own. Someone stopped it, and standing it up behind their
back is the bug this rule exists to prevent. To bring it back up, press Start, or reboot.

In the same spirit, `autod` **never re-enables a policy somebody else disabled.** It drives only
while the policy is driving (the `policy` label is `walk` or `stand`). In any other state it holds
its clock and sends nothing:

- a skill is running;
- the robot is seated by someone else;
- the robot is limp, fallen or homing.

## 3. The pad takes over, and gives it back

There is no arbitration inside `robotd`: every intent slot is last-writer-wins. So the handoff is
two cooperating clients and one fact `robotd` reports.

- **`padd` is silent until the pad is touched** while `[autonomous] enabled`. A button press, or a
  stick or trigger past the deadzone, *engages* it. From then on it drives exactly as it always
  has, heartbeat included. After `[autonomous] pad_idle_s` (default 30 s) with no touch, it sends
  one zero twist and goes silent again. With autonomy off, `padd` is unchanged.
- **`robotd` says whose twist it is.** Clients name themselves in `hello {client}`, and
  `robot.state.move.source` carries the name of the client whose `robot.move` filled the twist
  slot (v59).
- **`autod` yields while the pad is driving:** `move.source == "padd"` and the twist is not stale
  (`limited_by` has no `deadman`). While it yields it sends nothing at all, so the pad also gets
  the head, mouth and sounds to itself. Its brain is reset on resume, but its memories (energy,
  the novelty grid) are kept. It takes back over 1 s after the pad's twist goes stale. A pad that
  disables the policy (Start) leaves it disabled, per §2.

A pad that is merely connected therefore costs `autod` nothing, and touching it is all it takes
to drive.

## 4. The brain

A state machine on an energy model, ported from the runtime as it was at its last commit. These
states come over:

| state | what it does |
|---|---|
| Chill | stands, small slow head drifts, breathing mouth |
| LookAround | big glances, head cocks |
| Wander | walks to a goal point 1–3 m away in the odometry frame (below), brief side glances that snap back so the ToF keeps seeing the path |
| TurnInPlace | spins toward the least-visited direction, head leading |
| Zoomies | full speed, jerky turns, the wheee ride; only with fresh, clear ToF ahead |
| Stretch | crane up, yawn, coo, head shake |
| Ruffle | fast feather shiver |
| Dance | body-pose groove, the head grooving along |
| Sneeze | inhale, hitch, CHOO, dazed look |
| GroundPick | `robot.do ground_pick`, then a happy quack |
| Nap | `sit_toggle`: either a doze (head droops, snaps up, dream twitches) or a seated relax |
| Petted | melt, excited wiggle, or sleepy melt that can fall asleep into a nap |

Across all states:

- **Energy, from 0 to 1.** Walking and zoomies burn it; chilling, naps and petting refill it. It
  weights the next state, the walking speed and how chatty the duck is.
- **Novelty grid.** Dwell time is kept per 0.5 m cell of the odometry frame and decays over about
  15 minutes. Wander goals and turn headings lean toward space the duck has not been in.
- **Quack bursts.** One to four `chirp`s, with the mouth and a head toss.
- **Head bob while walking.**
- **Sounds.** A voice gets an attentive perk and a quacked answer after a beat. A sharp noise
  stirs a dozing duck without waking it. The duck's own sounds never count: anything heard within
  1.5 s of the brain asking for a sound, or while the wheee plays, is dropped.

**Left behind on purpose:**

- **BallPlay.** There is no ball detector in the daemon.
- **Held.** Pickup detection was shelved as unreliable.
- **Startle, and the curiosity glance built on the same per-bin change memory.** Neither worked
  well enough.
- **Preen**, and the seated preening in a nap. It did not read well.

**Body pose** carries Dance, and the body halves of Stretch, Ruffle and Petted. The velstand gait
takes body pose again, so `autod` sends it whenever a state poses, and once with `active: false`
when the state ends.

**Wander walks to goals, on odometry believed perfect.** There is no SLAM yet, and a goal a few
metres away is reached before drift matters. The runtime's version, holding a random heading for a
few seconds at a time, stayed within half a metre of its start on a robot.

- **Picking a goal.** Each bout tries 16 directions at 1, 2 and 3 m. It takes the point whose
  straight path crosses the least-visited ground of the novelty grid, preferring far over near,
  with a small cost for turning.
- **Avoiding dead ends.** It avoids paths within 0.4 m of where an obstacle stopped the duck in the
  last five minutes, so the next goal is not behind the same wall.
- **Walking it.** The duck pivots first when the goal is more than 0.7 rad off its heading, steers
  as it walks, and slows inside 0.4 m.
- **Ending a bout.** A bout ends on arrival (0.25 m), or after 8 s without getting 10 cm closer.
  A timer computed from distance and speed was wrong, because the gait does not walk as fast as it
  is asked to.
- **The bout log.** Every bout ends with one `wander bout over` line: distance covered against the
  goal's distance, metres walked and metres asked for, and seconds spent clear, aging, unseen and
  slowed by the obstacle gate. When a duck does not get far, that line says why.

**A forward command the gait will walk at.** The walking policy stands still at small commands,
with its label still saying `walk`. On the twin, a walking duck stops below about 0.12 m/s, and a
standing one does not start below 0.25. Everything that scales speed down (aging ToF data, closing
on an obstacle, arriving at a goal) used to land there. So `autod` never asks for less than
`MIN_WALK_VX` (0.15 m/s), and every start from a standstill gets 0.6 s at full speed.

**Speeds are the pad's.** Linear 0.3 m/s and angular 1.5 rad/s: `robotd_params::drive`, which
`padd`'s stick scaling defaults to as well. There is no separate autonomous speed knob.

**Smoothing.** `robotd` slews the head at `[control] head_alpha` for everyone. The brain's calm
glances are fine at that rate, but a 4.5 Hz feather shiver comes out at a third of its amplitude.
So `robot.head` takes an optional `alpha` for that one intent (v59). The brain sends its own
per-state rate, and every other client keeps the shared feel.

## 5. Obstacles

The runtime's avoidance was built when the ToF arrived at about 1 Hz over a bit-banged bus, and it
was unreliable. `autod/src/obstacle.rs` rebuilds it for the real 15 Hz:

1. **Pair each frame with the state nearest its `t_ns`**, not the latest one. The state carries
   head joints, gravity and odometry. `autod` keeps the last second of states.
2. **Statuses.** 255 means *observed clear*. A trusted status is a range. Anything else is
   *unknown*, never clear. The runtime counted only 5 and 9 as observed at all, so a dark target
   past about 30 cm (status 4 or 13) left bins uncovered, the summary went stale and the duck
   stalled. The trusted set is the theremin's (`[4, 5, 6, 9, 10, 12, 13]`), and step 4 is what
   keeps the extra noise out.
3. **Classify with `kinematics::tof::Reprojector`.** It is levelled by the IMU's gravity, at the
   odometry's trunk height. The runtime's floor rule used a fixed 0.12 m trunk height and no IMU,
   so a pitched trunk read the floor as a wall. Hits higher than `MAX_HEIGHT_M` above the floor
   (0.35 m, above the duck) or farther than 1.5 m are dropped: the duck walks under tables.
4. **Confirm before believing.** A hit counts only if a second hit, in the same frame or the one
   before, lies within 8 cm of it. A single noisy zone stops nothing.
5. **Remember in the world.** Confirmed hits are stored in the odometry frame and re-expressed in
   the *current* body frame every tick, so a remembered chair leg keeps getting closer as the duck
   walks at it. A hit expires after `STALE_S` (1.5 s). It is not dropped just because a later
   frame failed to see it again, which was the runtime's "clear overwrites remembered" bug.
6. **Ask about a corridor, not a cone.** The question is whether the body will hit something:
   the nearest hit with `x > 0` and `|y|` under half the body width plus a margin, measured from
   the beak (`BEAK_M`), not from the trunk origin. Clear space to the left and to the right picks
   which way to steer.
7. **Freshness.** Fresh means the forward corridor was observed within `FRESH_S`, either by a
   trusted range or by a 255 (the head is not glancing away). The gate is the runtime's:

   | data ahead | result |
   |---|---|
   | fresh | full speed |
   | aging, under 1.5 s | half speed |
   | stale | stop advancing, keep turning |
   | hit under `REACT_M` (0.45 m) | slow down and steer away |
   | hit under `STOP_M` (0.15 m) | spin away |
   | three hard stops within the heat window | a boxed-in break-out spin |

## 6. What is not here yet

- **Persisting memories across boots.** The novelty grid lives in an odometry frame that resets
  at boot, so it would mean nothing tomorrow.
- **The social and musical ideas.** See the holding pen; they land as inputs and states of this
  brain, not as modes beside it.
- **A real arbitration layer in `robotd`.** That would let the app or a remote peer preempt as
  well (`architecture.md` §6). §3 is two cooperating clients, which is all that two local clients
  need.
