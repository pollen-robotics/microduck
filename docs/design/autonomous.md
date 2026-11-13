# `autod` — the autonomous brain

Status: draft · Date: 2026-11-12 · Owner: antoine

The runtime's `autonomous.rs`, ported as a daemon of its own. It owns four things: what the duck
does when nobody is driving it, when it chases a laser dot or plays with the ball, how it gets out
of the way of a pad that is driving, and how it avoids walking into things with the head ToF. [`ideas/autonomous_behavior.md`](../ideas/autonomous_behavior.md)
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
     │                        │        robot.laserTrack · robot.soccer
     └────────────────────────┘
```

**It is off by default and runs only when switched on.** The switch is `robotd`'s:
`robot.autonomous {active}` sets it, and `robot.state.autonomous` and `robot.mode`'s `autonomous`
read it back (v60). It starts every boot from `[autonomous] enabled` in `robotd.toml`
(`robotctl configure`, "features"), and three things change it until the next boot:

- `robotctl autonomous on|off`;
- an NFC `autonomous_mode` tag, which switches it on, after the laser and soccer off and the robot
  into walk. Every other mode tag switches it off (`nfc/src/modes.rs`);
- nothing else. In particular a pad does not: touching one only takes over for a while (§3).

The switch lives in `robotd` rather than in the file because nothing that reads an NFC tag may
write `robotd.toml`, and because a switch made with a tag is a decision for now, not for every
boot. The unit ships and is enabled like every other daemon, because no unit in this repository is
started or stopped by a setting ([`tof-on-demand.md`](../project/tof-on-demand.md) §"Why not start
and stop the unit"). **Switched off, the duck costs nothing**: `autod` holds one connection to
`robotd` and asks `robot.mode` once a second — no state stream, no depth, no map, no brain —
and `mapd`, unless `[map] enabled`, asks every two seconds and maps nothing. Laser tracking and
soccer stay off unless somebody switches them on. Switched on, `autod` opens a session: the state
stream, the ToF and the map, and a fresh brain (the novelty grid is kept on disk); switched off,
the session closes all three. Measured against a `--fake` `robotd`: no CPU at all off, and the
streams gone within a second of the switch.

Switching it off from on stops the robot where it stands (one zero twist, beak closed, body pose
released, the wheee stopped). The robot is left standing: switching the duck off is not a reason to
sit it down. **`robotd` switches laser tracking and autonomous soccer off with the switch** — not
`autod`, which may be down or restarting at that moment and would otherwise leave the camera
looking for a dot and the legs following it. Soccer by hand, a pad's, is left alone.

**The map comes with it.** `mapd` maps whenever `[map] enabled` is set, and otherwise exactly while
the duck on its own is switched on, saving the map when it goes off
([`mapping-design.md`](mapping-design.md) owns the map). §6 is what the brain does with it.

The brain itself (`autod/src/brain.rs`) is pure: it reads a snapshot of the world each tick and
returns targets and one-shot events. It never touches a socket, which is what lets the ten-minute
soak test run it in a loop without a robot.

## 2. Standing up without a pad

A robot boots limp with its policy off, and until now only the pad's Start brought it up. With the
duck on its own switched on, `autod` does it: `robot.enable {on: true}`, then waits until the
policy drives. A robot booted sitting rises through `sitstand`, the same way Start does it.

**It does this when the switch goes on, and once per `robotd` for the switch already on.**
Switching it on (a tag, `robotctl autonomous on`) is somebody asking for it now, so the robot
stands up. The switch already on when `autod` starts is different: that is the boot, an update
(which restarts `robotd` and `autod` together), or `autod` alone restarting after a crash. Once the
robot is up, `autod` writes which `robotd` it stood up with to `/var/lib/autod/started`, its
`StateDirectory`: the kernel's `boot_id` and the second that `robotd` started (`robot.state`'s
`t_ns − t`). The first time it sees the switch on, it stands the robot up unless that marker names
the `robotd` running now. (It is not under `/run/autod`: that directory goes away with the unit,
so that a stopped daemon leaves no identity behind.)

- **A new `robotd`** (a boot, an update) came up limp by itself. Nobody stopped the robot, so it
  stands up again. The first version keyed the marker on the boot alone, and a duck updated
  without a reboot stayed limp after the update.
- **The same `robotd`**, with only `autod` restarted: the robot is as somebody left it. If it is
  limp, someone may have stopped it, and standing it up behind their back is the bug this rule
  exists to prevent. To bring it back up, press Start, switch the duck off and on, or reboot.

**After a fall, it sees the get-up through.** `robotd` catches a fall (limp, land, pose back) and
hands the robot to the standing policy, which works it upright. When that fails, a robot left to
it flails on the floor until somebody comes. So `autod` watches the get-up the way `robotctl stress`
does (`autod/src/getup.rs`, ported from the `stress-test` branch): up is `fallen` clear for 1 s.
Still down 5 s after the policy took over, it turns the policy off for 2 s, so the robot settles
into its init pose, and gives it back for a second try. Still down after that, it turns the torque
off and asks for help with the questioning quack: the robot needs a hand, and Start once it is on
its feet. A pad touched mid-get-up, or the policy turned off by somebody else, ends the watch. This
is the one time `autod` re-enables a policy: its own cut, given back.

In the same spirit, `autod` **never re-enables a policy somebody else disabled.** It drives only
while the policy is driving (the `policy` label is `walk` or `stand`). In any other state it holds
its clock and sends nothing:

- a skill is running;
- the robot is seated by someone else;
- the robot is limp, fallen or homing.

## 3. The pad takes over, and gives it back

There is no arbitration inside `robotd`: every intent slot is last-writer-wins. So the handoff is
two cooperating clients and one fact `robotd` reports.

- **`padd` is silent until the pad is touched** while the duck on its own is on, which it learns
  from `robot.mode`, polled once a second (so a tag switching it is heard within a second). A
  button press, or a stick or trigger past the deadzone, *engages* it. From then on it drives
  exactly as it always has, heartbeat included. After `[autonomous] pad_idle_s` (default 30 s) with
  no touch, it sends one zero twist and goes silent again. A pad that disconnects while it is not
  engaged does not sit the robot down: it was not the one driving. With autonomy off, `padd` is
  unchanged. `pad_map::engage` is the rule. `netpadd` does not take part: a UDP pad is a program
  somebody is running on purpose, and it drives as it always has.
- **`robotd` says whose twist it is.** Clients name themselves in `hello {client}`, and
  `robot.state.move.source` carries the name of the client whose `robot.move` filled the twist
  slot (v60).
- **`autod` yields while the pad is driving:** `move.source == "padd"` and the twist is not stale
  (`limited_by` has no `deadman`). While it yields it sends nothing at all, so the pad also gets
  the head, mouth and sounds to itself. Its brain is reset on resume, but its memories (energy,
  the novelty grid, how bored it is of the ball) are kept. It takes back over 1 s after the pad's
  twist goes stale. A pad that disables the policy (Start) leaves it disabled, per §2. A ball
  game the brain had started is switched off when the pad takes over: it would otherwise take
  the legs back the moment the stick is centred.
- **A ball game somebody else started is theirs.** Autonomous soccer switched on by anyone but
  `autod` (the pad's D-pad down in soccer mode) makes `autod` stand aside exactly as for the pad,
  until it is switched off.

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
| Laser | chasing a dot: `robotd` drives (below), the duck quacks and tires |
| BallPlay | playing with the ball: autonomous soccer on, `robotd` drives (below) |

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

**The laser and the ball are `robotd`'s to drive, and the brain's to want.** Following a dot and
playing soccer already exist in `robotd` (`laser.rs`, `soccer.rs`), each taking the legs and the
head only while no client asks for them. So the brain's part is the switches (`robot.laserTrack`,
`robot.soccer`, held as levels by `main.rs` and asked again at most every 3 s when refused) and
standing aside while one drives: it then sends a zero twist and a centred head, which `robotd`
reads as nobody asking, rather than going silent and leaving its last command winning until it
goes stale. `robotd` runs one of the two at a time, and switching soccer on switches the laser off.

- **The laser is on whenever the duck could chase**: not tired, not being petted (a dot would walk
  it out from under the hand), not playing ball. A dot being followed interrupts any standing or
  walking state with an excited quack, and **wakes a napping duck**: it stands up at once, no
  stretch, and goes after the dot (or looks for it, if it went). The nap that follows a chase is
  the exception: the duck is tired then, the laser is off, and no dot is seen. The chase ends 2 s after the dot goes, into
  a look around for it. Chasing burns energy at twice a walk's rate (one unit per 50 s). Below 0.2
  the duck has had enough: the laser goes off and it naps (or chills, with no sit/stand policy).
  It comes back on only above 0.45, which a nap gets to.
- **The ball is something the duck goes looking for**, picked from the standing pool like any other
  state when its energy is above 0.35 and no cooldown is running, with a weight that falls with
  energy and with boredom. It switches autonomous soccer on and stands aside.
  - **No ball in 20 s** is no ball around: it gives up, and does not look again for 3 minutes,
    doubling with every miss in a row up to 30. A room with no ball stops costing a spin.
  - **A game** ends on a kick goal of 3 to 8, after a play time of 1 to 3 minutes that shrinks with
    boredom, when the ball has not been seen for 15 s, or below 0.2 energy. Then a ruffle, and no
    new game for 4 to 8 minutes, longer the more bored it is. Playing costs one unit of energy per
    70 s.
  - **Boredom**, 0 to 1, fills over 5 minutes of play (half as fast while only searching) and
    wears off over 20 minutes of not playing.

There is no obstacle avoidance while `robotd` drives the dot or the ball: the ToF gate is the
brain's, and the brain is standing aside. Both are games somebody is watching.

**Left behind on purpose:**

- **Held.** Pickup detection was shelved as unreliable.
- **Startle, and the curiosity glance built on the same per-bin change memory.** Neither worked
  well enough.
- **Preen**, and the seated preening in a nap. It did not read well.

**Body pose** carries Dance, and the body halves of Stretch, Ruffle and Petted. The velstand gait
takes body pose again, so `autod` sends it whenever a state poses, and once with `active: false`
when the state ends.

**Wander walks to goals.** A goal is picked in the brain's frame — the map's when there is one
(§6), odometry's otherwise — and walked to on odometry, which is smooth and good over a few metres.
The runtime's version, holding a random heading for a few seconds at a time, stayed within half a
metre of its start on a robot.

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
So `robot.head` takes an optional `alpha` for that one intent (v60). The brain sends its own
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

**Nothing sees behind, so the duck must not go there.** The brain never asks for a backward twist.
The robot backs up anyway: the walking policy drifts backwards when it turns in place (to be fixed
in the policy), autonomous soccer steps back to line up a kick, and the gait may step back to hold
a body pose that leans. Three things keep that off the walls, all needing the map's grid (§6) —
without it nothing knows what is behind, and backing up is only logged:

- **Turns creep forward by a wall.** With a wall the map knows (or a spot where the ToF stopped the
  duck) within 45 cm behind, and fresh clear floor ahead, a turn in place or a wander pivot walks
  forward at the slowest speed the gait walks at instead of pivoting on the spot.
- **No leaning by a wall.** Stretch and Dance are not picked with a wall within 40 cm behind.
- **Backing up is stopped.** Odometry's forward speed, smoothed over 0.4 s, below −4 cm/s is
  walking backwards: logged once per stretch with what was moving the robot (`walking backwards`,
  with the brain's state, its command, the body pose and the ball). With a wall within 35 cm
  behind, the duck braces: whatever it was doing ends (a ball game with it), the body pose is let
  go, and it steps forward for a second if the ToF shows clear floor ahead, or stands.

## 6. The map

`mapd`'s map does two things for the brain, and the brain does one for the map. `autod/src/map.rs`
holds the pure part; `main.rs` reads `map.stream` and fetches `map.grid`. With no `mapd`, or one
that is not mapping, none of this happens and the duck walks on odometry as it always has.

**The map is an offset, not a pose.** The brain keeps reading odometry at 50 Hz: it is smooth, and
it is what the obstacle model and the gait live in. `map.stream`'s pose (4 Hz, and jumping when a
stop is matched or a loop closes) is paired with the odometry sample of the same `t_ns` to give
where the odometry frame sits in the map. Everything the brain remembers — the novelty grid, where
the way was shut, the current goal — is kept in the **brain's frame**: the map's from the first fix
of the session, odometry's until then. At the first fix, what was kept in odometry's frame is
carried into the map's. A goal is converted back through the latest offset every tick, so a
correction moves the goal, never the duck's footing.

**The duck makes stops worth mapping.** The map grows only where the robot stands still and looks
(`mapping-design.md` §2), and wider views match better. So with a map:

- **A wander bout that reaches its goal ends in a survey**: 4–6 s standing still, the head
  sweeping ±0.9 rad, level. The ToF's 45° wedge covers ~150° of the place.
- **Checked after a sit, one survey.** Standing up from a sit, `mapd` reports `sat` until a stop
  or two confirm the robot is where it sat: a carry while seated looks the same to it
  (`mapping-design.md` §4). `bumped` is the same nearby check. The brain answers both with one
  survey at its next change of state and otherwise carries on — a duck that sat by itself is not
  lost.
- **Lost, it stays put.** When `map.state` says lost for real (carried, fell, booted, or what it
  sees stopped matching), a walking duck stops into a survey, and from then on it alternates surveys with short turns in place — the
  stops the map's search needs, made where it lost itself rather than metres away. It walks again
  as soon as the map finds itself, or after 90 s regardless: a place the map never recognises
  becomes an island the map may join later, and the duck has a life to live. Nothing else
  changes while lost — the laser, the ball, naps and petting carry on. The novelty grid is not
  written while lost, since where the robot is is exactly what is not known.

**Goals know the room.** Every 20 s while the map knows where the robot is, `autod` fetches the
grid at 10 cm. A goal whose straight path passes within 15 cm of a wall the map knows is shut, like
one behind a spot where the ToF stopped the duck; and a goal with unseen floor within 50 cm of it
scores better, one unit per share of unknown cells against the novelty grid's dwell. So the duck
explores where there is something to discover. The ToF's obstacle gate is unchanged: the map
chooses where to go, never whether the way ahead is clear.

**The novelty grid outlives a reboot.** Once it is in the map's frame, it is written to
`/var/lib/autod/visits.json` every minute, and read back at start, faded by the time since at the
running rate (3 % per 30 s): yesterday's haunts are new again, the last hour's are not. It is merged
at the first fix, when the map's frame is known to be the same one.

## 7. What is not here yet

- **Going somewhere on purpose.** Planning a path on the grid around what the map knows — back to a
  favourite nap spot, to where the ball was last seen. Goals are still straight lines.
- **A wiped map leaves the saved novelty grid behind**, in a frame that no longer exists. It fades
  like any other memory; nothing yet notices the wipe.
- **The social and musical ideas.** See the holding pen; they land as inputs and states of this
  brain, not as modes beside it.
- **A real arbitration layer in `robotd`.** That would let the app or a remote peer preempt as
  well (`architecture.md` §6). §3 is two cooperating clients, which is all that two local clients
  need.
