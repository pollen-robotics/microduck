# The duck on its own, by default — the plan

Holding pen, written 2026-11-20. Planning only: nothing on this page is built unless it says so.
[`design/autonomous.md`](../design/autonomous.md) owns what `autod` does today, and
[`autonomous_behavior.md`](autonomous_behavior.md) stays the pen for the social and musical
ideas. This page is where the duck on its own is going: **the default operating mode of a
microduck**, a pet that lives its own life in the house, with the gamepad as an occasional toy
rather than the way it is used.

The pet brain on the unmerged `pet-life` branch (`duck-life/`, `life-sim/`,
`docs/ideas/duck_life.md`, 2026-11-03) is the starting point for §4 and is folded into this page;
the branch stays the place its code lives until §8 step 2 brings it over.

**Done when** a duck switched on in a house and left there for a week:

- is something people stop to watch, can explain ("it's tired, it's going to its spot"), and tell
  apart from another duck by its character;
- is mostly resting, with bursts of activity, and comes home to its dock when it is hungry;
- never walks off a stair, never needs rescuing more than rarely, and asks for help when it does;
- knows which room it is in after being carried there switched off, and never confuses two rooms;
- sends its owner a few photos a day of things it found;
- is still the default on the eighth day — nobody switched it off because it was annoying.

## 1. Where it stands, and why it feels flat

`autod` works: ToF obstacles at 15 Hz, map-aware wander goals, get-up supervision, the pad's
takeover, the laser and the ball (`autonomous.md`). It is still underwhelming, for four reasons.

- **It rolls dice between animations.** `pick_next_state` (`autod/src/brain.rs`) is hand-set
  transition probabilities nudged by one energy scalar. The duck wanders toward least-visited
  cells, ruffles, looks around, but never *at* a thing or *for* a reason. People believe in an
  animal when they can say why it is doing what it does, and here there is nothing to say.
- **It perceives little worth reacting to.** The brain reads the ToF, petting, voice/noise, the
  laser dot, the ball and the map. Inputs that already exist in the daemon and are not wired to
  it: the duck detector (`mediad`'s `media.detections`, off by default), the beacons (`btd`
  registers one only while a chorale is wanted), the camera's luma (`mediad/src/exposure.rs`), the
  time of day, and the battery (`robot.health`).
- **Nothing accumulates.** Energy and the novelty grid are the only memories. No favourite place,
  no friends, no growth, so nothing changes over weeks.
- **It is not fit to be on all day.** It is busy: Wander is the heaviest weight in the standing
  pool at any energy. A duck that walks all day spends the battery and the gears (the v8 policy
  was already breaking knees, hip pitch and neck pitch). There are no quiet hours, no reaction to
  a low battery, no stair detection, and only `padd` can take over — the app and a WebRTC peer
  cannot.

The fix is the selection mechanism, the perception it selects on, and the things that make it
safe to leave on. More states fix none of them: the fourteen there are become the vocabulary
*inside* activities (§4.6).

## 2. Assumptions

The plan changes if any of these goes away.

- **Thermals are not a constraint** on the beta units and the DVT units after them. The duck
  detector's 2 looks a second (`deploy/robotd.toml` `[duck_detector]`) and `tofd`'s on-demand IMU
  were set on alpha boards; perception budgets get re-measured on a beta (§5.1) and are set by
  CPU share and battery, not by the SoC's temperature. `robot.health` keeps reporting the
  temperature and the governor — reported, never judged.
- **There is no official dock.** Community docks get tried; the duck finds one by a large ArUco
  tag fixed to it (§6). Charging is detected from the battery, not from the dock.
- **Wi-Fi exists** in a home, so NTP time, the app's lane and downloads are available. Nothing in
  the core loop depends on it: a duck without Wi-Fi is still a pet, without the clock and the
  photo delivery.
- **The battery lasts about 3 h** of mixed activity. Seated idle draw is *not measured yet*, and
  it decides how much of a day can be spent off the dock (§3).
- **Gear wear is the lifetime limit** of a duck that is on all day. The walking budget (§3) is
  set from the vetbook's wear counters, not from taste.

## 3. Safe to leave on all day

Everything here comes before the default is flipped, and most of it is independent of the new
brain: it is written against today's `autod` and carried over.

- **Mostly resting.** A believable pet is mostly asleep with bursts of activity (cats sleep
  12–16 h a day), and so is a duck that keeps its gears and battery. Sleep pressure (§4.2) and a
  **walking budget per hour**, both drawn from the vetbook's per-joint wear and stall counters
  (the `vetbook-stall-time` branch), cap how much of the day is spent on its feet. The budget is a
  number to set from wear data, not from taste — an open decision (§9).
- **Battery as hunger.** Low battery slows the duck down and makes it head for the dock (§6); with
  no dock, or none reachable, it sits, calls for help, and at the floor goes limp cleanly rather
  than browning out mid-step.
- **A missing floor is an obstacle.** The ToF's lower rows meet the floor from about 0.7 m
  (`mapping-design.md` §1). A floor that should be there and is not — a stair, a table edge —
  must count as a hit in `obstacle.rs`, explicitly, with its own test against a twin with a step.
  This is the most serious risk of a duck that is autonomous by default.
- **Quiet hours and "go to bed".** A schedule in `robotd.toml` and the app, and a way to say it
  now (a tag, `robotctl`, the app). In quiet hours the duck sleeps and does not speak; woken, it
  is drowsy and settles again. Without this, autonomy stays the default only until the first
  annoyed evening.
- **Anyone driving takes over, not only the pad.** `autod` yields today only to
  `move.source == "padd"`. The rule becomes: a fresh twist from any client but `autod` takes the
  legs, head, mouth and sounds until it goes stale (§3 of `autonomous.md` generalised), so the
  app's stick and a WebRTC peer hand back just as the pad does. A real arbitration layer in
  `robotd` (`architecture.md` §6) stays out of scope until two clients that both want to drive
  exist.
- **Waking and booting.** A duck that boots sits until something wakes it (a sound, petting, the
  morning, the end of its sleep) rather than standing straight up — a robot booted on a table or
  in a box should not get up because the default is on.
- **The default flips.** `[autonomous] enabled = true` in `deploy/robotd.toml`, the first-run
  experience in the app adjusted, and a release cut: `main` being fixed is not a robot being fixed.

## 4. The brain: about something, and somebody

The shape is `duck-life`'s, which is the textbook stack of robot ethology (AIBO's EGO
architecture, Blumberg, Kismet) crossed with game AI (The Sims' needs and smart objects, utility
AI). Every layer is cheap.

```text
 percepts ──▶ beliefs ──▶ drives · emotion ──▶ activity selection ──▶ activity ──▶ motion + expression
 (map, people,  (with       (personality         (utility, commitment,   (goal + target:   (go to, face,
  objects, ducks, confidence shapes the curves)   repetition penalty,     "nap at the dock" pick, photo,
  light, petting) and age)                         reactive interrupts)    "inspect that")   dock, anim, sound)
                    ▲                                                            │
                    └──────── memory: places, rooms, people, ducks, diary ◀──────┘
```

`duck-life` is pure: each tick it takes `Percepts` and returns an `Output` (a motion goal, an
expression, actions). **`autod` becomes the adapter** that fills percepts from `mapd`, `mediad`'s
detections, `pet-detect` and the beacons, and carries out the output with its existing motion
layer (obstacle gate, wander-to-goal, surveys) and its existing state code as expressions.
`life-sim` implements the same seam with a toy house, and the brain cannot tell which it is
talking to. That seam is what lets a week of a duck's life be tuned in seconds.

### 4.1 Beliefs

Places and rooms come from `mapd`. Objects, people and ducks each carry a last-seen time, a
position in the map frame and a confidence that decays. Light comes from the camera's luma, time
from NTP.

### 4.2 Drives

Homeostatic, each 0..1 urgency:

- **Sleep pressure.** Borbély's two-process model: process S builds while awake and drains
  asleep; process C is circadian, set by the clock and the light. A duck that has been up for
  hours, sitting in the dark, wants to sleep.
- **Hunger.** The battery. Going to the dock reads as going to the food bowl.
- **Curiosity** — a cat's. It rises while nothing is new and is drained by investigating
  something that is. What counts as new: an object the detector has not seen here before, a
  change `mapd` notices (a moved chair, a door now open), a sound from a direction, a room not
  visited today, an unseen corner of the map. Investigating has a cat's shape, and it is the
  most watchable thing the duck will do:
  1. **Notice**: freeze, look at it, head tilt.
  2. **Approach in stages**: a few steps, stop and stare, a few more — slower and with longer
     stares the less bold the duck is.
  3. **Inspect**: lean in, peck at it or a ground pick at it, a sound.
  4. **Conclude**: lose interest (habituation, §4.7), retreat if startled, or take a photo of it
     (§7) — and remember it, so the same thing is less interesting tomorrow.
  The novelty grid becomes one source of curiosity among several, not the brain's whole notion
  of the world.
- **Social need**, one for people and one for ducks, rising with time since the last contact at a
  rate personality sets.
- **Play**: the laser, the ball, zoomies.
- **Nesting** (§6.3), later.

### 4.3 Emotion

Short-lived valence and arousal. Events push it, and it decays in minutes. It colours *how*
things are done — speed, amplitude, sound choice, head carriage — not what.

### 4.4 Personality, and making it visible

Sampled at birth like the voice (`sounds/src/personality.rs` already samples one; the brain's
shares the seed so a shy duck has a matching voice), fixed after that, drifting slowly with
experience. The axes come from animal-personality work rather than the human Big Five (the cat
"Feline Five", Litchfield 2017; dog dimensions, Jones & Gosling 2005), adapted to what a duck can
show:

| axis | what it visibly changes |
|---|---|
| activity | how much of the day it is up; walking speed; how long it rests |
| boldness | startle threshold; how close and how fast it approaches the new; stance width |
| curiosity | how strongly novelty pulls; how long it inspects before losing interest |
| human sociability | how fast it misses people; whether it approaches or waits to be approached; how long it stays for petting |
| duck sociability | the same for ducks |
| playfulness | zoomies, games, the ball, gifts |
| vocalness | how chatty it is; quack bursts per hour |
| homeliness | how far it roams from its spots; how much it invests in the nest |

**A personality nobody can see is a random seed.** So it is a requirement, with a test, not a
hope:

- **Exaggerate.** The axes are sampled toward the ends rather than uniformly: most ducks are
  clearly bold *or* shy, clearly chatty *or* quiet, with a few in the middle.
- **Every axis shows within ten minutes.** For each axis there is something an observer sees
  without knowing what to look for: a bold duck walks up to a new object at once and pecks it, a
  shy one stares from a metre away and circles; a lazy duck naps twice in ten minutes, an active
  one never; a chatty one quacks at everything.
- **Signature behaviours.** Each duck gets one or two favourite expressions, drawn at birth and
  weighted up for life — one always ruffles after a walk, another does a little dance on
  arriving. People name pets by their habits.
- **Measured.** In `life-sim`, behaviour statistics per axis must separate ducks of opposite
  personality. On robots, a blind test: people watching two ducks for ten minutes say which is
  the bold one, the lazy one, the chatty one, and agree.

### 4.5 Relationships and memory

Per person and per duck: *familiarity* (exposure) and *affinity* (how good the time together
was). Petting raises affinity; startles lower it. Places: favourite nap spots, where things were
found, where it got startled, which rooms it has been in and when. All of it kept across boots
in `autod`'s `StateDirectory`, in the map's frame, and keyed by room when there are several
(§6.2).

### 4.6 Selection

Every activity scores itself from drives × personality curves. The running one gets a commitment
bonus (no dithering); anything done recently gets a repetition penalty (no loops); a little noise
goes on top (surprise). Reactive interrupts preempt: petting, a startle, the laser, critical
battery, a missing floor. **Every decision keeps its runner-up and the reason**, published on
`robot.state` and in the journal — the console's and the app's "why is it doing that", and the
first thing to read when it does something odd.

The fourteen states of `autonomous.md` §4 stay as expressions: Nap, Stretch, Ruffle, Dance,
Sneeze, GroundPick and the rest are how an activity looks, not what the duck chooses between.

### 4.7 Readability, habituation, growth

- **Readability** (Takayama, Dooley & Ju 2011, animation principles for robots): look at the
  target before going to it; anticipate before a big action and follow through after it. Gaze is
  how the duck says what it is about to do.
- **Habituation.** A stimulus that keeps repeating gets less and less reaction — the same toy,
  the same noise, the same corner.
- **Growth.** Interest in home robots falls after a few weeks (de Graaf et al. 2016; Leite et
  al. 2013); Pleo and aibo answered with life stages. A duckling that sleeps more and knows fewer
  behaviours; behaviours unlocking over weeks; personality drifting with what happened to it.
- **A diary** in the app, "what I did today" — most of the growing happens with nobody
  watching, and the diary and the photos (§7) are how anybody finds out.

## 5. Perception and models

Each is a perception worker next to its sensor, safe to kill, publishing features rather than
pixels, as `pet-detect` and the duck detector do (`architecture.md` §2).

### 5.1 Vision on the NPU

One camera, one NPU, and the frames already on `mediad`'s raw tee (`npu-bringup.md`). Where it
can, a single multi-class model does the work of several, so one inference pays for all of them.

- **The duck detector — retrain it well (todo).** It exists (`duck-detect`, `[duck_detector]`,
  off by default) and is not yet good enough to build behaviour on. The data is the project, not
  the model: duck's-eye-view footage at robot height from the robot's own camera, auto-labelled by
  a large open-vocabulary model and distilled into the small one; synthetic renders from the CAD
  for the tail; hard negatives (rubber ducks, white prints). `media.frame` makes capturing that
  data cheap. Wanted: precise bearing to another duck for gaze, approach and following; the
  beacon gives identity, the ToF the last metre.
- **A human detector.** First question to answer with data, not assumption: **what of a person
  does a duck see?** From 25 cm up it sees feet, ankles and lower legs nearby, a whole body from
  a few metres, and a face only when someone bends down. A capture campaign from the duck's
  camera across a few homes decides the classes — probably *legs/feet* near and *person* far.
  Off-the-shelf COCO person models are trained from eye height and should be measured against
  that footage before anything is trained. Identity (who) is not in this step; see §9.
- **A salient small-object detector** — "a thing on the floor I could pick up". Class-agnostic
  and small: objects of beak size on the floor, from the camera, with the ToF's range to confirm
  it is on the floor and how far. It drives curiosity (§4.2) and grasping (§5.3); it does not
  need to know what the object is. Trained on the same duck's-eye footage, with the
  open-vocabulary teacher asked for "small object on the floor".
- **Light**: the luma `exposure.rs` already computes, published for the brain — the circadian
  input and where the dark corners are.
- **Rates.** With thermals out of the way (§2), how often each model runs is a CPU and battery
  number, re-measured on a beta unit. Faster while the duck is looking for something, slow while
  it rests.

### 5.2 Sound and touch

- **The petting detector — retrain it (todo).** `pet-detect` is a 20 KB CNN that hears the
  head being scratched; it needs more data across ducks, voices, rooms and the duck's own motor
  noise before petting can be the backbone of the bond (§4.5).
- **Startle on a loud sound.** The visual startle was dropped as unreliable (`autonomous.md` §4);
  an audio one, from the ambient sound sentry that already runs, is the cheap replacement.
- **Voice**, as today, with a direction if the hardware allows it.

### 5.3 Grasping

- **A generic grasping policy, trained.** Today's `ground_pick` is an open-loop skill. The plan is
  a learned policy in `microduck_rl`, conditioned on a target's position relative to the duck
  (from the small-object detector and the ToF), that lowers the beak onto the object and closes
  on it — closed loop on the camera, robust to where the object actually is. It runs inside the
  envelope `robotd` enforces (current limits, joint ranges, tilt, duration, falls), and lands
  through the policy channel like any other ONNX.
- **Precise positioning first.** A grasp starts with the duck standing at the right spot facing
  the right way; that is §6.1.

## 6. Places: positioning, the dock, many rooms

### 6.1 Footsteps — precise positioning

The `footsteps` branch walks printed patterns, walks to a pose with FootstepNet, and walks to a
printed tag closed-loop on the camera (re-sight from a staging pose, then the last stretch blind,
settling at about 5 cm / 12°). It keeps going, toward:

- the precision the dock and grasps need, measured, with the error budget written down;
- a goal that comes from the brain (a map pose, a detected object) as well as from a tag;
- landing on `main` behind an API version, so `autod` can ask for "stand here" as an intent.

### 6.2 The map: many rooms, reliably

`mapping-design.md` owns `mapd`; this is what autonomy-by-default needs from it, and it belongs in
that page's "not yet" until it is designed there.

- **A new room is new.** A duck switched off, carried to another room and switched on must
  conclude that it is somewhere it has not mapped, start mapping from scratch, and say so — not
  search the old map forever, and never draw the new room over the old one. Today an
  unrecognised place becomes an island that is kept and not drawn (`mapping-design.md` §4); the
  island has to be able to become a room of its own.
- **Several sessions, one house.** Rooms are maps of their own, each with its own frame, kept
  together. When the duck walks from one into the other (a doorway, an overlap both have seen),
  they are **stitched** — once, and only when the match is unambiguous.
- **Reliability is the requirement.** A wrong merge corrupts both rooms and the brain's memories
  in them, and is far worse than two rooms that stay separate. So: stitch only on strong,
  repeated agreement; keep the evidence; be able to undo a stitch. Camera place fingerprints
  (`mapping-design.md` §8) are the obvious second opinion for "which room is this", proposing and
  vetoing, never measuring.
- **The dock as the origin.** The dock's ArUco tag is a fixed, unambiguous landmark: the room
  with the dock has its map frame anchored on it, and seeing the tag relocalises the duck at once.
- **Benches** that test exactly the kidnap cases: carried while off to a known room, to an
  unknown one, and back; two rooms mapped separately then joined by walking through the door.

### 6.3 The dock

No official dock exists; community docks get tried, so the robot side assumes as little about the
dock as possible.

- **Finding it.** A large ArUco tag on the dock, readable from across a room, gives bearing and
  range from the camera; the tag also anchors the map (§6.2).
- **Going home reliably.** From anywhere in the mapped house: plan a path on the map's grid
  (planning is §7 of `autonomous.md` today — straight lines are not enough to cross rooms), walk it
  with the obstacle gate, and once the tag is in view, hand over to footsteps for the last
  stretch onto the contacts. A duck that cannot get home calls for help (§3).
- **Knowing it is on it.** From the battery: charging current or a rising voltage, not a switch
  on the dock.
- **When it docks.** Hunger (§4.2) sends it; long naps happen on the dock when the battery is
  below half and at its favourite spot otherwise. If its favourite spot is next to the dock, the
  two merge — LOVOT's design, where the dock is the nest.
- **The nest, later.** Over days the duck gathers soft things to a cosy place where it naps
  (`duck_life.md` on `pet-life`, §3.1). It waits on a reliable grasp (§5.3) and the small-object
  detector, and is the long-term showcase, not a first step.

## 7. Photos: the duck shows you what it found

The photo sequence exists (`photo.take`, API v68; pictures in `/var/lib/robot/photos`, listed and
fetched with `photo.list`/`photo.get` over Bluetooth and the lane). The duck on its own uses it:

- **What is worth a photo.** The end of a curious investigation (§4.2), another duck met, a
  person who came to pet it, a new room, the light through a window at sunset. Scored, and
  rate-limited to a few a day so that each one is a small event, not a camera roll.
- **Taken as a duck would.** Look at it, a moment's stillness, the shot — the pose part of the
  existing sequence, without the flashlight countdown unless it is dark.
- **Kept with why.** Each photo carries a caption from the brain ("found this under the sofa",
  "met Pato in the kitchen"), which is also a diary entry.
- **Sent to the phone.** The app shows the duck's photos as their own album and fetches new ones
  when it connects (over the lane, or Bluetooth). Delivery with the app closed needs a push path
  from the robot to the phone, through the remote-access rendezvous (`remote-access-design.md`);
  that is a design question for `mobile-app.md`, not decided here.
- **People in photos** are a privacy promise to make in writing (§9).

## 8. Order

Steps are ordered by what unblocks what; several run in parallel, and the model work (step 4)
starts at once because its lead time is data collection.

1. **Safe all day** (§3) on today's `autod`: resting-dominant selection, the walking budget,
   battery behaviour, the missing floor, quiet hours, any-driver takeover, waking rather than
   standing at boot. Then flip the default and cut a release.
2. **The brain** (§4): bring `duck-life` and `life-sim` over from `pet-life`, wire `autod` as the
   adapter to today's percepts, port the fourteen states as expressions, curiosity and visible
   personality first. Tuned in `life-sim` (behaviour entropy, per-axis separation) before a robot.
3. **Photos** (§7) on curiosity's investigations and on the existing sequence.
4. **Models** (§5): duck-detector data and retrain, the human-detector capture campaign, the
   petting detector retrain, then the small-object detector. Each wired into the brain as it lands.
5. **Footsteps** (§6.1) to `main`, with measured precision.
6. **Mapping** (§6.2): new-room detection and separate rooms first, stitching second, the dock
   tag as anchor with step 7.
7. **The dock** (§6.3): tag, path planning, going home, docking, hunger.
8. **Grasping** (§5.3), then the nest.
9. **Social and musical** (`autonomous_behavior.md`): greeting, mutual gaze, the spontaneous
   chorale, as activities in this brain.

## 9. Open decisions

- **The walking budget.** Minutes of walking per hour the gears can take, from the vetbook's wear
  data on v7/v8/run 3.
- **The beacon while on its own.** Greeting and the spontaneous chorale want it always on; it
  halves the phone advertising rate (`btd/src/chorale.rs`), which was tuned hard.
- **Recognising who.** Voice embedding, the phone's beacon, a face bent down to pet — any of
  them is a promise about what is stored on the robot. Out of scope until the privacy promise is
  written; the brain works with "a person" until then.
- **Photos of people**: on, off, or asked for in the app's first run.
- **Push to the phone** with the app closed: through the rendezvous or not at all.
- **Default on a fresh robot**: on from the first boot, or switched on at the end of the app's
  setup once the owner has seen what it does.

## Literature

- Arkin, Fujita, Takagi & Hasegawa, *An ethological and emotional basis for human-robot
  interaction*, RAS 42 (2003). The AIBO architecture.
- Blumberg, *Old tricks, new dogs: ethology and interactive creatures* (PhD, MIT 1996).
- Breazeal, *Designing sociable robots* (2002). Kismet's drives and emotion.
- Hiolle, Lewis & Cañamero, *Arousal regulation and affective adaptation to human
  responsiveness*, Front. Neurorobotics 8 (2014).
- Beckers, Holland & Deneubourg, *From local actions to global tasks: stigmergy and collective
  robotics*, ALife IV (1994).
- Litchfield et al., *The "Feline Five"*, PLOS ONE (2017); Jones & Gosling, *Temperament and
  personality in dogs*, Appl. Anim. Behav. Sci. (2005).
- de Graaf, Ben Allouch & van Dijk, *Long-term evaluation of a social robot in real homes*,
  Interaction Studies 17(3) (2016); Leite, Martinho & Paiva, *Social robots for long-term
  interaction: a survey*, IJSR 5 (2013).
- Takayama, Dooley & Ju, *Expressing thought: improving robot readability with animation
  principles*, HRI (2011).
- Borbély, *A two process model of sleep regulation*, Human Neurobiology (1982).
- Sha, *Using simplicity to control complexity* (the Simplex architecture), IEEE Software (2001).
- LOVOT (GROOVE X): the dock as the nest.
