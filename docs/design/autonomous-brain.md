# An autonomous brain for the Microduck — behaviour from prediction

Status: proposal · Date: 2026-10-07 · Owner: xaqmusic (community)

Video introduction and demo: https://youtu.be/W2jMf6EhW0k

[`autonomous_behavior.md`](../ideas/autonomous_behavior.md) calls the brain the biggest untracked gap in the parity audit:
the runtime's sixteen-state machine has to be ported, and no design document owns it. Its shape notes already say what the
answer should look like: *presence, mood and the beat are inputs to one brain, not modes beside it.* This document proposes a
design pattern for that one brain, and describes an implementation of it that runs in simulation on this repository's MJCF,
as a client of the daemon, on top of the shipped policies.

**The proposal.** The brain is *active inference inspired*: it borrows the framework's structure as a design pattern,
without claiming to implement its formal machinery. The duck carries predictions about what it will sense, and it has two ways
to make a prediction error smaller: update its beliefs about what is out there (perception, and over longer timescales,
learning), or act so the sensing comes true (behaviour). Goals are not a reward signal; they are preferred observations, things
the duck expects and acts to keep true. A behaviour is what it looks like from outside when one particular error is being
reduced. The sixteen states become sixteen descriptions of what the duck is visibly trying to make true, and adding a behaviour
means adding a prediction, not a mode.

## 1. The design pattern is a set of loops

An active inference loop has four parts, and it runs every tick:

1. **A prediction.** The loop holds a model of what its senses should report: where the head is pointing, how far away the
   wall is, what the floor feels like under a planted foot. Some predictions are preferences, things the loop exists to keep
   true (a level head, the target straight ahead); others are its best guess about the state of the world.
2. **A comparison.** What the senses actually report is set against what was predicted. The difference, the prediction
   error, is the only signal the loop needs. It is weighted by confidence: an error from a sense the loop has learned to
   trust counts for more than one from a sense that is usually noisy.
3. **Two ways to make the error smaller.** The loop can change its mind, revising its estimate of what is out there until
   the prediction fits, or change the world, acting so that the senses come to match the prediction. The first is
   perception and the second is behaviour, and they are one computation run in two directions.
4. **The world closes the loop.** The action moves the body, the body changes what the senses report, and the next tick's
   comparison says whether it worked.

```text
          prediction
              │
              ▼
  senses ──► compare ──► error ──┬──► revise the estimate      (perception)
    ▲                            │
    │                            └──► act                      (behaviour)
    │                                  │
    └────────────── the world ◄────────┘
```

Nothing in the loop names a behaviour; it names only what should be true. A loop that predicts "the thing I am going to is
straight ahead of my head", and senses it 30° to the left, finds that the cheapest way to make its prediction true is to turn
the head. Turning toward things is never written down anywhere.

The duck's brain is many of these at once, at different speeds, with two additions:

- **It learns the predictions it can.** How its walk commands change what it senses, what a kick does to a ball versus a wall,
  where things are in the room: each is learned from the duck's own experience, and each one's error is also its teacher.
- **It acts to reduce uncertainty, not only error.** A loop that only cancelled its current error would be happiest in a
  dark corner, sensing nothing. In active inference an action is valued by what it is expected to bring: the preferred
  observations it moves toward, and the uncertainty it is expected to resolve. The second part is curiosity. The clearest
  case in this implementation is the kick: the duck kicks a kind of thing only while it cannot yet predict what the kick
  will do, and leaves it alone once it can, which is how it gets bored of a block it has kicked a few times. Looking around
  uses a simpler stand-in: the longer a direction has gone unseen, the less the duck knows about it, so the head goes where
  the view is oldest.

When several loops want the body at once (go to the ball; look at the train; back off the wall), one rule (the arbiter) picks which loop
drives the behavior: the one with the most expected value, weighted by how far its past expectations have proved reliable. Active
inference proper scores whole plans several steps ahead; this rule looks one step ahead and is greedy, which is the simplest
version that works here. The control structure is loops that each own an error, arbitrated by that single rule. Expanding the arbiter to plan several steps ahead is ongoing work.

## 2. The sixteen states, as errors

Each runtime state, the error that produces it, and whether this implementation has it today:

| runtime state | the error the duck is reducing | here |
|---|---|---|
| LookAround | directions not seen recently; the head looks where the view is oldest | built |
| Wander | places not visited recently (the runtime's novelty grid is this idea) | built |
| BallPlay — approach, line up, kick | a thing whose response it cannot yet predict; it learns, per kind of thing, what a kick does | built (kick through the shipped skill) |
| GroundPick | the same, for small things, through the peck | built (peck through the shipped skill) |
| TurnInPlace | heading error toward what it is going to | built |
| Chill | nothing has an error worth moving for: it stops, settles, and looks | built |
| Startle | a large, sudden error it did not predict | not yet; the depth sensor is too narrow to make it reliable (§6) |
| Petted, Held, Nap, Zoomies, Preen, Stretch, Ruffle, Sneeze, Dance | bodily and social predictions; presence, mood and the beat enter as inputs, as the shape notes ask | not yet |

The ones not yet built are the invitation, not the gap: each is a prediction to add, and the pattern says how.

## 3. The principles that produced it

The implementation came out of a short set of rules, written down in the project's
[doctrine](https://github.com/xaqmusic/xaq-ai/blob/master/docs/brain_building_doctrine.md) and
[working guide](https://github.com/xaqmusic/xaq-ai/blob/master/CLAUDE.md). They are the reason it looks the way it does, and
they apply to anyone building behaviour on this robot:

1. **Build the error, not the behaviour.** To make the duck look around, the brain was not given a head sweep; it was given
   the direction it has gone longest without seeing, as a target for the head loop it already had. The sweep happens, and
   stops happening when nothing is stale. On the project's earlier robots, behaviours written as scripts (a scripted lift, a
   steering script) fought the gait; the same goals written as errors did not.
2. **If no sensor carries the error, the fix is a sensor.** When a behaviour will not emerge, the first question is whether
   the duck can perceive what it would need to. We work through these issues using real-time sensor data visualizations (our voxel visualization tools are a good example).
3. **What is learned cooperates; what is imposed fights.** Every mechanism that let the body find its own way kept working
   when walking dynamics changed; every mechanism that dictated to it broke the same way.
4. **The robot's own senses only.** No position from the simulator, no ground truth: the brain reads the depth frames, the
   IMU, the joints and its own odometry, which is what it would have on the robot.
5. **One change at a time, off by default, judged on many seeds and by eye.** Every change ships switched off, so the
   previous behaviour is byte-for-byte unchanged until it is turned on; it is compared over 18 to 24 seeds on a full set of
   measures; and nothing is promoted until someone has watched it. A change that failed is recorded with the conditions it
   failed in, so it can be retried when they change.
   
   More details are available here: https://claude.ai/artifact/Dirv1ZWg62pjQVM3SCvkzh

## 4. The boundary

The brain is a client of `robotd`, like `padd`. It subscribes to `robot.state` and the depth frames, sends `move`, `head` and
`do` intents, and `safety` keeps the only write handle. It does not touch the trained policies: it asks them to walk and to
run their skills, and hands the body back when it stops. It is off unless started, and a duck without it behaves exactly as
it does today.

The implementation shown in §5 uses two things that are not in the daemon's API today, and they should be read as open
questions, each to be raised separately in further discussions:

- **The head as the client's.** The head is driven by a small learned loop that holds it level and points it. Through
  `robot.head` the walking policy keeps its own head motion and answers commands 120–160 ms late, and every client-side
  correction measured made the head worse; with the four head joints written directly, the head holds level and the depth
  sensor and camera hold steady.
- **A joint-level stand during a stop.** When the duck stops to look, a standing loop of the brain's own holds the legs. The
  shipped standing policy could do this instead; that substitution has not been measured. Our standing loop is our first attempt at full body control, and this work is ongoing.

Not everything the duck does comes from its predictions yet. Some behaviour is still produced by fixed rules in the client,
each named as a scaffold, a temporary support to be replaced by a learned loop: stops on a timer and on arrival, a gaze sweep
during a stop, backing off and looking up when the walk is impeded, an escape when it is stuck, a reflex that holds its
heading, and fall recovery by the shipped standing policy. Replacing them is part of the work ahead (§9).

**What it costs.** On a desktop CPU the three brains together take about 1 % of one core at 50 Hz (the intent brain 0.2 ms
a tick on average, 1.6 ms at the 99th percentile), and the whole brain process 24 MB of memory. The Radxa Zero 3W's
Cortex-A55 cores are slower by an amount not yet measured; a replay benchmark that runs the brains alone on a recorded
session exists to measure it on the board, as a share of a core and as SoC temperature.

## 5. What it does

In simulation, on this repository's MJCF with the shipped walking and skill policies, in a 4 m square playroom with blocks, balls,
furniture and a toy train on a track; 24 runs of ten minutes, each a different seed. A *boring* second is one spent staring at
a wall or furniture, pressed against it, or walking toward a wall or nothing; an *interesting* second is one spent running a
skill, chasing something that moves, standing at a movable thing, or walking toward one. However, what a person finds
interesting is broader than the measure: the robot looking around, getting into and out of goofy situations, falling, and
generally being a menace. This is why human observation is always required to promote a new behavioural loop or improvement.

- **Interesting 48 % of the time, boring 21 %.** At the start of this work the same measure read 43 % boring; the wall-staring
  it found was fixed by teaching the duck that structure does not answer a kick (it learns that, per context) and by having
  it look up when it is impeded.
- **It goes to things and acts on them:** about 13 arrivals at a thing per run, and 37 % of its skills touched one. In 19 of
  24 runs it touched the green block that stands 1.4 m in front of it at the start, within the first minute.
- **It chases the train** about 13 s per run while the train is moving, making contact 3.7 times per run.
- **It falls** 2.8 times per ten-minute run, recovered by handing the body to the shipped standing policy
  until it is up. This is the number that most needs work before
  hardware.

Video Demo: https://youtu.be/W2jMf6EhW0k

## 6. What the camera would add

The depth sensor sees a 45° cone to about 2 m. With the train moving, it was inside that cone 8 % of the time; the camera's
~62° at room range would have it 19 % of the time, 2.4× as often, measured on the same runs. Everything built to notice
motion on the depth frames changed what the duck did once it saw the train, and nothing changed how often it saw it. A small
motion feature beside the camera — where in the frame something moved, a few times a second, in the "features, not pixels"
direction of [`npu-bringup.md`](../project/npu-bringup.md) — would give the brain a peripheral vision, and startle an input.
It would also be possible to feed small frames to one of the brain's own learned perception modules. This is a separate
conversation.

## 7. Building behaviours with coding agents

Video Overview of Interpretability and Debugging Tools: https://youtu.be/F9kEaafD-vo

The implementation was written almost entirely by an AI coding agent (Claude), working from the project's documents, with a
person choosing the direction and judging the results by eye. The documents are written for that reader:

- a [working guide](https://github.com/xaqmusic/xaq-ai/blob/master/CLAUDE.md) that tells the agent how to turn "the duck
  stares at walls" into an error, a sensor check, a change that ships switched off, and a seeded comparison;
- the [doctrine](https://github.com/xaqmusic/xaq-ai/blob/master/docs/brain_building_doctrine.md), the reasons behind each rule;
- ledgers of every change tried, with its verdict and the conditions it was tried in, so the agent does not re-propose what
  already failed;
- a register of open questions, and a fixed vocabulary for verdicts.

With them, a request like "the duck should look around while it walks" goes from a sentence to a measured, switched-off
change in an afternoon. The same documents would serve anyone here who wants to add a behaviour with an agent.

## 8. Not claimed

- Nothing has run on hardware. Every number is simulation, on this MJCF. The sensors there are exact; the only randomness is a small random offset of every joint at the start (standard deviation 0.05 rad) and the seed of each learner.
- 24 seeds separates a large effect from a small one; it is not a precise measurement, and run-to-run spread is wide (boring
  5–38 % per run).
- No comparison with the runtime's state machine has been made.
- The brain is not a formal active inference agent: it has no single generative model inverted by variational inference,
  and its action selection looks one step ahead.
- The demonstration uses the two capabilities in §4 that the daemon does not offer.

## 9. Next

1. The daemon as a packaged client (`ogma_duckd`), run on a developer board through the existing sideload path, to put the
   first numbers on hardware.
2. The two open questions in §4 and the camera feature in §6, each as its own issue.
3. The scaffolds in §4, one at a time, each replaced by a loop that owns the error it stands in for.
4. The states not yet built, starting with those whose inputs the daemon already publishes: petted (the pet detector),
   presence and greeting (the chorale beacon), the beat. xaq-ai has roots in the music industry, so we are excited to get
   Microducks out on the dance floor!
