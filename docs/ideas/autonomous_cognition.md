# A smarter duck — layers, a world model, and a behavior library

> Holding pen, not a decision. [`design/autonomous.md`](../design/autonomous.md) owns `autod` as it
> is; [`autonomous_behavior.md`](autonomous_behavior.md) holds the social and musical ideas. This
> page is about the duck doing things on purpose — *go to the next room* — and about letting
> anyone build behaviors on top of it.

Two audiences want the same thing. We want to build a really smart autonomous duck; a user wants
to *vibe code* a new behavior of their own. Both need the duck to have concepts worth composing
and a vocabulary for composing them.

## 1. Three layers, three speeds

| layer | where | pace | what |
|---|---|---|---|
| Reflexes | on board | 50 Hz | gait, ToF obstacle gate, fall handling, gaze, head bob, breathing |
| Behaviors | on board | seconds | wander, investigate, go to a place, play ball, nap — each with a goal and an end |
| Deliberation | remote, big model | event-driven, slow | what is worth doing next, and why; what this place is |

The first two exist today as `autod`'s brain, minus gaze. The third is new.

**Never freeze; smarter when online.** The on-board layers guarantee a duck that is alive: it
looks, wanders, plays and naps with no network. Being online *adds* abilities — labelling places,
going to a named place, choosing goals, conversation — which are simply not there offline rather
than degraded copies. The remote model never holds the duck up: a stalled call leaves it doing
what it was doing.

**Event-driven, not a clock.** What wants 1–2 Hz — where to look, what just moved — is attention,
and belongs on the board. Decisions change on the scale of tens of seconds. So the remote model is
called on events (arrived somewhere new, a person appeared, bored, asked something) plus a slow
background tick. A frame of somebody's home sent every second is also a privacy question
(`architecture.md` §7), and three thousand calls an hour is a bill.

## 2. Gaze is the first thing missing

The duck's head moves on a schedule — drifts, glances, the survey sweep — never *toward*
something. Gaze is most of what reads as alive: turning to look at you, holding it, losing
interest.

- **An attention layer, not new states.** One gaze target chosen every tick by salience — a
  person, then fresh motion, then another duck, then the brain's own glance — blended under
  whatever state runs, the way the walking head bob is. Chill becomes chill while watching you.
- **People** want a person class in the detector: one model for duck and person, one inference,
  the same thermal cost.
- **Interesting things are motion, not categories.** Animals orient to change. Open-vocabulary
  detection does not fit the NPU, and "something moved, glance at it" reads better than
  "recognised a mug".
- **Investigate before grab.** A failed grasp looks broken; a peck looks alive. Approach, look
  down, peck or nudge, react.

## 3. Higher cognition is mostly memory

*Go to the next room* needs the duck to know there are rooms, which one it is in, and where the
doors are. The map knows geometry at ToF height; it has no kitchen, no doorway, no "under the sofa
where the ball went". An LLM told to go to the next room has nothing to point at.

**A VLM is good at exactly what the board cannot do.** Shown a frame at a survey stop, it says
*kitchen, a doorway on the left, somebody sitting*. The board keeps the geometry, the remote model
names places, the brain walks there. Labels stick to the map, so tomorrow the duck still knows
where the kitchen is — offline too: what was learned online stays. Persistent memory reads as
intelligence far more than clever reasoning does.

This waits for the next `mapd`: the current one does not hold across rooms, and nothing
cross-room is worth building on it. The semantic layer should speak to the map in terms of
*places and the way between them*, not its internals, so it carries over.

## 4. The behavior library

Each brain state made into a unit with a goal, a success or failure, and a time limit —
`look_at(person)`, `go_to(place)`, `investigate(bearing)`, `wander(30 s)`, `nap`, `play_ball`,
`greet`. It is worth doing for its own sake:

- **It makes our own brain better.** States today are one state machine nothing else can reuse.
  As units, each is testable alone in the twin, measurable ("investigate succeeds 70 % of the
  time"), and composable.
- **It is what makes vibe coding safe.** The primitives carry the invariants — the obstacle gate,
  never walking backwards, yielding to the pad, the get-up watch — so nothing written on top can
  walk the duck off a table.
- **It is one vocabulary** for our brain, a user's script, the app and an LLM.

It is an internal API, free to change: sharing behaviors on the Hub is not a goal.

**Two ways to vibe code, on the same library:**

- **Authoring.** Somebody describes a behavior, an LLM writes a small program against the
  library, and it runs on the robot with no LLM in the loop — reliable, free to run, offline.
  The default.
- **Runtime composition.** Talk to the duck, and a model chains behaviors live. Good for demos
  and conversation; online only, per §1.

**The LLM goes above the brain, not beside it.** It asks `autod` for a behavior; the brain carries
it out under its own energy, obstacles and pad handoff. Another client driving `robotd` directly
would need the arbitration layer `architecture.md` §6 still wants, and would leave two brains
fighting over the legs.

## 5. Borrowed from the Reachy Mini conversation app

[`reachy_mini_conversation_app`](https://github.com/pollen-robotics/reachy_mini_conversation_app)
has solved the conversational half for a robot on a desk.

- **Layered motion.** The model queues primary moves while secondary motion — breathing, speech
  wobble — always runs underneath, so the robot never freezes while it thinks.
- **Fast local cues, slow remote thought.** Local VAD reacts at once; a small local model picks
  emotions without a tool call. The robot reacts before the model has decided anything.
- **Doing nothing is a tool.** The model is allowed to choose idle, which keeps it from being
  hyperactive.
- **Profiles** — a prompt and the tools it may use — as the unit of personality. A duck's would add
  the brain's weights.
- **Remote tools as MCP Spaces.** Abilities added without touching the robot.

The difference that matters: **on Reachy Mini the model is the life; on the duck it directs a life
that already exists.** A desk robot whose model goes quiet just sits there. A walking duck whose
model goes quiet must keep being a duck.

## 6. An order

1. Gaze, on board. Independent of everything else, and the largest step in feeling alive.
2. Brain states as behaviors with outcomes, tested in the twin.
3. A remote director: event-driven, choosing among behaviors, free to do nothing, online only.
4. Semantic places, once the next `mapd` is in.
