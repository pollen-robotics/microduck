# The policy manifest, schema 2

What a `manifest.json` beside a microduck `.onnx` says, and what the robot does with each field.
One vocabulary for two shapes: a **single-policy repo** (`<user>/microduck-<name>` on the Hub,
one `policy.onnx`, the fields at the top level) and the **official set**
(`pollen-robotics/microduck-policies`, nine files, the same fields once per entry under
`policies`). One reader understands both, and asking a publisher for something is "add a field",
never "adopt our format".

`uv run publish` in `pollen-robotics/microduck_rl` writes a conforming single-policy repo from a
checkpoint or an ONNX file. `robotctl policy load <slot> <repo>` and `robotctl policy add <name>
<repo>` read it. `docs/design/policy-channel-design.md` §9 has the reasoning; this file is the
contract.

## The two axes

**`kind` says who ends the policy.**

| `kind` | means | becomes |
| --- | --- | --- |
| `episodic` | runs for `duration_s` and returns itself to a safe pose | a skill, if its command is constant |
| `perpetual` | runs until told otherwise — a gait, or a pose a person has to end | a slot's gait (`policy load`); or, with `unwind_s`, a skill with `--hold` |
| `scripted` | episodic but interruptible: the daemon can change its command mid-flight | recorded; the daemon's own arm reads its timing |

**`command.encoding` says what the daemon feeds it.**

| `encoding` | the twist | who |
| --- | --- | --- |
| absent or `constant` | a fixed twist for the window, `idle` on the way back | every kick, the roulade, every community one-shot so far |
| `phase` | `[cos 2πφ, sin 2πφ, 0]`, φ from 0 over `period_s` s, hands back at `end_phase` | the ground pick; the roller crouch |
| `posture_flag` | one slot carries `sit` or `stand` | the sit↔stand |

Only a constant-command `episodic` policy can be a generic one-shot. A `phase` or `posture_flag`
policy is refused by `policy add` and belongs in the slot the daemon drives (`policy load
ground_pick …`, `policy load sitstand …`). The daemon guards this on the encoding, not the name.

## Fields

Everything is optional except `file` inside a set. Absent fields are not evidence: a robot refuses
a policy only on a claim that is present and wrong.

| field | type | read by | meaning |
| --- | --- | --- | --- |
| `schema_version` | int | display | `2`. A superset of 1; nothing gates on it |
| `model_api` | int | fetch | daemon API the policy needs; refused if newer than the daemon's |
| `obs_len` | int | fetch | `61`; refused if it disagrees with the robot |
| `action_len` | int | fetch | `14`; refused if it disagrees |
| `robot.model` | str | fetch | `microduck`; refused if another robot |
| `robot.hw_rev`, `robot.servos`, `robot.control_hz` | | display | `1`, `xl330`, `50` |
| `name` | str | skills | what a client asks for; defaults to the file's stem |
| `description` | str | display | one line, untrusted |
| `kind` | str | skills, slots | see above |
| `duration_s` | float | skills | seconds it runs; for a phase policy `period_s × end_phase` |
| `chain` | bool | skills | a held button starts another run when this one ends |
| `action_scale` | float | skills | its own output scale while it runs |
| `unwind_s` | float | skills, sitstand | seconds driving `command.idle` before handing back; for the sit↔stand, the rise |
| `ramp_s` | float | sitstand | seconds the seat takes to settle; the shutdown sit waits twice this |
| `mode` | str | set | `walk` (default) or `roller`; which mode's ground pick a phase entry is |
| `slot` | str | display | for a perpetual gait: the slot it is for (`walk`, `stand`, …), so `policy load <slot> <repo>` is the install line |
| `entry_pose` | str | display | the pose the policy expects to start from, e.g. `standing` |
| `command.encoding` | str | skills | see above |
| `command.idle` | [3] | skills | the twist that means "stop doing the thing" |
| `command.period_s`, `command.end_phase` | float | ground pick | the phase cycle |
| `command.sit`, `command.stand`, `command.slot` | | display | the flag's values and where it rides |
| `command.twist`, `command.head`, `command.body` | prose | display | what each block means to a person |
| `training` | object | display | `task_id`, `repo`, `commit`, `branch`, `dirty`, `run`, `checkpoint`, `exported` |
| `eval` | object | display | free-form: what was checked, in what sim, how it did |
| `policies[]` | array | set | one entry per file, with `file` plus any per-policy field above |

## A single-policy repo

```json
{
  "schema_version": 2,
  "model_api": 1,
  "obs_len": 61,
  "action_len": 14,
  "robot": { "model": "microduck", "hw_rev": 1, "servos": "xl330", "control_hz": 50 },
  "name": "polite-bow",
  "kind": "episodic",
  "duration_s": 4.0,
  "chain": false,
  "entry_pose": "standing",
  "description": "Bows from a two-foot stand and comes back up.",
  "command": { "encoding": "constant", "idle": [0, 0, 0],
               "twist": "unused (zeros)", "head": "unused (zeros)", "body": "unused (zeros)" },
  "training": { "task_id": "Mjlab-PoliteBow-Flat-MicroDuck", "repo": "pollen-robotics/microduck_rl",
                "commit": "0bf9897", "branch": "bow", "dirty": false,
                "run": "pollen-robotics/mjlab_microduck/abc123", "checkpoint": 3000,
                "exported": "2026-09-02T14:05:00Z" }
}
```

The repo carries exactly one `.onnx`, named `policy.onnx`. `robotctl policy add polite-bow
<user>/microduck-polite-bow` reads `duration_s`, `chain`, `action_scale`, `command.idle` and
`unwind_s`, refuses on `obs_len`, `action_len`, `model_api`, `robot.model` or a non-constant
encoding, and writes the skill. `RemiFabre/microduck-flamingo-cycle` is a published `perpetual`
example: no `duration_s`, so `--hold` is required, and its `command.idle` is what the unwind
drives.

## The official set

The same fields, once per entry, under `policies`, plus `file`. The live copy is
`https://huggingface.co/pollen-robotics/microduck-policies/blob/main/manifest.json`; the set's
`phase` entries set each mode's ground-pick timing and its `scripted` entry the sit↔stand's,
which is how a retrained pick with a longer cycle is a tag rather than a daemon release.

Every `file` is a plain file name — no directory, no leading dot. A set whose manifest names one
with a path in it has that entry skipped, by the seeder and by `robotctl policy update` both.

The manifest is installed **into the set**, so `/opt/robot/policies/current/manifest.json` is
what `robotd` reads for the skills. It is also the download list: adding a policy is an entry
here and a tag, and both the first seed of a board and every `policy update` after it take the
list from the revision they are installing.

## Changes from schema 1

Schema 1 was the official set's first shape: `kind` with the values `perpetual`, `episodic`,
`scripted`, where `scripted` meant "the daemon generates the command" and was applied to the
ground pick. Schema 2 moves that meaning to `command.encoding`, makes `scripted` mean
interruptible-episodic (the sit↔stand), and adds `chain`, `ramp_s`, `mode`, `entry_pose`,
`training`, `eval`, and the `command.*` timing fields. A daemon reading a schema-1 file gets the
prototype's timing for everything and the same three skills; nothing is refused on the version.
