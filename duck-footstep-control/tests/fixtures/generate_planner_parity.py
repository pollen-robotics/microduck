"""Regenerate planner_parity.json: the reference planner's rollouts, step by step.

    cd <microduck_rl checkout> && uv run python <this file> <footstepnet.onnx> <out.json>

The reference is `src/mjlab_microduck/footstep_planner.py` in microduck_rl (itself checked step-for-step against
FootstepNet's gym env). Each case records, per planned step, the observation the network saw, its raw output and the
landing pose, so the Rust glue (observation, mirror, clips, neutral step, rollout, reached test) is checked against a
stub network that replays the outputs — no ONNX Runtime needed in the test.
"""

import json
import math
import sys

import numpy as np

from mjlab_microduck.footstep_planner import FootstepPlanner

planner = FootstepPlanner(sys.argv[1])
recorded = []
run = planner.session.run


def recording_run(outputs, feeds):
    out = run(outputs, feeds)
    recorded.append((feeds["obs"][0].tolist(), out[0][0].tolist()))
    return out


planner.session.run = recording_run
rng = np.random.default_rng(7)
cases = []
fixed = [  # (support pose, support foot, goal pose, goal foot, obstacle)
    ((0.0, 0.0, 0.0), "left", (0.6, 0.3, math.pi / 2), "right", None),
    ((0.0, 0.0, 0.0), "right", (-0.3, 0.0, 0.0), "right", None),
    ((0.1, -0.2, 2.5), "left", (0.1, -0.2 + 0.095, 2.5), "right", None),  # already beside: one step or none
    ((0.0, 0.0, 0.0), "right", (0.5, 0.0, 0.0), "left", (0.25, 0.0, 0.08)),
]
for _ in range(8):
    fixed.append((tuple(rng.uniform([-1, -1, -math.pi], [1, 1, math.pi])), rng.choice(["left", "right"]),
                  tuple(rng.uniform([-1, -1, -math.pi], [1, 1, math.pi])), rng.choice(["left", "right"]),
                  (tuple(rng.uniform([-1, -1, 0], [1, 1, 0.12])) if rng.uniform() < 0.5 else None)))
for support, foot, goal, goal_foot, obstacle in fixed:
    recorded.clear()
    steps, reached = planner.plan(np.array(support), foot, np.array(goal), goal_foot, obstacle, max_steps=50)
    cases.append({
        "support": list(support), "support_foot": foot, "goal": list(goal), "goal_foot": goal_foot,
        "obstacle": list(obstacle) if obstacle else None, "reached": bool(reached),
        "steps": [{"obs": o, "net": n, "landing": p.tolist(), "foot": f} for (o, n), (p, f) in zip(recorded, steps)],
    })
json.dump({"options": planner.options, "cases": cases}, open(sys.argv[2], "w"), indent=1)
print(f"{len(cases)} cases, {sum(len(c['steps']) for c in cases)} steps → {sys.argv[2]}")
