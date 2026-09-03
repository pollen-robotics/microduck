# Dance gait (phase 3)

Trained on Spark (`/home/mstaub/jobs/feat-dancing-skill-1bf7770a/microduck_rl`),
not in this tree. The env cfg is copied here so the worktree has the recipe.

- Task id: `Mjlab-Dance-Flat-MicroDuck`
- Command slots: `body_x=sin(2πφ)`, `body_y=cos(2πφ)`, `body_yaw=energy`
- Smoke: 64 envs × 5 iters succeeded 2026-09-02 23:38 ET
- Full: 4096 envs, 15k iters, `WANDB_MODE=offline`, tmux `dance-train-1bf7770a`
- Finished: `TRAIN_EXIT 0` 2026-09-03 15:14 ET; checkpoint `model_14999.pt`
- ONNX (normalizer baked): `dance_gait_iter14999.onnx` — `obs[1,61] → actions[1,14]`
- Preview: `dance_gait_iter14999.mp4`

On a duck, the crouch overlay (`[audio] dance`) uses `alpha_stand`. To run **this**
net instead:

```toml
[policy]
stand = "/path/to/dance_gait_iter14999.onnx"

[audio]
dance = true
dance_gait = true   # body_x=sin, body_y=cos, body_yaw=energy; z stays 0
```

Do not set `dance_gait` while `stand` is still `alpha_stand.onnx`.

Export only via `uv run scripts/export.py` (bakes the obs normalizer).
