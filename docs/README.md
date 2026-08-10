# Docs

Start at the [README](../README.md) if you have a robot and want to use it.

## `robot/` — you have a robot

| | |
|---|---|
| [`cheatsheet.md`](robot/cheatsheet.md) | Every `robotctl` command. |
| [`cheatsheet-dev.md`](robot/cheatsheet-dev.md) | The commands that need a dev board: branch builds, candidates, `btctl`. |
| [`install-dev.md`](robot/install-dev.md) | Setting up a board for development, from nothing. |

## `design/` — you are changing the daemon

How it works and why. These change rarely; when behaviour and a design doc disagree, the doc is
the bug.

| | |
|---|---|
| [`architecture.md`](design/architecture.md) | The service split, the IPC contract, state ownership, safety and authority. |
| [`robotd-design.md`](design/robotd-design.md) | The control loop: model, bus, sensing, observations, policy, safety. |
| [`updater-design.md`](design/updater-design.md) | The update engine: verification, atomic swap, health gate, rollback, release format. |
| [`app-path-design.md`](design/app-path-design.md) | `btd` and `configd` — how a phone configures a robot over BLE. |

## `project/` — you are running the project

Dated records rather than reference. They describe a moment, and go stale on purpose.

| | |
|---|---|
| [`roadmap.md`](project/roadmap.md) | Milestones, and what works today versus what is designed. |
| [`ci-setup.md`](project/ci-setup.md) | One-time setup for the release pipeline: keys, secrets, rotation. |
| [`install-path-gap.md`](project/install-path-gap.md) | Why four install-path bugs reached a board, and what would close it. Open. |
| [`slice-2-bringup.md`](project/slice-2-bringup.md) | What a real Radxa Zero 3W did with slice 2. |

## Elsewhere

| | |
|---|---|
| [`../CONTRIBUTING.md`](../CONTRIBUTING.md) | Building, testing, repo layout, conventions, releasing. |
| [`../deploy/README.md`](../deploy/README.md) | What a robot image is configured with, and what provisioning actually does. |
