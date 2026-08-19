# Docs

Start at the [README](../README.md) if you have a robot and want to use it.

## `robot/` — you have a robot

| | |
|---|---|
| [`cheatsheet.md`](robot/cheatsheet.md) | Every `robotctl` command. |
| [`pair-a-gamepad.md`](robot/pair-a-gamepad.md) | Once per pad: pairing mode, `pad pair`, and what to do when it will not bond. |
| [`cheatsheet-dev.md`](robot/cheatsheet-dev.md) | The commands that need a dev board: branch builds, candidates, dev pushes. |
| [`dev-push.md`](robot/dev-push.md) | Build on your machine and install on the board over ssh, with no CI run. |
| [`duck-btctl.md`](robot/duck-btctl.md) | Every `duck-btctl` command — the robot over Bluetooth, from a laptop. |
| [`install-dev.md`](robot/install-dev.md) | Setting up a board for development, from nothing. |
| [`install-by-hand.md`](robot/install-by-hand.md) | The same install as separate commands, for testing one step at a time. |

## `design/` — you are changing the daemon

How it works and why. These change rarely; when behaviour and a design doc disagree, the doc is
the bug.

**One page owns a mechanism, and the others link to it.** The table below is that assignment: if a
fact belongs to a page listed here, every other page says one sentence and points, rather than
explaining it again. A fact written down in six places drifts in six directions, each of them locally
reasonable — which is how six documents came to promise that `updaterd` and `btd` kept their old
binaries until the next reboot, two releases after they stopped doing so, including the two pages
someone reads while diagnosing exactly that. So when two documents disagree, the one that does not
own the mechanism is the bug.

| | |
|---|---|
| [`architecture.md`](design/architecture.md) | The service split, the IPC contract, state ownership, safety and authority. |
| [`robotd-design.md`](design/robotd-design.md) | The control loop: model, bus, sensing, observations, policy, safety. |
| [`updater-design.md`](design/updater-design.md) | The update engine: verification, atomic swap, health gate, rollback, release format. |
| [`restart-order.md`](design/restart-order.md) | Which unit restarts, at which step, on every path that moves `current` — and at boot. |
| [`app-path-design.md`](design/app-path-design.md) | `btd` and `configd` — how a phone configures a robot over BLE. |
| [`boot-recovery-net.md`](design/boot-recovery-net.md) | Falling back to golden when the release that booted cannot start its daemons. |

## `project/` — you are running the project

Dated records rather than reference. They describe a moment, and go stale on purpose.

| | |
|---|---|
| [`roadmap.md`](project/roadmap.md) | Milestones, and what works today versus what is designed. |
| [`ci-setup.md`](project/ci-setup.md) | One-time setup for the release pipeline: keys, secrets, rotation. |
| [`install-path-gap.md`](project/install-path-gap.md) | Why four install-path bugs reached a board, and what closed it. Closed. |
| [`slice-2-bringup.md`](project/slice-2-bringup.md) | What a real Radxa Zero 3W did with slice 2. |

## Elsewhere

| | |
|---|---|
| [`../CONTRIBUTING.md`](../CONTRIBUTING.md) | Building, testing, repo layout, conventions, releasing. |
| [`../deploy/README.md`](../deploy/README.md) | What a robot image is configured with, and what provisioning actually does. |
