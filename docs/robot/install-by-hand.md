# Installing by hand, step by step

What `scripts/provision-board.sh` does, as separate commands. Use this when a step needs to be
tested on its own; use `provision-board.sh` when you just want a working board.

## Copy the files up

From a clone on your machine. Into `~`, **not** `/tmp` — there is a reboot in the middle and
`/tmp` does not survive it.

```bash
scp scripts/setup-board.sh scripts/migrate-network.sh pierre@192.168.1.42:~/
```

```bash
scp scripts/install.sh deploy/dev-key/team.dev.pub pierre@192.168.1.42:~/
```

## Before the reboot

The `robot` group first, so membership is live after the reboot rather than needing another one:

```bash
sudo groupadd --system robot
```

```bash
sudo usermod -aG robot "$USER"
```

Board bring-up — device-tree overlay, kernel console off the motor UART, the getty mask,
`Privacy = device`, onnxruntime:

```bash
sudo sh ~/setup-board.sh
```

Network — netplan to NetworkManager:

```bash
sudo sh ~/migrate-network.sh
```

```bash
sudo reboot
```

The reboot is not optional: a device-tree overlay and a network stack cannot swap under a running
kernel.

## After the reboot

Both again. They are idempotent, and the second `migrate-network.sh` run is what retires the wifi
backstop that would otherwise revert this board to netplan on any boot where wifi is slow:

```bash
sudo sh ~/setup-board.sh
```

```bash
sudo sh ~/migrate-network.sh
```

Then the daemon. `install.sh` reads its settings from the environment, and `sudo -E` is what gets
them through:

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
export DUCK_REF=main
```

```bash
export DUCK_DEV_KEY=$HOME/team.dev.pub
```

```bash
sudo -E sh ~/install.sh
```

Drop `DUCK_DEV_KEY` for a board that should only take releases. Set `DUCK_REF` to a branch to
install what that branch last built.

Name it, if you want a name:

```bash
robotctl system set-name duck-01
```

## Check it

```bash
robotctl health
```

```bash
robotctl version
```

`robotd` reporting unhealthy on a bench board with unpowered servos is the honest answer, not a
failed install.

Then the gamepad — [`pair-a-gamepad.md`](pair-a-gamepad.md):

```bash
sudo robotctl pad pair
```
