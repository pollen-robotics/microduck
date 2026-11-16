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

`DUCK_WEIRD_BLE=1` on the `setup-board.sh` runs above is `--weird-ble`: for a board whose Bluetooth
cannot bond a gamepad at all. See [`pair-a-gamepad.md`](pair-a-gamepad.md).

`DUCK_NO_START=1` installs the release, the units, the users and the groups and enables **nothing**
— not even for the next boot. It also stops and disables any daemon a previous install left
running, so the state is the same whether the card is fresh or not. For separating a board-level
fault from the daemons: reboot into a board with nothing of ours running, test, then bring them up
one at a time.

```bash
sudo -E DUCK_NO_START=1 sh ~/install.sh
```

```bash
sudo reboot
```

The reboot is part of it, not tidiness. The release's own `hooks/postinstall` enables and starts
every daemon before `install.sh` can stop them, so they have run on that boot whatever the knob
says — and a daemon does not undo what it pushed to a subsystem when it dies. `btd` leaves
`Pairable` set, an advertising instance, and the IO capability its default pairing agent gave the
adapter.

To make it a working robot again:

```bash
sudo systemctl enable --now updaterd robotd configd btd padd autod
```

Name it, if you want a name:

```bash
robotctl system set-name duck-01
```

## GStreamer, for a board that will stream

Provisioning does this by default — `--no-gstreamer` on `provision-board.sh`
(`DUCK_GSTREAMER=0` here) skips it. No reboot either way, so it can be run by hand at any time
on a board that skipped it.

```bash
scp scripts/setup-gstreamer.sh pierre@192.168.1.42:~/
```

```bash
sudo sh ~/setup-gstreamer.sh
```

It prints what this board can encode — which `webrtc*` elements are registered, which H.264
encoder is reachable, and whether the VPU is exposed by the running kernel. On a Zero 3W it
reports `/dev/mpp_service` and no `v4l2h264enc`, which is the expected shape for a Rockchip BSP
kernel: the VPU is reached through MPP rather than V4L2, and the report names the two Radxa debs
that prove it encodes. Re-run it after any kernel change; that is the event that changes the
answer:

```bash
sudo /usr/local/sbin/robot-setup-gstreamer
```

Add `--dev` to also install the headers, for building `gst-plugin-webrtc` on the board or an
aarch64 sysroot to cross-build against.

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
