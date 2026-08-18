# The minimal setup a gamepad bonds under

Recorded 2026-08-18, on Radxa Zero 3W `50:37:CD:16:2A:39`, with an Xbox Wireless Controller
`78:86:2E:92:47:67`. Confirmed on a second Zero 3W with the same card.

## The sequence that works

Flash Armbian for Radxa Zero 3, Minimal. Fill in wifi and the username in the imager. Install
nothing else — no daemons, no provisioning.

```bash
sudo sed -i -E 's|^[[:space:]]*#?[[:space:]]*Privacy[[:space:]]*=.*|Privacy = device|' /etc/bluetooth/main.conf
```

```bash
grep -n "^Privacy" /etc/bluetooth/main.conf
```

```bash
sudo reboot
```

A reboot rather than `systemctl restart bluetooth`: the restart sometimes leaves the kernel
holding hci0 with `No default controller available`, and only a reboot clears that.

Hold the pad's pair button until it blinks fast, then:

```bash
bluetoothctl
```

```
scan on
```

Wait for `[NEW] Device 78:86:2E:92:47:67 Xbox Wireless Controller`, then:

```
scan off
```

```
connect 78:86:2E:92:47:67
```

Answer `yes` to `Request authorization`, then:

```
trust 78:86:2E:92:47:67
```

`pair` is never typed.

## What it looks like when it worked

```bash
ls /dev/input/js*
```

```bash
dmesg | tail -3
```

```
input: Xbox Wireless Controller as /devices/virtual/misc/uhid/0005:045E:0B13.0001/input/input5
microsoft 0005:045E:0B13.0001: input,hidraw0: BLUETOOTH HID v5.09 Gamepad [Xbox Wireless Controller]
```

Input actually flowing — move the left stick throughout, and look past the first 184 bytes for
events with `type 0x02` and advancing timestamps:

```bash
sudo timeout 5 cat /dev/input/js0 | od -Ad -tx1 | head -20
```

The bond survives a pad power cycle (hold the Xbox button ~6 s, then switch back on):

```bash
ls /dev/input/js*; bluetoothctl info 78:86:2E:92:47:67 | grep -E "Connected|Bonded|Trusted"
```

## The board it worked on

Not the configuration anyone expected, which is why each is written down:

| | |
|---|---|
| `cat /sys/module/aic8800_bsp/srcversion` | `738316A2E9D9825966BDB6B` (86016) |
| `conn_min_interval` / `conn_max_interval` | 24 / 40 — kernel defaults, 30–50 ms |
| `/etc/bluetooth/main.conf` | `Privacy = device` |
| daemons running | none |

The driver is the build `design/pad-bond-failure.md` calls broken. The connection interval is
untouched. Neither is what decides whether a pad bonds.

## What has failed

| setup | result |
|---|---|
| Bare Armbian, `Privacy = off` (or unset — BlueZ defaults to `off`) | `connect` returns `le-connection-abort-by-local`; `Paired: no`; no SMP exchange at all |
| Bare Armbian, no `Privacy` line, leading with `pair` instead of `connect` | bonds, then every reconnect fails `Encryption Change: PIN or Key Missing (0x06)`, flapping ~1/s |
| Provisioned board (`Privacy = device` confirmed at line 99), `padd` and `btd` stopped, manual `connect` | `Request authorization` accepted, then `ServicesResolved: no` — the pad drops immediately, no `js0` |

The third row is the open one: the same card image and the same `Privacy` value fail once the
board has been provisioned. So it is something provisioning changes rather than a process that
happens to be running — stopping `padd` and `btd` did not bring pairing back.

## Making a bond and keeping one are different

On a fully provisioned board with everything running, a pad that is **already bonded** connects
and drives. A **fresh** `robotctl pad pair`, after `pad forget` and with the pad held in pairing
mode, fails.

So nothing here breaks an existing bond. Something in the installed system stops a new one being
made. `configd` and `btd` logs say nothing useful about it.

## Tomorrow: add one thing at a time

From a board reflashed and confirmed working by the sequence at the top of this page, add one
layer, reboot, and try a fresh pairing before adding the next.

Copy the scripts to `~`, **not** `/tmp` — every step here reboots, and `/tmp` does not survive
one. Same for any `btmon` capture worth keeping.

```bash
scp scripts/setup-board.sh scripts/migrate-network.sh pierre@BOARD:~/
```

| step | what it adds | pad pairs? | notes |
|---|---|---|---|
| 0 | nothing — the minimal sequence above | yes | the control |
| 1 | `sudo sh ~/setup-board.sh` — overlays, `console=display`, getty mask, onnxruntime | | |
| 2 | `sudo sh ~/migrate-network.sh` — netplan → NetworkManager | | |
| 3 | `sudo -E sh ~/install.sh`, then `systemctl disable --now updaterd robotd configd btd padd` | | |
| 4 | `systemctl enable --now updaterd` | | |
| 5 | `... robotd` | | |
| 6 | `... configd` | | |
| 7 | `... btd` | | |
| 8 | `... padd` | | |

Reboot after each step, and clear **both** halves of the bond before each attempt: `pad forget`
or `bluetoothctl remove` on the board, and the pad held in pairing mode. An Xbox pad keeps one
host bond, and a half-completed attempt leaves it holding a key the board no longer has — which
looks exactly like the fault.

Step 3 needs the environment `install.sh` reads:

```bash
export DUCK_TOKEN=github_pat_replace_with_your_token
```

```bash
export DUCK_REF=pad-privacy-device-not-off
```

```bash
export DUCK_DEV_KEY=$HOME/team.dev.pub
```

Steps 1 and 2 need neither a token nor the network.

## Untested differences against `microduck_runtime`

`microduck_runtime`'s installer disables wifi powersave on the active NetworkManager connection
(`install.sh:244`, `:383`):

```
sudo nmcli con mod "$WIFI_CON" wifi.powersave 2
```

`scripts/` has no equivalent. The aic8800 is a combined wifi and Bluetooth part sharing one radio
over SDIO, so this is a candidate for the third row above — but the value on the bare board that
worked was never read, so it is a candidate and not a finding.

```bash
iw dev wlan0 get power_save
```
