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
| 1 | `sudo sh ~/setup-board.sh` — overlays, `console=display`, getty mask, onnxruntime | **yes** | |
| 2 | `sudo sh ~/migrate-network.sh` — netplan → NetworkManager | **yes** | |
| 3 | `sudo -E sh ~/install.sh` | **no** | first attempts; the daemons were running |
| 3b | `DUCK_NO_START=1`, then reboot | **yes** | so nothing install.sh writes is at fault |
| 4 | `systemctl enable --now updaterd` | | |
| 5 | `... robotd` | | |
| 6 | `... configd` | | |
| 7 | `... btd` | | |
| 8 | `... padd` | | |

Ran 2026-08-19, pairing manually with `bluetoothctl` at each step and removing the pad again
afterwards. Steps 1 and 2 bond fine; `install.sh` is where it stops. So the board bring-up, the
device-tree overlays, the console move and the NetworkManager cutover are all exonerated.

Split inside `install.sh` on 2026-08-19: with `DUCK_NO_START=1` and a reboot, a pad bonds. So
nothing `install.sh` writes to disk is at fault — not the units, the users, the groups, the release
tree or the token drop-in. It is one of the five daemons, running.

Three attempts at that measurement were wasted before it worked, all for the same reason:
`hooks/postinstall` inside the release does `systemctl enable --now` on every unit it ships, from
inside `updaterd install`, which happens *before* `install_units` — so the daemons were up during
every test, and `systemctl disable --now` afterwards does not undo what they pushed to the adapter.
`btd` leaves `Pairable` set, an advertising instance, and the IO capability its default pairing
agent gave the adapter. **Reboot before measuring.**

## It is `btd`

2026-08-19, one variable, reproducible both ways on `2A:39`:

| | |
|---|---|
| `btd` enabled, reboot, pad reset, fresh pair | **fails** |
| `btd` disabled, reboot, pad reset, fresh pair | **works** |

`/var/lib/bluetooth` was **not** wiped for the working run, so the two files `btd` causes BlueZ to
write — `attributes`, the persisted local GATT database, and `identity`, the adapter's local IRK —
are both exonerated. So is anything else persisted.

An earlier round where disabling `btd` did *not* restore pairing was the pad's own bond slot: an
Xbox pad holds one host bond, and a half-completed attempt leaves it holding a key the board no
longer has. **Reset the pad on a laptop between attempts**, or the fault is indistinguishable from
a poisoned pad — this cost most of two days.

Nothing running keeps an existing bond from working: a bonded pad connects and drives with the
whole stack up. Only *making* a bond fails.

### The mechanism, not yet measured

`btd` advertises continuously as a peripheral. With `Privacy = device` that advertising uses a
resolvable private address, while the same adapter acts as central to bond the pad. The SMP DHKey
check is computed over both devices' addresses, so this is the shape that produces
`DHKey check failed (0x0b)` — the failure that made this tree abandon `Privacy = device` in the
first place, at a time when `btd` was running. A `btmon` capture of a failing pair, read for own
address type and the SMP failure reason, settles it.

### Two faults, and they were being read as one

Re-tested with a reset pad, so neither result rests on a poisoned bond slot:

1. **The board split.** On a fresh Armbian with nothing installed, some Zero 3W units bond a pad
   under BlueZ's default `Privacy = off`. Others do not bond at all under `off`, and only `device`
   works. Roughly half of ten in each group, with nothing measurable separating them.
2. **`device` versus `btd`.** Under `device`, a pad cannot form a new bond while `btd` advertises.

Together they explain the `DHKey check failed (0x0b)` that made this tree set `Privacy = off` in the
first place: that was fault 2, measured on a board that needed `device` for fault 1, and it was read
as proof that `device` broke pairing. The result was a setting that could not bond a pad on half the
boards, for a fortnight.

Stopping `btd` is not sufficient on its own: what it pushed to the controller outlives the process,
so every manual pairing that worked had a **reboot** after the stop. An adapter power cycle
(`bluetoothctl power off && power on`) substitutes for that reboot — measured 2026-08-19, `pad pair`
succeeding first try on a fully installed board — which is what keeps pairing one command.

Both workarounds are behind `provision-board.sh --weird-ble`, so a board that does not need them
does not carry them: the flag sets `Privacy = device` and leaves `/var/lib/robot/weird-ble`, and
`robotctl pad pair` pauses `btd` only on a board with that marker. Try a board without the flag
first. Both go when the aic8800 does.

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

## Fault 3: a bonded pad cannot reconnect while a phone is connected  · **measured** (2026-09-17)

The two faults above are about *bonding*. This one is about coming back, it is separate, and it is
what somebody using the phone app meets first.

On `lavandiere` — provisioned `--weird-ble`, so `Privacy = device` — with an Xbox pad already
bonded and trusted:

| | |
|---|---|
| app connected, pad switched on | the pad never connects |
| app quit, pad switched on | connects immediately |
| pad connected first, then the app | **both work** |

So it is not the bond, and it is not the pad. Whichever link is established first is kept, and the
second one can be added to it — but a *new* central connection cannot be started while `btd` holds
a peripheral one.

An explicit connect from the board says so rather than merely timing out:

```
Failed to connect: org.bluez.Error.Failed le-connection-abort-by-local
```

That is the local host giving up, not the pad refusing. BlueZ's background auto-connect for a
trusted device is the thing normally doing this, and it fails the same way — which is why the pad
simply sits there.

**`btd` is not advertising at the time.** It stops while a central is connected, which is why
`duckctl scan` cannot see a robot somebody's phone is talking to. So this is not fault 2 in another
costume: advertising is off and initiating still fails, which points at the controller refusing to
be an initiator while it is a peripheral rather than at anything either daemon chose.

### What is not worth building

The obvious fix — `configd` or `padd` polling for a trusted-but-disconnected pad and connecting it
— **does not work** for an LE pad, because that is exactly the call that returns the error above.
A classic pad is a different case — the link is BR/EDR and the robot pages it — and `configd` does
sweep for those; `configd/src/bluez.rs` has why.

The two that would work are both bad trades on this radio. `btd` could drop its link whenever a pad
wants one, which makes the phone the thing that disconnects at random and needs `btd` to know a pad
is asking, which it has no way to learn. Or pairing and reconnection could be moved behind a
sequence the app drives across a deliberate disconnect, which is real work against a chip the rest
of this page is about replacing.

### What to do instead

**Turn the pad on before opening the app.** Both then work, and the order is the whole of the
workaround. `pad.status` reports `paired` and `connected` separately, so a client can see the state
this produces — bonded, trusted, not connected — and say so rather than leaving somebody pressing
the Xbox button at a robot that is ignoring it.

Like the other two, this goes when the aic8800 does.
