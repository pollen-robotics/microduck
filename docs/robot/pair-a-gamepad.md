# Pair a gamepad

Once per pad. After this, `padd.service` drives whatever pad is connected, from boot — nothing to
start, and nothing that dies with your ssh session.

## Put the pad in pairing mode

On an **Xbox** controller this is two presses, and the second is the one that goes wrong:

1. Switch it on with a **short** press of the Xbox button. Do not hold that button — held, it
   switches the controller off.
2. Press the small **Sync** button on the top edge, next to the USB-C port, until the Xbox light
   **flashes quickly**. Slow blinking means it is on but not pairing.

On a **DualSense**: hold Create and PS together until the light bar flashes.

On a **Pro Controller** — the no-name Switch-style pads, which bluetoothctl lists as
`Pro Controller` — hold the small **Sync** button on the top edge, next to the USB-C port, until the
player lights sweep back and forth. It is a classic Bluetooth (BR/EDR) pad, the Xbox one is LE, and
the robot pairs the two in opposite orders; `pad pair` picks the right one by itself, and
[pairing one by hand](#pairing-a-pro-controller-by-hand) says what the order is if you have to.

## Pair it

```bash
sudo robotctl pad pair
```

```
looking for a gamepad in pairing mode — on an Xbox pad, press the small Sync button on the
top edge (not the Xbox button, which switches it off)
paired  Xbox Wireless Controller 78:86:2E:BB:13:28
padd is driving from it now.
```

No MAC address needed: the robot looks for a gamepad in pairing mode and takes the one it finds. The
pad is *trusted* as well as paired, which is what makes it reconnect by itself after a reboot with
nobody logged in.

If two are in pairing mode it refuses rather than guessing and prints their addresses. Naming one is
also how to pair hardware the robot does not recognise as a gamepad:

```bash
sudo robotctl pad pair 78:86:2E:BB:13:28
```

**A second pad needs no forgetting.** A pad already bonded is in range and in every sweep, so the
robot prefers one in pairing mode; both stay paired afterwards and `padd` drives whichever connects.
The cost is that re-running with nothing new in pairing mode waits out the whole search window
before reporting the pad you already have — `--timeout 5` if you are only repairing trust.

## Check it

```bash
robotctl pad status
```

```
pad     Xbox Wireless Controller 78:86:2E:BB:13:28  connected
padd    active — driving whatever pad connects
```

Two lines, because they fail separately: a connected pad with a dead driver looks exactly like a
working robot ignoring you.

A Pro Controller also carries a six-axis IMU, which the kernel exposes as a second input device
beside the one that drives. `robotctl monitor`, with `p` for the pad block, shows it as a panel
that appears only when the pad has one: a wireframe pad posed like the real one, pitch and roll
from gravity, yaw from the gyro alone (it drifts — nothing on a pad observes heading), and the
gyro's rest bias, which the clone needs learned before the picture stops turning on its own. Set
the pad down for half a second and the panel says `settled`. The stream costs nothing while the
monitor is not open: `padd` reads the IMU node only while somebody is subscribed to the tap — or
while it is steering the head from it, below.

The same attitude can drive the robot's head. `sudo robotctl configure`, section *Controller-IMU
head control*, `enabled`: Y then hands the head to the pad's tilt while the sticks keep driving,
Y again holds the head, and a third Y re-centres on the pad's current attitude and follows again —
the [cheat sheet](cheatsheet.md#gamepad-configd) has the full cycle. The picture in the monitor and
the head use one filter, so where the drawn pad points is where the head goes.

`paired but NOT trusted` is the state worth knowing. It works now and does not reconnect after a
reboot, because approving a reconnection needs an agent and at boot there is none. Re-run `pad pair`
to fix it.

## Forget one

```bash
sudo robotctl pad forget 78:86:2E:BB:13:28
```

This removes **the robot's half** of the bond, which is all a robot can remove. The pad keeps its own
half, so pairing it again needs it back in pairing mode — otherwise it arrives with a key this robot
no longer has and the bond is refused.

An Xbox pad holds **one** host bond, and a half-completed attempt leaves it holding a key this board
no longer has — which fails in exactly the way a broken board does. If pairing keeps failing, pair
the pad to a laptop once and remove it there; that consumes and releases its bond slot, and putting
it in pairing mode alone does not reliably do so.

## When a pad will not bond at all

On the aic8800 radio a pad cannot form a **new** bond while `btd` is advertising. Re-provision such a
board with `--pause-btd-on-pair`:

```bash
./scripts/provision-board.sh --pause-btd-on-pair pierre@192.168.1.42
```

That leaves a marker at `/var/lib/robot/weird-ble` and changes nothing else. On a board with that
marker, `sudo robotctl pad pair` handles the rest by itself — it stops `btd`, power-cycles the
adapter, pairs, and starts `btd` again. It says so as it goes. An existing bond is unaffected by any
of this, so a paired pad connects and drives with everything running.

Some units additionally cannot bond under BlueZ's default `Privacy = off` at all, even with `btd`
paused. Those want `--weird-ble`, which implies the pause and also sets `Privacy = device`.

**Try the pause first.** `Privacy = device` on a board that only needed the pause produces a worse
failure than no flag at all: the pad bonds and then flaps with
`Encryption Change: PIN or Key Missing (0x06)`, never creating an input device. If you see that, drop
`--weird-ble` and keep the pause —
[`install-dev.md`](install-dev.md#the-three-configurations-and-how-to-tell-which-you-have) has the
table and the commands to move a board between the two.

Pairing by hand on such a board needs the same two steps:

```bash
sudo systemctl stop btd
```

```bash
sudo bluetoothctl power off && sudo bluetoothctl power on
```

Pair, then:

```bash
sudo systemctl start btd
```

The power cycle is not optional. Stopping `btd` leaves its advertisement and the IO capability its
pairing agent gave the controller behind, and a pad still refuses to bond — a reboot has the same
effect, which is how this was found. Never `systemctl restart bluetooth` instead: on this board that
leaves no adapter at all until a reboot.

`--weird-ble` is the default in [`install-dev.md`](install-dev.md), because about half these boards
need it and nothing measurable says which. But a board that does not need it should not carry it —
the flag costs a `btd` stop and an adapter power cycle on every pairing — so that page also says how
to check and how to drop it.

Both are workarounds for the aic8800 radio, not properties of the design. They go when the radio
does.

### A classic pad and bluetoothd's CPU

A Pro Controller streams IMU samples in every packet, about 200 packets a second, whether or not
anyone touches it. With BlueZ's default `UserspaceHID=true` each one is relayed by bluetoothd through
uhid, and that cost 16% of a core on graphite with the pad idle. `scripts/setup-board.sh` sets
`UserspaceHID=false` in `/etc/bluetooth/input.conf` on every board, which hands the channel to the
kernel's `hidp` and takes bluetoothd out of the data path — measured 0.0% afterwards, same driver,
same input nodes, same bond. It applies at the next boot. An LE pad such as the Xbox is untouched by
the setting: HID over GATT never went through `input.conf`. A board provisioned before this exists
gets it by re-running the script:

```bash
sudo sh scripts/setup-board.sh && sudo reboot
```

## Pairing a Pro Controller by hand

Only if `pad pair` is not available. The order matters and is the **reverse** of the Xbox one:

```bash
bluetoothctl pair 98:B6:E9:28:06:09
```

```bash
bluetoothctl connect 98:B6:E9:28:06:09
```

```bash
bluetoothctl trust 98:B6:E9:28:06:09
```

`connect` first — which bonds as a side effect — ends in the state that is hardest to read: the pad's
light goes solid, `pad status` says `connected`, `bluetoothctl info` says `Paired: yes` and
`Connected: yes`, and **no input device exists**, so `padd` sits at "waiting for padd to open a pad"
and nothing drives. Recover with `sudo robotctl pad forget <address>`, put the pad back into pairing
mode and pair again. Running `pad pair` never produces this state: it knows the pad is classic from
the class BlueZ reports and pairs before it connects.

## When pairing fails every time

Check `/etc/bluetooth/main.conf` for the `Privacy` setting. It should read `Privacy = device`.
A board that carries `Privacy = off` may refuse to bond a pad at all — the connect gives up with
`le-connection-abort-by-local` and the pad never reaches `Paired: yes`.

If instead a capture shows the pairing reaching `DHKey check failed (0x0b)`, that is the opposite
fault and `Privacy = off` is the value to try on that board. Take a `btmon` capture before changing
the value, because both have been seen and they need different answers.

```bash
sudo sh scripts/setup-board.sh
```

```bash
sudo reboot
```

`setup-board.sh` corrects the value, and it does not take effect until the reboot.

Otherwise the usual cause is the pad having left pairing mode before the exchange began: press Sync
again and re-run while the light is still flashing quickly. To see the exchange itself:

```bash
sudo btmon -t > /tmp/btmon.log 2>&1 &
```

Pair, then `sudo pkill btmon` and look for `SMP: Pairing Failed` and the reason beside it. That is
the one instrument that distinguishes a board setting from a pad that is not listening.

## When it drops while you are driving

Watch the pad's own input stream, which is already on the board:

```bash
robotctl monitor
```

Press `p`. The block that opens is the raw evdev stream from the pad `padd` is driving from — every
report, timestamped by the kernel:

```
┌ pad Xbox Wireless Controller · /dev/input/event5 · 78:86:2e:bb:13:28 ─────────────┐
│ cadence  124/s while driving · last 8 ms ago · worst 84 ms · over 100 ms 0 · …    │
│ gap ms                                       ▁▁▁▁▂▁▁▁▁▃▁▁▁▁▁▁▂▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁ │
│ X     ····│██··   -8734 Y     ····│····       0 Z     ·········       0 …         │
│ held     BTN_START                                                                │
└ 4213 reports · gap ≤100 ms ───────────────────────────────── reports intact ──────┘
```

Move the sticks and the bars follow them. What the trace is for is the row above: a bar per report,
so a stall is a spike, and one that has already recovered is still on screen. Full height is 100 ms
— the point a driver starts to feel it — and past 500 ms `robotd` has zeroed the velocity.

**Nothing else on the robot can show this.** `padd` resends the last stick value at 50 Hz, so a
radio that has stopped delivering still looks like a live driver everywhere downstream: `robot.state`
carries fresh intents, the deadman never fires, and the robot keeps walking on a command nobody is
giving. The `asked` column in the block above it will look perfect while this one flatlines.

A pad at rest sends nothing, so silence is only evidence while you are driving. The block says
`the sticks are still` past five seconds rather than accusing the link, and counts those spells
separately.

Then, for a verdict over a window rather than a live picture, copy the measurement onto the board
from a clone of this repo:

```bash
scp scripts/pad-link-test.sh radxa@<board>:/tmp/
```

What has already happened, out of `padd`'s journal — no pad needed, and it answers immediately:

```bash
sudo sh /tmp/pad-link-test.sh --history
```

To measure the link now, with the pad on and `padd` running. **Keep the sticks moving for the whole
two minutes**: a pad at rest sends nothing, and silence reads exactly like a stalled link.

```bash
sudo sh /tmp/pad-link-test.sh
```

It counts drops, and the gaps between the pad's input reports while it is connected. A gap past
500 ms is the robot stopping — `robotd` zeroes the velocity there. Every drop is followed by the
kernel's reason: `0x08` is a supervision timeout, which means range or interference, and `0x13`
means somebody switched the pad off.

Putting the pad down is not a stall, and is not counted as one — but it is time the measurement
learns nothing from, so the report says how much of the window you actually drove, and declines to
judge a link it barely saw.

Walking away from the robot while it watches is how you find the range.

## Is this board running the same stack as that one

A pad that stalls on one robot and not on its twin is usually not the pad. Two boards built weeks
apart run different kernels, different BlueZ, different controller firmware, and pads on different
pad firmware — and none of that is visible in `pad status`.

Copy the report onto each board, from a clone of this repo:

```bash
scp scripts/pad-stack-report.sh radxa@<board>:/tmp/
```

```bash
sudo sh /tmp/pad-stack-report.sh
```

It prints the whole stack and saves the same text to `/tmp/pad-stack-<host>-<when>.log`: kernel,
BlueZ, the adapter's HCI version, which radio it is and the firmware the kernel loaded for it at
boot, what is carrying HID, which keys the bond holds, the transport in use right now, and the pad's
own firmware revision. It runs without root, and says `unreadable` for the three things that need
it.

To compare two boards, ask each for only the values that have to match:

```bash
ssh radxa@<board-a> sudo sh /tmp/pad-stack-report.sh --fingerprint > /tmp/a.fp
```

```bash
ssh radxa@<board-b> sudo sh /tmp/pad-stack-report.sh --fingerprint > /tmp/b.fp
```

```bash
diff /tmp/a.fp /tmp/b.fp
```

No output means the same stack. The fingerprint carries no timestamps and no addresses, so anything
`diff` prints is a real difference.

Two lines to read before the rest. `transport` is `LE` on every pad tried so far, and a board that
says `BR/EDR` is putting the pad through the kernel's classic HID path instead of BlueZ's — a
different driver, different button numbering. `input` is the `Bus`/`Vendor`/`Product`/`Version`
quadruple that SDL and `gilrs` hash into a mapping GUID: two boards differing there have different
axis and button mappings, whatever else matches.

---

Driving — the controls and the speed limits — is in the
[cheat sheet](cheatsheet.md#gamepad-configd); running `padd` from a laptop over a forwarded socket
is in the [dev cheat sheet](cheatsheet-dev.md#from-a-laptop--drive-with-a-pad-in-your-hands).
