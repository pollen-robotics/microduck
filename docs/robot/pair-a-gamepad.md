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

## When pairing fails every time

Check `/etc/bluetooth/main.conf` for `Privacy = device`. Boards provisioned before this was
understood have it, and with it **a pad cannot bond at all**: it rejects the pairing with `DHKey
check failed (0x0b)`, because that check is computed over both devices' addresses and privacy pairs
from a resolvable private one. `Privacy = off` is what works.

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

---

Driving — the controls, the speed limits, and running `padd` from a laptop over a forwarded socket —
is in the [README](../../README.md#drive-it).
