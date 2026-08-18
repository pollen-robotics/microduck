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
board has been provisioned, with every daemon stopped. So it is something provisioning changes
about the system rather than a process that is running.

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
