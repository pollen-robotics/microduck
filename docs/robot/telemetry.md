# Telemetry

A microduck sends anonymous usage data to help us see how ducks are used and where they break. It
is **on by default** and you can turn it off at any time (see below). Same design as Reachy Mini's
daemon.

## What is sent

Every event carries:

- a random robot id, created when telemetry is turned on and deleted when it is turned off
- the board (`zero3`, `beta`, or `unknown`), the CPU architecture
- the daemon version, and the OS image: `ID VERSION_ID` and its `BUILD_ID` from `/etc/os-release`

| Event | When | Extra data |
|---|---|---|
| `daemon_started` | `updaterd` starts: each boot and each daemon update | — |
| `update_result` | the first start on a new daemon version | previous and new version |
| `daily_heartbeat` | once a day | `updaterd`'s uptime |
| `policies` | once a run, when `robotd` first answers | the mode, and `policy_<slot>` for every filled slot |
| `policy_loaded` | a slot's policy changed (checked every minute) | slot, mode, origin, policy |

A policy is named `org/name@revision/file` when it came from the Hugging Face Hub — the official
set or `policy.fetch`, both of which download without a token, so the repo was public when the
robot got it. A file put on the robot by hand is only ever `local`: its path is not sent.

## What is never sent

IP addresses, location, Wi-Fi names, the robot's name, the Hugging Face account or token, the SoC
serial or any hardware id, logs, camera images or audio.

## Where it goes

[PostHog](https://posthog.com) EU Cloud (servers in the European Union), with IP-based location
disabled and no person profiles. Events carry no timestamp of their own: the board's RTC has no
battery and resets on a hard power-off, so until NTP syncs the clock is wrong, and PostHog stamps
them on arrival. Events wait in memory, at most 500, while the robot is offline or not yet synced —
TLS cannot validate a certificate before then either.

## Turning it off

The **share anonymous usage data** switch in the console's drawer, or from anything that speaks the
API — the phone over BLE, `duckctl`, the console over WebRTC:

```json
{"jsonrpc":"2.0","id":1,"method":"telemetry.set","params":{"enabled":false}}
{"jsonrpc":"2.0","id":2,"method":"telemetry.status"}
```

It applies at once and survives reboots and updates. Off drops what is queued and deletes the
robot id; turning it back on starts a new id that nothing links to the old one.

`telemetry.status` answers `enabled` (the stored choice) and `active` (whether anything is actually
sent). `MICRODUCK_TELEMETRY=0` or `DO_NOT_TRACK=1` in `updaterd`'s environment forces it off
whatever the choice; `scripts/duck-sim` and `scripts/systemd-test.sh` set the first, so a simulated
or containerised duck sends nothing.

The choice lives in `/var/lib/robot/updater/telemetry.json`. A file that cannot be read means off.

The implementation is [`updater/src/telemetry.rs`](../../updater/src/telemetry.rs).
