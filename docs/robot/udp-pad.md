# Drive with a pad over UDP

A program on the same network sends the state of a gamepad — sticks, triggers, buttons — as UDP
datagrams, and the robot does with it exactly what it does with a Bluetooth pad: the same mapping,
the same D-pad modes, the same holds on Start and Select. It is for a pad that is not near the
robot, or not Bluetooth at all — a laptop with a pad in its USB port, a phone, a script. `netpadd`
is the daemon that listens; `padd` is the one that reads a pad over Bluetooth, and exactly one of
them runs at a time (see [what the robot does](#what-the-robot-does)).

**It is a quick, unauthenticated LAN tool, made for one case: a pad driving a robot on a network you
trust, with as little latency as possible.** Anyone on that network who can reach the port can
drive the robot, stand it up, sit it down and power it off — see [trust](#trust). It is not the
way to control a robot in general: that is WebRTC, which is authenticated and encrypted, carries a
return path, and works from outside the LAN —
[`../design/remote-webrtc.md`](../design/remote-webrtc.md).

Pairing a Bluetooth pad is a different thing, and it is
[`pair-a-gamepad.md`](pair-a-gamepad.md).

## Turning it on

It is off by default. Turn it on only when you know what it opens, on a network you trust.

Once on, **it stays on** — across reboots and updates — until you switch it off again: it is a
configuration key, not a session. While it is on, the port is open and the Bluetooth pad does not
work.

Switch it on in the configuration:

```bash
sudo robotctl configure
```

set `netpad.enabled = true`, then restart both daemons:

```bash
sudo systemctl restart padd netpadd
```

Both, because each decides at start whether it is the one to run: `padd` stands down when
`[netpad] enabled` is true, and `netpadd` stands down when it is not. Restarting only one leaves the
other on the old answer, and for a moment two sources — or none.

To see the selected source, both service states and Bluetooth connection evidence, use
`robotctl pad status`. To check only which services are running:

```bash
systemctl is-active padd netpadd
```

```
inactive
active
```

`padd` inactive and `netpadd` active is the UDP pad. The reverse is the Bluetooth pad. Both active
means one of them was started on an older answer: both units select from the same key, so after a
hand edit that restarted only one, the other runs on until it is restarted too — restart both, or
change the key with `robotctl configure`, which does. Neither active means the one that should be
running has failed — `journalctl -u netpadd -b` has why.

To go back to Bluetooth, set `netpad.enabled = false` (or reset the key) and restart the same two
units. A configuration that cannot be read counts as off, so a damaged file never leaves somebody
without a pad.

The other four keys are all read at start, so they too take a restart of both units:

| key | default | what it is |
|---|---|---|
| `enabled` | `false` | UDP is the pad source, and `padd` stands down |
| `port` | `4210` | the UDP port, bound on IPv4 `0.0.0.0` |
| `max_hz` | `30` | the most intent frames a second sent to the robot, 10 to 100; outside that is refused |
| `stale_ms` | `300` | nothing new for this long and the robot stops where it stands, the client kept; also how late a datagram may arrive and still drive. It must be below `[safety] deadman_ms`, or the configuration is refused and the error names both keys |
| `timeout_ms` | `1000` | the silence after which the client counts as gone; at least `stale_ms`, or the configuration is refused |

## Driving from a laptop

```bash
duckctl udp-pad
```

Without an address it finds the robot over Bluetooth the way `duckctl ip` does, so `--name` and
`DUCK_ROBOT` pick which one. An address skips that and works without Bluetooth:

```bash
duckctl udp-pad 192.168.1.42
```

Press any button on the pad you want to drive with: that pad is the one it sends, until it is
unplugged. The press itself is not sent as a press. A port on the end of the
address (`duck.local:4210`) overrides the default, and `--hz` (10 to 100, default 30) is the most it
will send a second. It sends the moment anything changes and otherwise a keepalive every `1/hz`, so
the robot can tell a still pad from a gone one. With no pad on the laptop it sends nothing, and the
robot times it out as it would a Bluetooth pad that went away. `Ctrl-C` ends it: the robot stops
after `stale_ms`, and sits down a second after `timeout_ms`.

If pushing the stick forward walks the robot backwards, add `--invert-y`: it flips both sticks' Y
before sending.

### The laptop quacks, the duck opens its mouth

RT's quack and LT's wheee play from the laptop's speakers, through whatever output the laptop has
selected, and the duck only opens its mouth for them. `--robot-sound-on` has the duck quack and
wheee as well. It is bit 0 of the datagram's `flags` ([below](#writing-your-own-sender)); a
`netpadd` from before that bit ignores it, and the duck quacks either way until it is updated. The
duck's other sounds — asking where the pad went, and the greeting when it is back — are its own
and always play. The laptop's follow the duck's rules: a quack cuts the one still
playing, a quack during a ride is dropped, and the wheee loops while LT is held and stops when it
is let go. The laptop plays them for the pad it sends, whether or not the duck is taking it.

The laptop has a voice of its own, not the duck's: a duck's voice comes from its chip's serial,
which the laptop cannot read. The first run picks a random voice and keeps its number in
`duckctl/voice-seed` under your configuration directory (`~/.config` on Linux and macOS,
`%APPDATA%` on Windows), so it is the same voice every time. The log says which:

```
quacking here too, in voice 813162435 — --voice N for another, --mute for none
```

`--voice 42` quacks in voice 42 for this run, and `--mute` keeps the laptop quiet — with
`--robot-sound-on` for the duck's sounds, or no sound at all. Delete the file for a new voice. With no audio output, or on Linux without `libasound.so.2`, the log says
`no sound on this machine` and the pad drives as before.

### Several ducks from one laptop

Plug in one pad per duck. Open one terminal per duck, and start one `udp-pad` in each, one at a time.

In the first terminal:

```bash
duckctl udp-pad 192.168.1.42
```

Press a button on the pad for that duck. Its block now shows the pad's name. In the second terminal:

```bash
duckctl udp-pad 192.168.1.43
```

Press a button on the second pad. And so on for each duck.

A pad that already drives a duck is skipped: pressing it again says which duck it drives. If two
`udp-pad`s wait at once, the second one waits until the first has its pad.

`--pad 2` sends the second pad on the laptop without waiting for a press, for a script. The number
is the order the laptop found the pads in, which can change when a pad is unplugged and plugged back.

On a terminal it takes the screen, as `robotctl monitor` does, and `q` quits:

```
┌ duck 192.168.1.42:4210 ─────────────────────────────────────────────────────┐
│ ● CONNECTED  driving the duck                                               │
│ rtt 3.1 ms · loss 0.0% · 0 stale                                            │
│ rtt ms    ▁▁▁▁▂▁▁▁▁▁▁▁▁▁▃▁▁▁▁▁▁▁▁▁▁▁▁                                       │
└ rtt ≤50 ms ─────────────────────────────────────────────────────────────────┘
┌ robot healthy ──────────────────────────────────────────────────────────────┐
│ power    batt 7.62 V 61% · cpu 71 °C · loop 50.0/50 Hz                      │
│ motors   hottest left_knee 58 °C · mean 44 °C · bus ok                      │
│ pose     walk · upright · imu ok                                            │
│ health   healthy                                                            │
│ refused  robot.do — not home yet · 3 s ago                                  │
└─────────────────────────────────────────────────────────────────────────────┘
┌ pad Xbox 360 Controller · battery 70% ──────────────────────────────────────┐
│ left   X  ·······│·······  +0.00   Y  ·······│████···  +0.50                │
│ right  X  ·······│·······  +0.00   Y  ·······│·······  +0.00                │
│ trig   LT ···············  +0.00   RT ███████████████  +1.00                │
│  A   B   X   Y   LB   RB   START   SELECT   UP   DOWN   LEFT   RIGHT   HOME │
│ last input 12 ms ago · values as sent, before the duck's deadzone           │
└ sending 30 Hz of 30 · 0 failed ─────────────────────────────────────────────┘
┌ log ────────────────────────────────────────────────────────────────────────┐
│ gamepad: Xbox 360 Controller — sending its state                            │
│ ● CONNECTED — driving the duck                                              │
└ q quit ─────────────────────────────────────────────────────────────────────┘
```

The duck block is green when connected and red when it is not, or when another client is driving
— its first line says which, and why. Held buttons are lit. The figures come
from the robot's [answers](#what-the-robot-answers):

| the duck block says | meaning |
|---|---|
| `● CONNECTED` | the robot takes this laptop's datagrams. `rtt` is the round trip, `loss` the share of the last second's datagrams the robot did not count, and `stale` the ones it dropped as out of order or late |
| `✕ REFUSED` | another client is driving the robot, and only one may at a time. This laptop is ignored until that one has been quiet for `timeout_ms`, then takes over |
| `✕ NOT CONNECTED` — nothing listening | the robot's address refuses the port: `netpadd` is not running there, or `[netpad] enabled` is off |
| `✕ NOT CONNECTED` — stopped answering | there were answers and they stopped. The Wi-Fi, the robot rebooting, or `netpadd` restarting |
| `✕ NOT CONNECTED` — nothing answers | sends go out and nothing has ever come back: a wrong address, a firewall, or a `netpadd` older than its answers — which still drives |

The robot block is the robot's own `robot.health` and posture, the same facts as `robotctl health`
and the monitor's power row, and coloured by the same rules. Red: unhealthy, a battery at 15% or
less, motors not answering, an IMU missing or frozen, a fall. Yellow: degraded, a battery at 30% or
less, motor bus read failures, a throttled CPU, limp. A servo's temperature is shown but never
coloured, because nothing on the robot defines one as too hot. `refused` is the last thing you
asked for that the robot would not do, and why — the answer to "I pressed it and nothing
happened". Each alarm, when raised and when cleared, and each refusal also goes to the log. Only
the client driving the duck is sent any of this.

Piped to a file, it writes the same facts as one line every 10 seconds instead.

## Writing your own sender

One datagram carries the whole state. Every field is little-endian, and the packet is 28 bytes:

| offset | size | field | |
|---|---|---|---|
| 0 | 4 | magic | `b"DKPD"` |
| 4 | 1 | version | `1` |
| 5 | 1 | flags | bit 0: the sender plays the triggers' sounds itself — the robot opens its mouth for RT and LT but plays no quack and no wheee. Bits 1 to 7 reserved: send 0, the receiver ignores them |
| 6 | 4 | seq | u32, +1 per datagram, wraps |
| 10 | 4 | t_ms | u32, the sender's own clock in ms, starting anywhere; how late a datagram arrived is read from it. `0` means none, and nothing is checked |
| 14 | 12 | axes | i16 ×6: lx ly rx ry (±32767 maps to ±1, up is positive), lt rt (0..32767 maps to 0..1) |
| 26 | 2 | buttons | u16: bit 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Start, 7 Select, 8 Up, 9 Down, 10 Left, 11 Right, 12 Home |

Send raw stick values. The robot applies the deadzone, as it does for a Bluetooth pad, so a sender
that applies its own would do it twice. Bits 13 to 15 of `buttons` are reserved: the receiver
ignores them and logs it once. A version-1 packet *longer* than 28 bytes is accepted and the tail is
ignored, so a field can be added later without a version bump.

A sender in Python. It sends the whole state thirty times a second for as long as it is driving —
a sender that pauses for longer than `timeout_ms` is gone, and the next datagram is a fresh
connection whose buttons count as already held, so a press sent on its own after a pause does
nothing. It opens with a neutral pad, taps Start to stand the robot up, keeps sending a still pad
while it stands, taps Start again to turn the policy on, walks forward for three seconds, then
sends one stale datagram the robot drops, and falls silent:

```python
import random, socket, struct, time
DUCK, START = ("duck.local", 4210), 1 << 6
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
seq = random.getrandbits(32)                  # a restarted sender is not mistaken for a stale one
def pkt(seq, ly=0, buttons=0):
    t_ms = int(time.monotonic() * 1000) % 2**32
    return b"DKPD" + struct.pack("<BBII6hH", 1, 0, seq, t_ms, 0, ly, 0, 0, 0, 0, buttons)
def hold(frames, ly=0, buttons=0):            # the same state, every 1/30 s
    global seq
    for _ in range(frames):
        seq = (seq + 1) % 2**32
        s.sendto(pkt(seq, ly, buttons), DUCK); time.sleep(1 / 30)
hold(10)                                      # a still pad first
hold(5, buttons=START); hold(60)              # tap Start: robot.init, and 2 s to stand
hold(5, buttons=START); hold(10)              # tap Start again: the policy on
hold(90, ly=32767)                            # 3 s forward
s.sendto(pkt((seq - 50) % 2**32, ly=-32767), DUCK)  # stale: dropped, never walks backwards
# then silence: the robot stops after stale_ms, and sits down a second after timeout_ms
```

The rules a sender follows, and why:

- **Send the whole state every time.** Nothing is retransmitted and nothing is queued, so a lost
  datagram must cost nothing — which it does only if the next one says everything.
- **Start `seq` at a random value and add 1 per datagram.** The robot drops a datagram that is not
  newer than the last, which is how a reordered one never moves the robot backwards, and one more
  than 1000 ahead of it, so a sender must never skip ahead. A random start
  means a restarted sender is not mistaken for a stale one.
- **Stamp `t_ms` from a monotonic clock in milliseconds.** The robot never syncs with it: it
  compares each datagram's delivery against the fastest it has seen from the same stream, so any
  starting point works, but a clock that jumps reads as late for a second before it is taken.
- **Send on change, plus a keepalive every `1/hz`.** The change is what makes it feel immediate; the
  keepalive is what tells the robot you are still there.
- **Mark the socket DSCP EF** (IP TOS `0xB8`). An access point honouring WMM then queues the
  datagrams ahead of bulk traffic; one that does not loses nothing. `duckctl udp-pad` does, and the
  robot marks its answers the same way.
- **Stop sending to let go.** There is no goodbye packet. Silence is the signal, and it is the same
  one a pad that lost its battery gives.

## What the robot answers

`netpadd` answers every datagram from the client that holds the robot, stale ones included, with
24 bytes sent back to the address the datagram came from. A client that does not hold it is
answered at most once a second, and a malformed datagram not at all, so an answer is never larger
than what it answers and never more frequent. A sender is free to ignore the answers: nothing on
the robot waits for them to be read.

| offset | size | field | |
|---|---|---|---|
| 0 | 4 | magic | `b"DKST"` |
| 4 | 1 | version | `1` |
| 5 | 1 | verdict | `1` driving: this client holds the robot. `2` busy: another client does |
| 6 | 4 | seq | u32, the `seq` of the datagram answered |
| 10 | 4 | accepted | u32, datagrams taken from the client holding the robot |
| 14 | 4 | stale | u32, datagrams from that client dropped as not newer, more than 1000 ahead, or late |
| 18 | 4 | malformed | u32, datagrams on the port that were not pad state, from anyone |
| 22 | 2 | reserved | `0` |

The three counts run from `netpadd`'s start and wrap, so read their change, not their value. The
time from sending a `seq` to its answer is the round trip. Datagrams sent over a second, compared
with how much `accepted` grew over the same second, give the loss. As with the pad datagram, an
answer longer than 24 bytes is a version-1 answer with fields this reader does not know.

The answer is sent without waiting: when the socket's buffer is full, or the client cannot be
reached, it is dropped, and the receive loop never slows for it.

### The robot's health

Every second, and at once when a client takes the robot, `netpadd` sends the driving client a second
kind of datagram: `b"DKRB"`, the version byte `1`, then a JSON object. Every field may be absent,
and absent means *not known*, never zero:

| field | what |
|---|---|
| `health` | `robot.health` as `robotd` answered it: `healthy`, `degraded`, `reason`, `battery`, `motors`, `cpu_temp_c`, `cpu_throttle`, `control_loop`, `bus`, `imu` |
| `health_error` | why there is no `health` — `robotd` not answering |
| `posture` | `policy`, `fallen`, `limp`, `picked_up`, from `robot.state` |
| `posture_error` | why there is no `posture` |
| `refused` | the last pad request the robot did not carry out in the past 30 s: `method`, `reason`, `ago_ms` |

```json
{"health":{"healthy":true,"battery":{"volts":7.62,"percent":61.0},"cpu_temp_c":71.2,"bus":{"consecutive_errors":0,"startup_failures":0}},
 "posture":{"policy":"walk","fallen":false,"limp":false,"picked_up":false}}
```

`netpadd` gathers it on connections to `robotd` of its own, so a slow answer never holds up the
pad, and a `robotd` that cannot be reached is a `health_error`, never a stopped `netpadd`. A field
added to `robot.health` reaches this report without a change to the format.

## What the robot does

- **Only the newest state counts.** Nothing is buffered, so a late datagram is dropped and an
  in-between state is merged into the next. `max_hz` caps how often intents go to the robot, and a
  change that arrives inside the interval waits for the next slot rather than being dropped. A
  press and release that both land between two slots still fire, because buttons are compared per
  datagram, not per slot.
- **A datagram delivered late is dropped.** Wi-Fi can hold datagrams for seconds and release them
  at once, in order, so each is still the newest. One that took more than `stale_ms` longer than
  the fastest delivery seen from the same stream is dropped and counted as stale, and buttons
  pressed in it fire nothing: it says what the pad did, not what it does. The count is logged at
  most once every 10 seconds. A stream is the same sender continuing its `seq`, across a silence
  too, so a backlog that arrives after the client counted as gone is still dropped, and so is a
  duplicate or reordered one in it, at or a little behind the last `seq`. A restarted sender, at a
  random `seq`, is a new one, judged by its own clock. Late by the same amount for a whole second
  is the sender's clock having moved, not a backlog — a backlog drains in bursts, each far less
  late than the one before — and the stream is taken again.
- **How late datagrams arrive is logged.** Every 10 seconds while a client drives, `netpadd` logs
  a `link:` line: the p50, p99 and largest delay above the stream's fastest delivery, in ms, and how
  many were dropped as late. A few ms is a healthy link. Hundreds with nothing dropped is a lag
  under `stale_ms`; `late` above 0 is a backlog or a lag beyond it. `stalls` is how many times
  the robot was stopped for nothing new in `stale_ms`.
- **The first sender holds it.** The robot locks to the address of the first datagram it accepts and
  drops datagrams from any other address until that one has been silent for `timeout_ms`, so two
  people driving honestly cannot fight over one duck. It is not protection: see [trust](#trust).
- **A short silence stops the robot.** After `stale_ms` with nothing accepted, `netpadd` stops the
  walking at once, leaves the head and the body pose where they are, and sends nothing more until
  the client is heard again — not the last stick again, which would keep `robotd`'s deadman fed on
  what the pad did. The client keeps the robot: the lock, the holds in flight and its buttons all
  carry on when it comes back, so a press in its next datagram fires. Bad Wi-Fi costs a stop, not
  the session.
- **A long silence is the client gone.** After `timeout_ms` with nothing accepted, in-flight holds
  are reset and the lock is released. Then it does what `padd` does when a Bluetooth pad drops
  out: a second later the robot asks where the pad went (a sound) and sits down if it is standing,
  so it is not left frozen in the middle of a room. A robot that refuses — mid-kick, mid-rise — is
  asked again for a few seconds, then left as it is. While it waits, `robotd` has it look around
  on its own. The client coming back is greeted with another sound — only if the robot had asked:
  a blip shorter than that second, which bad Wi-Fi produces many times a minute, makes no sound. The journal says so once, at
  `warn`, in each direction: `pad client connected` and `pad client gone`.
- **The first datagram after the client was gone, or after `netpadd` starts, fires nothing.** Its buttons
  are taken as already held, so a client that connects with Start down does not stand the robot up or sit it down — not when it
  comes back, not while it stays down, not when it is let go. Any sequence number is accepted after
  a silence, so a restarted sender is picked up within `timeout_ms`: until the old one counts as
  gone, the new sender's random `seq` may read as stale and be dropped.
- **Malformed datagrams are dropped.** Wrong magic, fewer than 28 bytes, or an unknown version: the
  datagram is dropped and counted, and the count is logged at most once every 10 seconds, so
  something noisy on the port cannot flood the journal.

Exactly one of `padd` and `netpadd` runs because each asks the same configuration key in its unit's
`ExecCondition=` before starting. Why that and not `Conflicts=` is in
`netpadd/systemd/netpadd.service`, and how it interacts with an update is
[`restart-order.md`](../design/restart-order.md).

## Trust

There is none. Anyone on the LAN who can reach UDP port 4210 can drive the robot once the current
client has been quiet for `timeout_ms` — including holding Select to power it off. While a client
is talking, its address is the only thing that keeps others out, and a source address on a LAN can
be forged: a datagram carrying the driver's address and a `seq` just ahead of theirs is taken as
theirs. The 1000-ahead bound stops a single blind forgery from locking the driver out, not someone
who can see the traffic. Use it on a network you
would let into the room with the robot, and for anything beyond that use
[WebRTC](../design/remote-webrtc.md).
