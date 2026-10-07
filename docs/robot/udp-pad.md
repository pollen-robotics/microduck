# Drive with a pad over UDP

A program on the same network sends the state of a gamepad — sticks, triggers, buttons — as UDP
datagrams, and the robot does with it exactly what it does with a Bluetooth pad: the same mapping,
the same D-pad modes, the same holds on Start and Select. It is for a pad that is not near the
robot, or not Bluetooth at all — a laptop with a pad in its USB port, a phone, a script. `netpadd`
is the daemon that listens; `padd` is the one that reads a pad over Bluetooth, and exactly one of
them runs at a time (see [what the robot does](#what-the-robot-does)).

**This is a LAN tool.** There is no authentication, and nothing about the datagrams is meant to cross
the internet. For a client outside the LAN the path is WebRTC, which is encrypted, carries a return
path, and works from a data centre — [`../design/remote-webrtc.md`](../design/remote-webrtc.md).

Pairing a Bluetooth pad is a different thing, and it is
[`pair-a-gamepad.md`](pair-a-gamepad.md).

## Turning it on

It is off, because a robot that has never heard of it should keep driving from the pad it already
has. Switch it on in the configuration:

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

To check which one has the robot:

```bash
systemctl is-active padd netpadd
```

```
inactive
active
```

`padd` inactive and `netpadd` active is the UDP pad. The reverse is the Bluetooth pad. Both active
cannot happen, and neither active means the one that should be running has failed — `journalctl -u
netpadd -b` has why.

To go back to Bluetooth, set `netpad.enabled = false` (or reset the key) and restart the same two
units. A configuration that cannot be read counts as off, so a damaged file never leaves somebody
without a pad.

The other three keys are all read at start, so they too take a restart of both units:

| key | default | what it is |
|---|---|---|
| `enabled` | `false` | UDP is the pad source, and `padd` stands down |
| `port` | `4210` | the UDP port, bound on IPv4 `0.0.0.0` |
| `max_hz` | `30` | the most intent frames a second sent to the robot, 10 to 100; outside that is refused |
| `timeout_ms` | `250` | the silence after which the client counts as gone; it must be below `[safety] deadman_ms`, or the configuration is refused and the error names both keys |

## Driving from a laptop

```bash
duckctl udp-pad $(duckctl ip)
```

It reads the first gamepad plugged into the laptop and sends its state. A port on the end of the
address (`duck.local:4210`) overrides the default, and `--hz` (10 to 100, default 30) is the most it
will send a second. It sends the moment anything changes and otherwise a keepalive every `1/hz`, so
the robot can tell a still pad from a gone one. With no pad on the laptop it sends nothing, and the
robot times it out as it would a Bluetooth pad that went away. `Ctrl-C` ends it, and the robot
stops within `timeout_ms`.

## Writing your own sender

One datagram carries the whole state. Every field is little-endian, and the packet is 28 bytes:

| offset | size | field | |
|---|---|---|---|
| 0 | 4 | magic | `b"DKPD"` |
| 4 | 1 | version | `1` |
| 5 | 1 | flags | reserved: send 0, the receiver ignores it |
| 6 | 4 | seq | u32, +1 per datagram, wraps |
| 10 | 4 | t_ms | u32, the sender's own clock; logged for jitter, never compared with the robot's |
| 14 | 12 | axes | i16 ×6: lx ly rx ry (±32767 maps to ±1, up is positive), lt rt (0..32767 maps to 0..1) |
| 26 | 2 | buttons | u16: bit 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Start, 7 Select, 8 Up, 9 Down, 10 Left, 11 Right |

Send raw stick values. The robot applies the deadzone, as it does for a Bluetooth pad, so a sender
that applies its own would do it twice. Bits 12 to 15 of `buttons` are reserved: the receiver
ignores them and logs it once. A version-1 packet *longer* than 28 bytes is accepted and the tail is
ignored, so a field can be added later without a version bump.

A sender in a dozen lines of Python — Start twice to stand and then walk, then three seconds
forward, then silence:

```python
import socket, struct, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def pkt(seq, ly=0, buttons=0):
    return b"DKPD" + struct.pack("<BBII6hH", 1, 0, seq, 0, 0, ly, 0, 0, 0, 0, buttons)
seq = 1
for b in (64, 0, 64, 0):                  # Start, release, Start, release
    s.sendto(pkt(seq, 0, b), ("duck.local", 4210)); seq += 1; time.sleep(1.5)
for _ in range(90):                        # 3 s forward at 30 Hz
    s.sendto(pkt(seq, 32767), ("duck.local", 4210)); seq += 1; time.sleep(1/30)
# then silence: the robot stops
```

The rules a sender follows, and why:

- **Send the whole state every time.** Nothing is retransmitted and nothing is queued, so a lost
  datagram must cost nothing — which it does only if the next one says everything.
- **Start `seq` at a random value and add 1 per datagram.** The robot drops a datagram that is not
  newer than the last, which is how a reordered one never moves the robot backwards. A random start
  means a restarted sender is not mistaken for a stale one.
- **Send on change, plus a keepalive every `1/hz`.** The change is what makes it feel immediate; the
  keepalive is what tells the robot you are still there.
- **Stop sending to let go.** There is no goodbye packet. Silence is the signal, and it is the same
  one a pad that lost its battery gives.

## What the robot does

- **Only the newest state counts.** Nothing is buffered, so a late datagram is dropped and an
  in-between state is merged into the next. `max_hz` caps how often intents go to the robot, and a
  change that arrives inside the interval waits for the next slot rather than being dropped. A
  press and release that both land between two slots still fire, because buttons are compared per
  datagram, not per slot.
- **The first sender holds it.** The robot locks to the address of the first datagram it accepts and
  drops datagrams from any other address until that one has been silent for `timeout_ms`. Two
  people cannot fight over one duck.
- **Silence is the client gone.** After `timeout_ms` with nothing accepted, `netpadd` sends *nothing*,
  exactly as `padd` does with no pad, and `robotd`'s deadman (`[safety] deadman_ms`) holds the
  robot. In-flight holds are reset and the lock is released. The journal says so once, at `warn`,
  in each direction: `pad client connected` and `pad client gone`.
- **The first datagram after a silence fires nothing.** Its buttons are taken as already held, so a
  client that reconnects with Start down does not stand the robot up or sit it down. Any sequence
  number is accepted after a silence, so a restarted sender is picked up at once.
- **Malformed datagrams are dropped.** Wrong magic, fewer than 28 bytes, or an unknown version: the
  datagram is dropped and counted, and the count is logged at most once every 10 seconds, so
  something noisy on the port cannot flood the journal.

Exactly one of `padd` and `netpadd` runs because each asks the same configuration key in its unit's
`ExecCondition=` before starting. Why that and not `Conflicts=` is in
`netpadd/systemd/netpadd.service`, and how it interacts with an update is
[`restart-order.md`](../design/restart-order.md).

## Trust

There is none. Anyone on the LAN who can reach UDP port 4210 can drive the robot once the current
client has been quiet for `timeout_ms`, and while a client is talking the first sender's address is
the only thing that keeps others out — an address on a LAN is not a secret. Use it on a network you
would let into the room with the robot, and for anything beyond that use
[WebRTC](../design/remote-webrtc.md).
