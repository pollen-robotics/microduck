# What the robot does when nobody is asking it to

A duck sitting on a desk with no pad connected, no viewer, and nothing driving it still runs
five daemons. This is what they were each doing in that state, what was changed, and — for the
two candidates that were not — the numbers that say why.

Everything here is arithmetic on the code or a measurement on this machine. **Nothing in it has
been measured on a board**, and the last section says what that would take.

## What was costing something

| | idle before | idle after |
|---|---|---|
| `mediad` raw branch | 1.84 MB copied 30×/s | copied when a reader asks — ~2×/s |
| `tofd` frame poll | ~100 I²C reads/s | ~45 |
| `padd`, pad connected, sticks centred | 50–100 msg/s | 10 |
| `pet-detect`, when enabled | ~400 FFTs/s + 4 forward passes/s | none until the room makes a sound |

### `mediad` copied every frame

The appsink callback copied every buffer off the tee into a slot readers took the latest of.
At 720p30 — the shipped quality, and `media.camera` defaults on — a UYVY frame is 1 280 × 720 × 2
= 1.84 MB. The readers are auto-exposure at 2 Hz and the duck detector at 2 Hz when enabled, so
twenty-eight of every thirty copies were made for nobody: 55 MB/s of memcpy and a 1.8 MB
allocation thirty times a second, from boot.

A reader now asks and the callback answers the *next* frame. Answering with the last one instead
would be cheaper again and would put the reader's own polling period into the measurement — an
exposure loop steering on half-second-old luma hunts rather than settles.

### `tofd` polled a 15 Hz sensor 100 times a second

`data_ready` every 10 ms across the whole 66 ms between frames: about seven I²C transactions to
find one frame, six of them answered no. The loop now sits out the stretch in which the sensor
cannot yet have one and polls through the rest. Frame age is unchanged — still bounded by the
10 ms poll, which is the granularity a frame is noticed at either way.

This runs on every duck with a ToF fitted whether or not anyone uses the theremin, because
`robotd`'s depth reader subscribes at startup rather than when the instrument is picked up.

### `padd` re-sent the same three zeros fifty times a second

The sticks are polled, so an untouched pad produced an identical frame every tick — an encode, a
write, a flush, and a parse on `robotd`'s side, doubled in the two modes that also send a zeroing
`robot.move`. Identical frames are now held back, and the two intents share one write.

A frame is only held back when it asks for **no motion**. That is what keeps this away from
`[safety] deadman_ms`, a value `padd` cannot read: the deadman zeroes the twist and nothing else,
so on a frame already commanding zero, letting it fire changes nothing — and on one commanding
motion it would stop a robot whose stick is still held.

### The petting classifier ran through silence

A 1 s window of 40 log-mel bands is a hundred 512-point FFTs plus a forward pass, four times a
second. The ambient-sound watcher beside it was already measuring the room, so the classifier now
waits until something stands out from that floor — except while petting, where the End edge has
to be found by inference exactly as the Start was.

Off by default, so this is a cost anyone who turns it on stops paying rather than one every robot
was paying.

## What was left alone, and why

### The 50 Hz bus write is not worth making conditional

`safety.apply` writes sixteen goal positions every tick, including the ticks where the robot is
parked and the targets have not moved. Making that conditional saves:

- **Bus:** a Protocol 2.0 sync-write of sixteen servos is 94 bytes — 4 header, 1 broadcast id,
  2 length, 1 instruction, 4 address/length, 16 × 5 payload, 2 CRC. At 1 Mbaud, 940 µs. The tick's
  sync-read of seventeen devices is about 422 bytes, or 4.2 ms, so the write is a fifth of a bus
  already at roughly a quarter utilisation on a 20 ms tick. It is not the constraint.
- **CPU:** one `write(2)` of 94 bytes and the tty layer behind it. Call it 15 µs including the
  completion interrupt; at 50 Hz that is **0.075% of one core**.

Against that: the write is the loop's one statement guaranteeing the servos have been told where
to be. Making it conditional means enumerating every path that can invalidate a servo's stored
goal — a torque toggle, a brown-out, a servo rebooting itself after clearing a shutdown latch,
`init`'s own `interpolate_to` — and being wrong means a robot that stops holding its pose in a
case nobody thought of. Seven hundredths of a percent of a core does not buy that.

The rest of an idle `robotd` tick is already lean: the ONNX step is gated on `driving`, the state
frame is only assembled when something has subscribed, the voltage and thermal registers are read
once a second rather than per tick, and the serial read sleeps rather than spins.

### Multi-threaded tokio runtimes cost nothing to leave alone

`robotd`, `configd` and `updaterd` use bare `#[tokio::main]`, which starts one worker per core —
four each on an RK3566. Idle workers park; they do not poll. Measured with a runtime of four
workers and four parked tasks, against a `current_thread` runtime doing the same:

```text
multi:   5 threads   0.01 s CPU at start → 0.01 s after 20 s
current: 1 thread    0.00 s CPU at start → 0.00 s after 20 s
```

The difference is four parked threads' worth of stacks — memory, not heat. Switching flavours
would buy none of the CPU this page is about, and would cost something real: on
`current_thread`, a synchronous call on the async path blocks the whole runtime including the IPC
socket, and `updaterd` has several (`engine.rs` reads a signature, writes the embedded manifest,
and scans a unit directory inline; only the heavy verification is on `spawn_blocking`).

## What still needs a board

Every figure above is arithmetic on frame sizes, packet layouts and poll intervals, or a
measurement on a developer's machine. The four changes want confirming where the heat actually
is:

- `mediad`, `tofd` and `padd` CPU before and after, from `dev-push.sh` and `top -H`. The camera
  one should be the visible change.
- SoC temperature at idle over ten minutes, which is the number the whole exercise is for. The
  `videoflip` episode took this board to 97 °C and throttled it to 408 MHz, so idle headroom is
  what decides whether a duck walks well while it is also looking at something.
- `tofd`'s guard against a real sensor clock. The poll is anchored on each frame's arrival and
  tolerates the sensor running 20 ms early — a 30% period error — but no VL53L8 has been watched
  doing it.
- That petting still starts as promptly as it did, on a robot in an ordinary room rather than a
  silent one.
