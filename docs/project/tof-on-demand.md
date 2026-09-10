# The head sensors when nobody is looking

`tofd` ranged a laser and read an IMU from boot to shutdown on every duck with the head module
fitted, whether or not one process was subscribed to either stream. This started as "start the
unit when it is needed and stop it after" and ended somewhere much smaller, by measuring at each
step: **the depth stream is not what costs anything, the sensor that does has no consumer at all,
and nothing inside its loop was worth optimising.** So the laser and the unit were left alone and
the head IMU got a switch, off by default.

It follows [`idle-cpu.md`](idle-cpu.md), which took `tofd`'s poll from ~100 I²C reads a second to
~45 and left the question of *why it polls at all* open.

## What it costs, measured

On olducky (Radxa Zero 3, RK3566), an idle board with nothing subscribed to either stream:

```text
$ ps -L -o tid,comm,pcpu -p "$(pidof tofd)"
    TID COMMAND         %CPU
  12209 tofd             0.0
  12212 tof-sensor       0.5
  12213 head-imu         4.5
```

Percentages are of one core, so the ~5% `top` shows for `tofd` is 1.25% of this SoC — and it is
**nine parts head IMU to one part depth**. The socket-serving runtime, which has nothing to serve,
costs nothing.

That split is the whole of this document. Two things follow from it.

**The IMU number is the two reads a sample takes.** Not the wakeups, and not the fusion. The
crate's `bench_imu` runs the loop five ways, 10 s each at 100 Hz, on this board with `tofd`
stopped:

```text
  sleep only  (0 reads)   cpu  0.69 %
  filter only (0 reads)   cpu  1.00 %
  update      (2 reads)   cpu  3.35 %
  update_all  (2 reads)   cpu  3.86 %
  three-reads (3 reads)   cpu  4.20 %
```

Being woken a hundred times a second is 0.69 points; the Madgwick update adds 0.31; the two I²C
transactions are the remaining ~2.4–2.9. A gyro sample and an accelerometer sample *are* twelve
bytes at two addresses, so there is no version of a full sample that costs fewer than two
transactions — which is why nothing in this loop is worth optimising.

Two caveats on those numbers, since they are the basis for a decision. `update` and `update_all`
do identical work (one delegates to the other) and came out 0.51 points apart, so the noise floor
is around half a point — CPU frequency scaling on an A55, most likely. And an earlier run of the
same three read modes, with `tofd` still holding the bus, compressed them to 4.44/4.46/4.53 and
made the third read look free. Neither changes the shape: the reads dominate, the floor is small,
the third read is worth somewhere between 0.3 and 0.9 points.

**Nothing subscribes to it.** `head_imu.stream` has no consumer in this tree: `btd` declines to
proxy it (`btd/src/route.rs:412`), the updater's degraded IPC declines it
(`updater/src/ipc.rs:819`), and no daemon subscribes. It was added for the mapping work, which has
not arrived. So the largest recurring cost in this daemon is a sensor read for nobody.

Worth saying plainly what this is *not*: the walk policy's IMU is a different chip on a different
bus — the `imu_to_dxl` v2 board on the Dynamixel bus, read in the same 50 Hz `sync_read` as the
fifteen servos (`duck-control/src/bus.rs`, `duck-control/src/model.rs:76`). Nothing here touches
it, and nothing here can cost the policy an IMU sample.

## What was decided

**`[head_imu] enabled`, default off.** Nothing in the loop paid off, so the switch is the answer:
a stream nothing subscribes to should not cost ~4% of a core from boot. It lives in
`robotd.toml`, where `mediad` already reads `[media]`, so `robotctl configure` writes it and
offers the `tofd` restart; `tofd --imu` reads the chip for one session without touching the file,
and a subscriber while it is off gets a reason naming the key rather than the silence an unfitted
sensor gives. Turn it on when the mapping work reads the stream.

**The redundant third read went anyway**, as bmi088-rs#1 and a `tofd` that takes the sample from
one call. It is worth 0.3–0.9 points, which is inside the noise of the table above, so it landed
for the other reasons: the published `accel` becomes the sample the quaternion was computed from
rather than one read ~200 µs later, and a read can fail in one place instead of two.

**The rate is the remaining lever, and it is linear** — `--imu-hz 50` halves the read cost. Left
as a flag rather than a config key: the first real consumer decides what rate it needs, and
guessing now would be a number nobody chose.

**If a consumer ever needs 100 Hz cheaply, the answer is the FIFO.** Both chips have one, so ten
samples could arrive in one transaction instead of twenty — the only change that attacks the term
that actually dominates. It costs frame parsing, watermark configuration and up to 100 ms of
latency, which is why it is not being built for a consumer that does not exist.

## The gating that was designed instead, and is no longer needed

Kept because the reasoning is what led to the switch, and because step 1 is a bug regardless.

### 1. Make "somebody wants this" true — a bug either way

`accept` subscribes a connection to **both** channels before it has read a byte of the request
(`tof/src/main.rs:612`), so `receiver_count()` counts connections, not interest: a client that
asked for `head_imu.stream` holds a depth receiver, and one that connects and says nothing holds
both.

Move the `subscribe()` calls inside the matched arms of `subscriber()`, so a receiver exists only
where the method is known, and hand the function the two `Sender`s instead of two `Receiver`s. This
is worth doing on its own — a connection that has asked for nothing should not read as wanting
depth — and it is the prerequisite for anything below.

### 2. The IMU thread could open on the first subscriber and close after the last

There is nothing expensive to preserve: `open_imu` is a handful of register writes, no firmware and
no probe. So the thread could wait on an IMU lease, open, read at `imu_hz` while somebody is
listening, and close when they go — the same 4% saved, without an operator having to know about a
switch.

**Not built, because the switch got there first and is a tenth of the machinery.** This becomes
the better answer the moment there *is* a consumer: an opted-in duck would otherwise pay the 4%
whenever nothing happens to be reading. Two details would decide whether it is done right:

Two details decide whether it is done right:

- **The subscribe answer must not lie.** `ImuStatus` starts at `unavailable: "no reading yet"`,
  and `head_imu.stream`'s answer is written before the first sample. Cold, that answer would read
  as "no BMI088 fitted", which is the one thing it must not say on a board that has one. So the
  handler would signal the thread and wait for the open to resolve — bounded, a couple of hundred
  milliseconds — before answering `found` or `lost`. (The switch has the same problem and solves
  it the same way: `ImuStatus::off` is its own sentence, naming the key.)
- **A cold `quat` is a converging `quat`.** The Madgwick fusion settles over about a second
  (`tof/src/imu.rs:43`), so a subscriber gets orientation that is still moving where today it gets
  one converged since boot. `gyro` and `accel` are raw and unaffected. Document it on
  `method::HEAD_IMU_STREAM`; the first real consumer can say whether it needs a warm filter, which
  is easy to add then and pointless to guess at now.

### 3. Do not gate the ranging, and do not touch the unit

Both were the plan before the measurement, and both are now closed:

- **Ranging on demand buys 0.5% of one core.** That is what the `tof-sensor` thread costs to hold a
  laser open, poll it and publish fifteen frames a second, after the poll work in `idle-cpu.md`.
  Stopping and starting ranging is cheap in itself — `start(hz)`/`stop()` are one transaction each
  (`tof/src/sensor.rs:293`, `:181`) — but it means a state machine in the one process that owns a
  shared I²C bus, a resume path that has to be right, and a hardware assumption nobody has tested
  (that stop-then-start does not re-upload firmware). Half a percent does not buy that.
- **`robotd`'s theremin can keep subscribing at startup.** Its depth reader connects once and holds
  (`robotd/src/theremin.rs:237`), which under a ranging gate would have pinned the laser on for
  exactly the ducks that play notes. With no ranging gate there is nothing to pin, and the reason it
  connects early — so the first arming window waits only for frames — stands unchallenged.

The one argument left for gating the laser is the VCSEL's own power, which is not CPU and does not
show up in `ps`. If somebody wants that closed, the number is in the VL53L8CX datasheet; at
milliamps it stays closed and this bullet can go.

## Why not start and stop the unit

Kept because it is the answer to the question that started this, and because the reason is a durable
fact about the daemon rather than a measurement.

**The bring-up is seconds, and it is per process.** `Sensor::open` probes the device ID and then
uploads the ULD firmware — ~90 KB over I²C, "a few seconds at 400 kHz" (`tof/src/sensor.rs:226`) —
once per process, before ranging. A `monitor` that started the unit on `t` would show an empty grid
for seconds, and `robotctl theremin` could not begin arming until the upload finished.

**Nothing owns the "off".** Two clients can want depth at once, so stopping on exit needs a
reference count that survives a client being `SIGKILL`ed, and systemd has none for manually started
units. A `monitor` killed with Ctrl-\\ would leave the sensor ranging forever.

**It needs privilege no client has.** `monitor` runs as an operator in the `robot` group;
`systemctl start` wants root or a polkit rule granting `manage-units` on that unit to that group — a
new install-path artifact that a board provisioned before it would silently not have.

**One unit, two sensors.** Stopping `tofd` stops the head IMU too, which is the sensor this document
ends up caring about.

Socket activation would fix the privilege and the leak for free — a `tofd.socket` unit owning
`/run/tofd/tof.sock`, where connecting *is* the start signal and systemd owns the lifecycle. It does
not fix the firmware, which it moves onto the first connection of every session and repeats on each
idle exit. It is the right mechanism for the day the *process* is what we want gone; it is not the
answer to a poll, and after the measurement there is no poll worth answering.

## What is left for a board

- `ps -L` on a duck running the switch, showing no `head-imu` thread at all.
- The bench again with the CPU governor pinned and the modes interleaved, if anybody wants the
  third read's worth to better than half a point. Nothing now rests on it.
- SoC temperature at idle over ten minutes, before and after. ~4% of a core will not be visible in
  it, and that is the honest expectation to write down rather than discover: this was about not
  reading a sensor for nobody, not about heat.
