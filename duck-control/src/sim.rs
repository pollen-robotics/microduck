//! A [`RobotIo`] whose robot is in MuJoCo.
//!
//! **The third backend the design doc named.** `docs/design/robotd-design.md` §9 deferred "the
//! MuJoCo backend and the `RemoteIo` protocol"; this is it. Everything above this trait — the
//! control loop, the policy, `Safety`, fall detection, odometry, kinematics, every IPC call and
//! `robotctl` — runs unchanged and cannot tell the difference. That is the whole point: the seam
//! is the one place a simulator is allowed to exist.
//!
//! ## Why TCP and not a unix socket
//!
//! Two reasons, both learned rather than assumed. A unix path is capped at `SUN_LEN` — about 108
//! bytes — which a scratch directory blows through immediately. And the simulator has to be
//! reachable from *outside* whatever the daemons run in: a container on Linux, and on macOS a Linux
//! VM with MuJoCo on the host beside it. A port crosses all of those; a socket path does not.
//!
//! ## Why JSON
//!
//! One tick is fifteen joints in and fifteen out — about a kilobyte, so 50 KB/s at the loop's 50 Hz,
//! which is nothing next to being able to read a frame with `nc` and write the other half of it in
//! twenty lines of Python. The alternative is a packed struct shared between two repositories in
//! two languages, which is exactly the shape of thing this project has already lost days to when an
//! offset was wrong and the failure was silent.
//!
//! ## What it does when the simulator goes away
//!
//! It goes away *often*: MuJoCo compiles its model, so changing the number of ducks means restarting
//! it, and the ducks are expected to survive that. So a dead connection is an error returned to the
//! caller and a reconnect on the next call — no backoff thread, because **the control loop is the
//! retry timer**. `robotd` already treats a failed `read` as a tick to skip rather than a reason to
//! exit; this is the same tolerance it has for a missing `tofd`.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::imu::ImuData;
use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::NUM_JOINTS;

/// Bumped when the wire format changes in a way an older peer would misread.
///
/// Checked in the handshake and reported by *both* numbers when it fails, because the two halves
/// live in two repositories and "the simulator is too old" and "the daemon is too old" are the same
/// symptom otherwise.
pub const PROTOCOL: u32 = 1;

/// How long to wait for the simulator to answer one request.
///
/// Comfortably longer than a tick, because a simulator that has just been asked to step a
/// contact-heavy scene can be late without being broken — and comfortably shorter than forever,
/// because a wedged simulator must not wedge the control loop with it.
const TIMEOUT: Duration = Duration::from_millis(200);

/// One request. `op` is the tag, so a frame is readable in a log without a decoder.
#[derive(Debug, Serialize)]
#[serde(tag = "op")]
enum Request<'a> {
    #[serde(rename = "hello")]
    Hello { protocol: u32, joints: usize },
    #[serde(rename = "read")]
    Read,
    #[serde(rename = "write")]
    Write { targets: &'a [f64; NUM_JOINTS] },
    #[serde(rename = "gain")]
    Gain { kp: u16 },
    #[serde(rename = "torque")]
    Torque { on: bool },
    #[serde(rename = "slow")]
    Slow,
}

#[derive(Debug, Deserialize)]
struct Hello {
    protocol: u32,
}

/// What the simulator reports, in exactly the units [`Sensors`] wants — radians, rad/s, mA, and the
/// IMU already resolved into the trunk frame.
///
/// **The simulator does the conversion, not this.** MuJoCo knows its own model's joint order,
/// scaling and frame; a translation layer here would be a second place for that knowledge to live
/// and drift. What crosses the wire is the robot's own units.
#[derive(Debug, Deserialize)]
struct SensorFrame {
    positions: [f64; NUM_JOINTS],
    velocities: [f64; NUM_JOINTS],
    #[serde(default)]
    currents_ma: [f64; NUM_JOINTS],
    imu: ImuFrame,
}

#[derive(Debug, Deserialize)]
struct ImuFrame {
    gyro: [f64; 3],
    gravity: [f64; 3],
    quat: [f64; 4],
}

#[derive(Debug, Deserialize)]
struct SlowFrame {
    volts: f64,
    temps_c: [f64; NUM_JOINTS],
}

/// An acknowledgement, so a write that the simulator refused is not silently a write that worked.
#[derive(Debug, Deserialize)]
struct Ack {
    #[serde(default)]
    error: Option<String>,
}

/// A robot in MuJoCo, reached over TCP.
pub struct RemoteIo {
    addr: String,
    link: Option<Link>,
    /// Reported once rather than every tick a disconnected loop runs.
    complained: bool,
}

struct Link {
    write: TcpStream,
    read: BufReader<TcpStream>,
}

impl RemoteIo {
    /// Name the simulator. Nothing is connected until the first call — a daemon must start whether
    /// or not the simulator is up yet, exactly as it starts without a robot on the bus.
    pub fn at(addr: impl Into<String>) -> Self {
        Self {
            addr: addr.into(),
            link: None,
            complained: false,
        }
    }

    fn connect(&mut self) -> Result<&mut Link> {
        if self.link.is_none() {
            let address = self
                .addr
                .to_socket_addrs()
                .and_then(|mut a| {
                    a.next()
                        .ok_or_else(|| std::io::Error::other("resolved to no address"))
                })
                .map_err(|source| IoError::Port {
                    path: self.addr.clone(),
                    source,
                })?;

            // A connect timeout as well as a read one: a port nobody listens on refuses instantly,
            // but a host that silently drops packets would otherwise hang the loop.
            let stream =
                TcpStream::connect_timeout(&address, TIMEOUT).map_err(|source| IoError::Port {
                    path: self.addr.clone(),
                    source,
                })?;

            // **Nagle would be catastrophic here and silent.** It delays a small write waiting for
            // more to send, up to ~40 ms — twice the tick — turning every transaction into a
            // missed deadline that looks like a slow simulator.
            let _ = stream.set_nodelay(true);
            let _ = stream.set_read_timeout(Some(TIMEOUT));
            let _ = stream.set_write_timeout(Some(TIMEOUT));

            let read = BufReader::new(stream.try_clone().map_err(|source| IoError::Port {
                path: self.addr.clone(),
                source,
            })?);
            let mut link = Link {
                write: stream,
                read,
            };

            let hello: Hello = exchange(
                &mut link,
                &Request::Hello {
                    protocol: PROTOCOL,
                    joints: NUM_JOINTS,
                },
            )?;
            if hello.protocol != PROTOCOL {
                return Err(IoError::Bus(format!(
                    "the simulator speaks protocol {} and this daemon speaks {PROTOCOL} — one of \
                     the two is out of date, and they are in different repositories",
                    hello.protocol
                )));
            }

            tracing::info!(addr = %self.addr, protocol = PROTOCOL, "simulated body");
            self.complained = false;
            self.link = Some(link);
        }
        Ok(self.link.as_mut().expect("just connected"))
    }

    /// One request and its answer, dropping the connection if anything about it fails.
    ///
    /// Dropping on *any* error, not only on a closed socket: a frame that will not parse means the
    /// two ends disagree about where a message begins, and there is no way to resynchronise a
    /// line protocol except by starting again.
    fn call<T: for<'de> Deserialize<'de>>(&mut self, request: &Request<'_>) -> Result<T> {
        let result = self.connect().and_then(|link| exchange(link, request));
        if let Err(error) = &result {
            self.link = None;
            if !self.complained {
                self.complained = true;
                tracing::warn!(
                    addr = %self.addr, %error,
                    "no simulated body; retrying every tick until it answers"
                );
            }
        }
        result
    }
}

fn exchange<T: for<'de> Deserialize<'de>>(link: &mut Link, request: &Request<'_>) -> Result<T> {
    let mut line = serde_json::to_string(request).map_err(|e| IoError::Bus(e.to_string()))?;
    line.push('\n');
    link.write
        .write_all(line.as_bytes())
        .map_err(|e| IoError::Bus(format!("sending {}: {e}", tag(request))))?;

    let mut answer = String::new();
    let read = link
        .read
        .read_line(&mut answer)
        .map_err(|e| IoError::Bus(format!("waiting for {}: {e}", tag(request))))?;
    if read == 0 {
        return Err(IoError::Bus(format!(
            "the simulator closed the connection during {}",
            tag(request)
        )));
    }
    serde_json::from_str(&answer).map_err(|e| {
        IoError::Bus(format!(
            "{} answered with something this cannot read: {e}",
            tag(request)
        ))
    })
}

fn tag(request: &Request<'_>) -> &'static str {
    match request {
        Request::Hello { .. } => "hello",
        Request::Read => "read",
        Request::Write { .. } => "write",
        Request::Gain { .. } => "gain",
        Request::Torque { .. } => "torque",
        Request::Slow => "slow",
    }
}

fn acked(ack: Ack, what: &str) -> Result<()> {
    match ack.error {
        None => Ok(()),
        Some(why) => Err(IoError::Bus(format!("the simulator refused {what}: {why}"))),
    }
}

impl RobotIo for RemoteIo {
    fn read(&mut self) -> Result<Sensors> {
        let frame: SensorFrame = self.call(&Request::Read)?;
        Ok(Sensors {
            positions: frame.positions,
            velocities: frame.velocities,
            currents_ma: frame.currents_ma,
            imu: ImuData {
                gyro: frame.imu.gyro,
                gravity: frame.imu.gravity,
                quat: frame.imu.quat,
            },
        })
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let ack: Ack = self.call(&Request::Write {
            targets: &targets.positions,
        })?;
        acked(ack, "the joint targets")
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        let ack: Ack = self.call(&Request::Gain { kp })?;
        acked(ack, "the position gain")
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        let ack: Ack = self.call(&Request::Torque { on })?;
        acked(ack, if on { "torque on" } else { "torque off" })
    }

    /// A simulated servo has no latched hardware error to clear and no firmware to restart, so
    /// there is nothing to send: the reboot succeeds at once and the caller restores the gains as
    /// it would on a robot. Deliberately not an op on the wire — the simulator would only have to
    /// answer it with an ack.
    fn reboot(&mut self, id: u8) -> Result<()> {
        tracing::debug!(id, "reboot of a simulated servo: nothing to do");
        Ok(())
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let frame: SlowFrame = self.call(&Request::Slow)?;
        Ok(SlowSensors {
            volts: frame.volts,
            temps_c: frame.temps_c,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::thread;

    /// A simulator made of canned answers: one script per connection, one answer per request.
    ///
    /// A real socket rather than a trait behind the socket, because the things worth testing here
    /// *are* the socket — a peer that hangs up, a frame that will not parse, a second connection
    /// after the first died.
    fn simulator(scripts: Vec<Vec<&'static str>>) -> (String, thread::JoinHandle<Vec<String>>) {
        let listener = TcpListener::bind("127.0.0.1:0").expect("a loopback port");
        let addr = listener.local_addr().expect("its address").to_string();
        let handle = thread::spawn(move || {
            let mut heard = Vec::new();
            for script in scripts {
                let (stream, _) = listener.accept().expect("a connection");
                let mut out = stream.try_clone().expect("a writer");
                let mut lines = BufReader::new(stream);
                for reply in script {
                    let mut line = String::new();
                    if lines.read_line(&mut line).expect("a request") == 0 {
                        break;
                    }
                    heard.push(line.trim().to_string());
                    out.write_all(reply.as_bytes()).expect("a reply");
                    out.write_all(b"\n").expect("a newline");
                }
            }
            heard
        });
        (addr, handle)
    }

    const HELLO: &str = r#"{"protocol":1}"#;
    // One line, because the protocol is one frame per line and a test fixture that wraps would be
    // testing the fixture.
    const SENSORS: &str = concat!(
        r#"{"positions":[0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"#,
        r#""velocities":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.5],"#,
        r#""currents_ma":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"#,
        r#""imu":{"gyro":[0,0,0],"gravity":[0,0,-1],"quat":[1,0,0,0]}}"#
    );
    const OK: &str = r#"{}"#;

    #[test]
    fn a_read_carries_the_sensors_the_loop_expects() {
        let (addr, sim) = simulator(vec![vec![HELLO, SENSORS]]);
        let mut io = RemoteIo::at(addr);
        let sensors = io.read().expect("a sensor frame");
        assert_eq!(sensors.positions[0], 0.1);
        assert_eq!(sensors.velocities[NUM_JOINTS - 1], 0.5);
        assert_eq!(sensors.imu.gravity, [0.0, 0.0, -1.0]);

        let heard = sim.join().expect("the simulator thread");
        assert!(heard[0].contains(r#""op":"hello""#), "{heard:?}");
        assert!(heard[1].contains(r#""op":"read""#), "{heard:?}");
    }

    #[test]
    fn the_targets_cross_in_joint_order() {
        let (addr, sim) = simulator(vec![vec![HELLO, OK]]);
        let mut io = RemoteIo::at(addr);
        let mut positions = [0.0; NUM_JOINTS];
        positions[2] = -0.4579;
        io.write(&JointTargets::new(positions)).expect("a write");

        let heard = sim.join().expect("the simulator thread");
        // Positional, exactly as the wire everywhere else in this project is: the third number is
        // the third joint, and nothing names it.
        assert!(heard[1].contains("[0.0,0.0,-0.4579,"), "{heard:?}");
    }

    #[test]
    fn a_protocol_mismatch_names_both_versions() {
        let (addr, _sim) = simulator(vec![vec![r#"{"protocol":99}"#]]);
        let mut io = RemoteIo::at(addr);
        let error = io
            .read()
            .expect_err("a mismatch is fatal to the connection");
        let said = error.to_string();
        assert!(said.contains("99") && said.contains("1"), "{said}");
        assert!(said.contains("different repositories"), "{said}");
    }

    #[test]
    fn a_refusal_is_an_error_rather_than_a_silent_success() {
        let (addr, _sim) = simulator(vec![vec![HELLO, r#"{"error":"torque is disabled"}"#]]);
        let mut io = RemoteIo::at(addr);
        let error = io
            .set_torque(true)
            .expect_err("a refusal must not read as done");
        assert!(error.to_string().contains("torque is disabled"), "{error}");
    }

    #[test]
    fn a_simulator_that_restarts_is_reconnected_to_on_the_next_tick() {
        // The case this exists for: MuJoCo compiles its model, so changing the number of ducks
        // restarts it — and the ducks are expected to survive that. The first connection dies
        // mid-read; the second is expected to be made without anyone asking.
        let (addr, sim) = simulator(vec![vec![HELLO], vec![HELLO, SENSORS]]);
        let mut io = RemoteIo::at(addr);

        io.read().expect_err("the simulator hung up");
        let sensors = io.read().expect("reconnected on the next call");
        assert_eq!(sensors.positions[0], 0.1);

        let heard = sim.join().expect("the simulator thread");
        let hellos = heard.iter().filter(|l| l.contains("hello")).count();
        assert_eq!(
            hellos, 2,
            "the second connection must handshake again: {heard:?}"
        );
    }

    #[test]
    fn a_frame_that_will_not_parse_drops_the_connection() {
        // Not merely an error: a half-read line means the two ends disagree about where a message
        // starts, and a line protocol cannot resynchronise except by starting again.
        let (addr, sim) = simulator(vec![vec![HELLO, "not json"], vec![HELLO, SENSORS]]);
        let mut io = RemoteIo::at(addr);
        io.read().expect_err("garbage is an error");
        io.read().expect("a fresh connection, not a wedged one");

        let heard = sim.join().expect("the simulator thread");
        assert_eq!(heard.iter().filter(|l| l.contains("hello")).count(), 2);
    }
}
