//! Persistent raw-maintenance client; the owning process also sends the lease heartbeat.
use crate::{Failure, exit};
use clap::Subcommand;
use duck_ipc_proto::{self as proto, diagnostics as d};
use std::{
    collections::BTreeMap,
    io::{BufRead, BufReader, Write},
    os::unix::net::UnixStream,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};
#[derive(Debug, Subcommand)]
pub enum Command {
    Inspect,
    Capabilities,
    Status,
    Refresh,
    Cancel,
    /// Existing robot.subscribe stream, JSONL to stdout.
    Watch,
    /// Persistent stdin commands. EOF, Ctrl-C or process suspension ends the lease.
    Session {
        #[arg(long,value_parser=["maintenance-raw"])]
        mode: String,
        #[arg(long)]
        record: Option<PathBuf>,
    },
}
static INTERRUPTED: AtomicBool = AtomicBool::new(false);
extern "C" fn interrupted(_: libc::c_int) {
    INTERRUPTED.store(true, Ordering::Relaxed);
}
fn err(e: impl std::fmt::Display) -> Failure {
    Failure::new(exit::FAILED, e.to_string())
}
struct Client {
    stream: BufReader<UnixStream>,
    next: u64,
}
impl Client {
    fn open(path: &Path) -> Result<Self, Failure> {
        let stream = UnixStream::connect(path).map_err(err)?;
        stream
            .set_read_timeout(Some(Duration::from_secs(4)))
            .map_err(err)?;
        stream
            .set_write_timeout(Some(Duration::from_secs(2)))
            .map_err(err)?;
        Ok(Self {
            stream: BufReader::new(stream),
            next: 0,
        })
    }
    fn call(&mut self, c: d::Call) -> Result<serde_json::Value, Failure> {
        self.next += 1;
        let r = proto::Request::call(
            proto::Id::Number(self.next),
            &proto::Call::RobotDiagnostics(c),
        );
        let line = serde_json::to_string(&r).map_err(err)?;
        writeln!(self.stream.get_mut(), "{line}").map_err(err)?;
        let mut line = String::new();
        if self.stream.read_line(&mut line).map_err(err)? == 0 {
            return Err(err("robotd disconnected"));
        }
        let r: proto::Response = serde_json::from_str(&line).map_err(err)?;
        if let Some(e) = r.error {
            return Err(err(e.message));
        }
        Ok(r.result.unwrap_or(serde_json::Value::Null))
    }
}
fn output(v: &serde_json::Value) -> Result<(), Failure> {
    println!("{}", serde_json::to_string(v).map_err(err)?);
    Ok(())
}
fn accepted(v: &serde_json::Value) -> Result<(), Failure> {
    if v.get("accepted").and_then(|x| x.as_bool()) == Some(false) {
        Err(err(v
            .get("reason")
            .and_then(|x| x.as_str())
            .unwrap_or("request refused")))
    } else {
        Ok(())
    }
}
fn watch(path: &Path, mut writer: impl Write) -> Result<(), Failure> {
    let stream = UnixStream::connect(path).map_err(err)?;
    let mut r = BufReader::new(stream);
    let q = proto::Request::call(
        proto::Id::Number(1),
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(50) }),
    );
    writeln!(r.get_mut(), "{}", serde_json::to_string(&q).map_err(err)?).map_err(err)?;
    let mut line = String::new();
    loop {
        line.clear();
        if r.read_line(&mut line).map_err(err)? == 0 {
            break;
        }
        writer.write_all(line.as_bytes()).map_err(err)?;
    }
    Ok(())
}
/// Parse stdin syntax without moving anything; malformed input never sends a command.
fn parse(line: &str, sid: u64, rid: u64) -> Result<d::Call, Failure> {
    let p: Vec<_> = line.split_whitespace().collect();
    let session = d::Session {
        session_id: sid,
        request_id: rid,
    };
    match p.as_slice() {
        ["stop"] => Ok(d::Call::Stop(session)),
        ["release"] => Ok(d::Call::Release(session)),
        ["torque", joint, on] if matches!(*on, "on" | "off") => Ok(d::Call::Torque(d::Torque {
            session_id: sid,
            request_id: rid,
            joints: vec![(*joint).into()],
            on: *on == "on",
        })),
        ["gains", joint, p, i, z] => {
            let val = |s: &str, k: &str| {
                s.strip_prefix(k)
                    .ok_or_else(|| err("expected p= i= d="))
                    .and_then(|s| s.parse::<u16>().map_err(err))
            };
            Ok(d::Call::Gains(d::Gains {
                session_id: sid,
                request_id: rid,
                gains: BTreeMap::from([(
                    (*joint).into(),
                    d::Pid {
                        p: val(p, "p=")?,
                        i: val(i, "i=")?,
                        d: val(z, "d=")?,
                    },
                )]),
            }))
        }
        ["position", joint, q, valid] => {
            let q = q
                .strip_prefix("rad=")
                .ok_or_else(|| err("expected rad="))?
                .parse::<f64>()
                .map_err(err)?;
            if !q.is_finite() {
                return Err(err("non-finite position"));
            }
            let ms = valid
                .strip_prefix("valid_for_ms=")
                .ok_or_else(|| err("expected valid_for_ms="))?
                .parse()
                .map_err(err)?;
            Ok(d::Call::Command(d::Command {
                session_id: sid,
                request_id: rid,
                positions_rad: BTreeMap::from([((*joint).into(), q)]),
                valid_for_ms: ms,
            }))
        }
        _ => Err(err(
            "expected: torque JOINT on|off; gains JOINT p=N i=N d=N; position JOINT rad=X valid_for_ms=N; stop; release",
        )),
    }
}
pub fn run(path: &Path, command: Command) -> Result<(), Failure> {
    if matches!(command, Command::Watch) {
        return watch(path, std::io::stdout());
    }
    let mut client = Client::open(path)?;
    let record = match command {
        Command::Inspect => return output(&client.call(d::Call::Inspect)?),
        Command::Capabilities => return output(&client.call(d::Call::Capabilities)?),
        Command::Status => return output(&client.call(d::Call::Status)?),
        Command::Refresh => {
            let v = client.call(d::Call::Refresh(d::Cancel { request_id: 1 }))?;
            output(&v)?;
            return accepted(&v);
        }
        Command::Cancel => {
            let v = client.call(d::Call::Cancel(d::Cancel { request_id: 1 }))?;
            output(&v)?;
            return accepted(&v);
        }
        Command::Session { record, .. } => record,
        Command::Watch => unreachable!(),
    };
    // Create the evidence file before acquiring authority. Existing files are never overwritten.
    let evidence = record
        .map(|p| {
            std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(p)
        })
        .transpose()
        .map_err(err)?;
    eprintln!(
        "maintenance_raw: policy, pickup/fall reactions and anatomical clamps bypassed. Native servo restrictions remain. EOF/release closes the session."
    );
    let acquired = client.call(d::Call::Acquire(d::Acquire {
        request_id: 1,
        mode: "maintenance_raw".into(),
        acknowledge_raw: true,
    }))?;
    output(&acquired)?;
    accepted(&acquired)?;
    let sid = acquired
        .get("session_id")
        .and_then(|x| x.as_u64())
        .ok_or_else(|| err("missing session ID"))?;
    if let Some(file) = evidence {
        let path = path.to_owned();
        std::thread::spawn(move || {
            if let Err(e) = watch(&path, file) {
                eprintln!("recording ended: {}", e.message)
            }
        });
    }
    INTERRUPTED.store(false, Ordering::Relaxed);
    // Only an atomic store in the handler; the owner loop handles release outside signal context.
    unsafe {
        libc::signal(libc::SIGINT, interrupted as *const () as libc::sighandler_t);
    }
    let (tx, rx) = mpsc::sync_channel(1);
    std::thread::spawn(move || {
        for line in std::io::stdin().lock().lines() {
            if tx.send(line).is_err() {
                break;
            }
        }
    });
    let mut rid = 1;
    let mut sequence = 0;
    let mut last = Instant::now();

    // On I/O failure dropping the connection triggers server-side termination; no detached heartbeat.
    (|| {
        loop {
            if INTERRUPTED.load(Ordering::Relaxed) {
                break;
            }
            if last.elapsed() >= Duration::from_millis(100) {
                sequence += 1;
                let v = client.call(d::Call::Keepalive(d::Keepalive {
                    session_id: sid,
                    sequence,
                }))?;
                accepted(&v)?;
                last = Instant::now();
            }
            match rx.recv_timeout(Duration::from_millis(20)) {
                Ok(Ok(line)) => {
                    let line = line.trim();
                    if line.is_empty() || line.starts_with('#') {
                        continue;
                    }
                    rid += 1;
                    match parse(line, sid, rid) {
                        Ok(call) => {
                            let release = matches!(call, d::Call::Release(_));
                            let v = client.call(call)?;
                            output(&v)?;
                            if release {
                                return accepted(&v);
                            }
                        }
                        Err(e) => eprintln!("{}", e.message),
                    }
                }
                Ok(Err(e)) => return Err(err(e)),
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {}
            }
        }
        rid += 1;
        let v = client.call(d::Call::Release(d::Session {
            session_id: sid,
            request_id: rid,
        }))?;
        output(&v)?;
        accepted(&v)
    })()
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn all_joint_names_use_same_parser() {
        for n in proto::JOINT_NAMES {
            assert!(parse(&format!("position {n} rad=0.1 valid_for_ms=50"), 1, 2).is_ok());
            assert!(parse(&format!("torque {n} on"), 1, 3).is_ok());
        }
        assert!(parse("position mouth rad=NaN valid_for_ms=1", 1, 2).is_err());
        assert!(parse("gains mouth p=1 i=0", 1, 2).is_err());
    }
}
