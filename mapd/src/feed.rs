//! The two inputs: `robot.state` from `robotd`, `tof.frame` from `tofd`.
//!
//! Each is a plain thread holding one subscription and reconnecting when it drops — the theremin's
//! shape, for the theremin's reason: a daemon restarting under us is ordinary (an update restarts
//! both), and the map must pick up where it was without anyone restarting `mapd`.
//!
//! Samples go to the worker with `try_send`. A worker busy with a relocalization search lets the
//! channel fill, and what does not fit is dropped rather than queued behind it: a frame the worker
//! reaches seconds late is placed by a history that has moved on, and the mapper already treats a
//! gap in the state stream as a gap, never interpolating across it.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::mpsc::{SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_ipc_proto as proto;
use maploc::input::{DepthFrame, StateSample};

use crate::worker::{Input, Shared};

const RECONNECT: Duration = Duration::from_secs(2);

#[derive(Clone, Copy)]
pub enum Source {
    Robot { hz: u32 },
    Tof,
}

impl Source {
    fn name(self) -> &'static str {
        match self {
            Source::Robot { .. } => "robotd",
            Source::Tof => "tofd",
        }
    }

    fn call(self) -> proto::Call {
        match self {
            Source::Robot { hz } => {
                proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(hz) })
            }
            Source::Tof => proto::Call::TofStream,
        }
    }

    fn mark(self, shared: &Mutex<Shared>, down: Option<String>) {
        if let Ok(mut s) = shared.lock() {
            match self {
                Source::Robot { .. } => s.robot_down = down,
                Source::Tof => s.tof_down = down,
            }
        }
    }
}

pub fn spawn(source: Source, socket: PathBuf, tx: SyncSender<Input>, shared: Arc<Mutex<Shared>>) {
    std::thread::Builder::new()
        .name(format!("feed-{}", source.name()))
        .spawn(move || {
            let mut said = None::<String>;
            loop {
                let err = stream(source, &socket, &tx, &shared);
                let down = format!("{}: {err}", source.name());
                // Once per distinct failure: a robot with no ToF fitted would otherwise log this
                // every two seconds forever.
                if said.as_deref() != Some(&down) {
                    tracing::warn!(reason = %down, socket = %socket.display(), "input down; retrying");
                    said = Some(down.clone());
                }
                source.mark(&shared, Some(down));
                std::thread::sleep(RECONNECT);
            }
        })
        .expect("spawn a feed thread");
}

/// One connection, from its subscribe to whatever ended it.
fn stream(
    source: Source,
    socket: &PathBuf,
    tx: &SyncSender<Input>,
    shared: &Mutex<Shared>,
) -> String {
    let stream = match UnixStream::connect(socket) {
        Ok(s) => s,
        Err(e) => return format!("cannot connect: {e}"),
    };
    let mut writer = match stream.try_clone() {
        Ok(w) => w,
        Err(e) => return format!("clone: {e}"),
    };
    let request = proto::Request::call(proto::Id::Number(1), &source.call());
    let Ok(line) = serde_json::to_string(&request) else {
        return "encode".to_owned();
    };
    if let Err(e) = writer.write_all(format!("{line}\n").as_bytes()) {
        return format!("subscribe: {e}");
    }
    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    let mut flowing = false;
    let mut unusable = 0u32;
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Err(e) => return format!("read: {e}"),
            Ok(0) => return "closed the stream".to_owned(),
            Ok(_) => {}
        }
        let Ok(msg) = serde_json::from_str::<proto::Request>(&line) else {
            // The subscription's answer, or something a newer daemon says that this build does
            // not read. A refusal is worth saying; the rest is skipped.
            if let Ok(r) = serde_json::from_str::<proto::Response>(&line) {
                if let Some(e) = r.error {
                    return format!("refused the subscription: {}", e.message);
                }
                if let (Source::Tof, Ok(status)) = (source, r.result_as::<proto::TofStreamResult>())
                    && let Some(why) = status.unavailable
                {
                    source.mark(shared, Some(format!("tofd: {why}")));
                }
            }
            continue;
        };
        let input = match source {
            Source::Robot { .. } => msg
                .as_state()
                .map(|s| StateSample::from_proto(&s).map(|s| Input::State(Box::new(s)))),
            Source::Tof => msg
                .as_tof_frame()
                .map(|f| DepthFrame::from_proto(&f).map(Input::Depth)),
        };
        match input {
            None => continue,
            Some(None) => {
                // Parsed, but missing what mapping needs: a daemon predating the v24 telemetry,
                // or a tick before the sensors were read. Say it once things persist.
                unusable += 1;
                if unusable == 50 {
                    source.mark(
                        shared,
                        Some(format!(
                            "{}: its frames carry no capture clock or poses (predates API v24?)",
                            source.name()
                        )),
                    );
                }
            }
            Some(Some(input)) => {
                if !flowing {
                    flowing = true;
                    unusable = 0;
                    tracing::info!(source = source.name(), "input flowing");
                    source.mark(shared, None);
                }
                match tx.try_send(input) {
                    Ok(()) | Err(TrySendError::Full(_)) => {}
                    Err(TrySendError::Disconnected(_)) => return "worker stopped".to_owned(),
                }
            }
        }
    }
}
