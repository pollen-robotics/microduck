//! `map.*` on [`proto::socket::MAP`].
//!
//! A connection may ask any number of questions, one at a time, until it asks for
//! [`proto::Call::MapStream`] — then it becomes a stream of [`proto::method::MAP_STATE`]
//! notifications and reads nothing more, like every other subscription in this system.

use std::io::ErrorKind;
use std::sync::mpsc::SyncSender;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

use crate::worker::{Input, Shared};

/// How often a stream subscriber hears the pose. The pose itself moves at the state stream's rate;
/// a viewer drawing a duck on a map wants a few updates a second, not fifty.
const STREAM_PERIOD: Duration = Duration::from_millis(250);

/// The grid's resolution when the caller does not say, and the bounds on what it may say: finer
/// than the keyframes' own cells is invention, and coarser than this is not a map.
pub const DEFAULT_RES_M: f64 = 0.05;
const MIN_RES_M: f64 = 0.025;
const MAX_RES_M: f64 = 0.5;

#[derive(Clone)]
pub struct Ctx {
    pub shared: Arc<Mutex<Shared>>,
    /// `None` when mapping is off: there is no worker to ask.
    pub worker: Option<SyncSender<Input>>,
}

impl Ctx {
    fn status(&self) -> proto::MapStatusResult {
        self.shared.lock().map(|s| s.status()).unwrap_or_default()
    }

    /// Ask the worker something and wait for the answer off the async thread.
    async fn ask<T: Send + 'static>(
        &self,
        make: impl FnOnce(SyncSender<T>) -> Input + Send + 'static,
    ) -> Result<T, String> {
        let Some(tx) = self.worker.clone() else {
            return Err("mapping is off ([map] enabled = false in robotd.toml)".to_owned());
        };
        tokio::task::spawn_blocking(move || {
            let (reply, rx) = std::sync::mpsc::sync_channel(1);
            tx.send(make(reply))
                .map_err(|_| "the map worker has stopped".to_owned())?;
            // A relocalization search can hold the worker for a couple of seconds on the board.
            rx.recv_timeout(Duration::from_secs(15))
                .map_err(|_| "the map worker did not answer".to_owned())
        })
        .await
        .map_err(|e| e.to_string())?
    }
}

pub async fn connection(stream: UnixStream, ctx: Ctx) -> std::io::Result<()> {
    let (read, mut write) = stream.into_split();
    let mut reader = BufReader::new(read);
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line).await? == 0 {
            return Ok(());
        }
        let request: proto::Request = match serde_json::from_str(line.trim()) {
            Ok(r) => r,
            Err(e) => {
                let r = proto::Response::err(
                    None,
                    proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
                );
                write_line(&mut write, &r).await?;
                continue;
            }
        };
        let id = request.id.clone();
        let response = match request.as_call() {
            Ok(proto::Call::MapStatus) => proto::Response::ok(id, &ctx.status()),
            Ok(proto::Call::MapGrid(p)) => {
                let res = p.res_m.unwrap_or(DEFAULT_RES_M).clamp(MIN_RES_M, MAX_RES_M);
                match ctx.ask(move |reply| Input::Grid(res, reply)).await {
                    // Nothing mapped yet is an empty grid, not an error: it is the true answer.
                    Ok(grid) => proto::Response::ok(id, &grid.unwrap_or_default()),
                    Err(e) => {
                        proto::Response::err(id, proto::Error::new(proto::code::INTERNAL_ERROR, e))
                    }
                }
            }
            Ok(proto::Call::MapWipe) => match ctx.ask(Input::Wipe).await {
                Ok(keyframes) => proto::Response::ok(id, &proto::MapWipeResult { keyframes }),
                Err(e) => {
                    proto::Response::err(id, proto::Error::new(proto::code::INTERNAL_ERROR, e))
                }
            },
            Ok(proto::Call::MapStream) => {
                write_line(&mut write, &proto::Response::ok(id, &ctx.status())).await?;
                let mut sink = tokio::io::sink();
                return tokio::select! {
                    r = stream_states(&mut write, &ctx) => r,
                    r = tokio::io::copy(&mut reader, &mut sink) => r.map(|_| ()),
                };
            }
            _ => proto::Response::err(
                id,
                proto::Error::new(
                    proto::code::METHOD_NOT_FOUND,
                    "mapd serves map.* and nothing else",
                ),
            ),
        };
        write_line(&mut write, &response).await?;
    }
}

async fn stream_states(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    ctx: &Ctx,
) -> std::io::Result<()> {
    let mut tick = tokio::time::interval(STREAM_PERIOD);
    loop {
        tick.tick().await;
        let state = ctx.status().state;
        match write_line(write, &proto::Request::notify_map_state(&state)).await {
            Err(e) if e.kind() == ErrorKind::BrokenPipe => return Ok(()),
            other => other?,
        }
    }
}

async fn write_line(
    write: &mut tokio::net::unix::OwnedWriteHalf,
    message: &impl serde::Serialize,
) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(message)?;
    line.push(b'\n');
    write.write_all(&line).await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::worker::Worker;
    use maploc::mapper::MapperConfig;
    use maploc::sim::{Drift, Out, Robot, Step, World};
    use tokio::io::AsyncBufReadExt;

    async fn call(stream: &mut BufReader<UnixStream>, c: &proto::Call) -> proto::Response {
        let line = serde_json::to_string(&proto::Request::call(proto::Id::Number(7), c)).unwrap();
        stream
            .get_mut()
            .write_all(format!("{line}\n").as_bytes())
            .await
            .unwrap();
        let mut back = String::new();
        stream.read_line(&mut back).await.unwrap();
        serde_json::from_str(&back).unwrap()
    }

    /// The whole service minus its sockets to `robotd` and `tofd`: a synthetic robot maps a room
    /// through the worker, and the map comes back over `map.*` — then is wiped, and stays wiped
    /// across a restart.
    #[tokio::test]
    async fn a_room_mapped_through_the_worker_comes_back_over_the_socket() {
        let dir = tempfile::tempdir().unwrap();
        let map_path = dir.path().join("map.json");
        let shared = Arc::new(Mutex::new(Shared::default()));
        let (tx, rx) = std::sync::mpsc::sync_channel(100_000);
        let worker = Worker::new(
            MapperConfig::default(),
            map_path.clone(),
            Duration::from_secs(3600),
            shared.clone(),
        );
        let worker_thread = std::thread::spawn(move || worker.run(rx));

        // Drive a short walk with stops straight into the worker's channel.
        let mut robot = Robot::new(
            World::room(3.0, 2.5),
            maploc::se2::Pose2::new(1.0, 1.0, 0.0),
            Drift::NONE,
        );
        let mut steps = vec![Step::Stop {
            secs: 3.0,
            sweep: true,
        }];
        for p in [[1.6, 1.0], [2.0, 1.4], [1.4, 1.6]] {
            steps.push(Step::Walk { to: p });
            steps.push(Step::Stop {
                secs: 3.0,
                sweep: true,
            });
        }
        let feed = tx.clone();
        robot.run(&steps, &mut |o| {
            let _ = feed.send(match o {
                Out::State(s) => Input::State(Box::new(s)),
                Out::Depth(f) => Input::Depth(f),
            });
        });

        let socket = dir.path().join("map.sock");
        let listener = tokio::net::UnixListener::bind(&socket).unwrap();
        let ctx = Ctx {
            shared,
            worker: Some(tx.clone()),
        };
        tokio::spawn(async move {
            loop {
                let (s, _) = listener.accept().await.unwrap();
                tokio::spawn(connection(s, ctx.clone()));
            }
        });
        let mut c = BufReader::new(UnixStream::connect(&socket).await.unwrap());

        // The grid is asked of the worker, so it is answered after every queued input.
        let grid: proto::MapGridResult = call(
            &mut c,
            &proto::Call::MapGrid(proto::MapGridParams::default()),
        )
        .await
        .result_as()
        .unwrap();
        assert!(grid.width > 10 && grid.cells.len() == (grid.width * grid.height) as usize);
        assert!(
            grid.cells.contains('#') && grid.cells.contains('.'),
            "no walls or no floor"
        );
        assert!(grid.keyframes.len() >= 3, "{} stops", grid.keyframes.len());

        let status: proto::MapStatusResult = call(&mut c, &proto::Call::MapStatus)
            .await
            .result_as()
            .unwrap();
        assert!(status.state.pose.is_some() && status.state.lost.is_none());
        assert_eq!(status.state.keyframes as usize, grid.keyframes.len());

        let wiped: proto::MapWipeResult = call(&mut c, &proto::Call::MapWipe)
            .await
            .result_as()
            .unwrap();
        assert_eq!(wiped.keyframes as usize, grid.keyframes.len());
        let grid: proto::MapGridResult = call(
            &mut c,
            &proto::Call::MapGrid(proto::MapGridParams::default()),
        )
        .await
        .result_as()
        .unwrap();
        assert_eq!(grid.width, 0, "a wiped map still renders");

        let (reply, done) = std::sync::mpsc::sync_channel(1);
        tx.send(Input::Shutdown(reply)).unwrap();
        done.recv().unwrap();
        worker_thread.join().unwrap();
        assert!(!map_path.exists(), "a wiped map came back from disk");
    }
}
