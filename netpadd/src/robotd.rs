//! robotd's socket, from inside the one tokio task: every write and every answer bounded by a
//! timeout, because a robotd that stops reading must not freeze the receive loop and let the
//! socket fill behind it. Any failure is the caller's cue to exit; systemd brings us back.

use std::path::Path;
use std::time::Duration;

use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::UnixStream;

/// How long an answered request may take. robotd answers intents from memory.
const ANSWER: Duration = Duration::from_secs(1);

pub struct Robotd {
    stream: BufReader<UnixStream>,
    next_id: u64,
    /// One send interval: a write that cannot land in that has missed its slot anyway.
    write_timeout: Duration,
}

fn timed_out(what: &str) -> std::io::Error {
    std::io::Error::new(
        std::io::ErrorKind::TimedOut,
        format!("robotd {what} timed out"),
    )
}

impl Robotd {
    pub async fn connect(path: &Path, write_timeout: Duration) -> std::io::Result<Self> {
        let stream = UnixStream::connect(path).await?;
        Ok(Self {
            stream: BufReader::new(stream),
            next_id: 1,
            write_timeout,
        })
    }

    pub async fn write(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        tokio::time::timeout(self.write_timeout, self.stream.get_mut().write_all(bytes))
            .await
            .map_err(|_| timed_out("write"))?
    }

    pub async fn notify(&mut self, call: &proto::Call) -> std::io::Result<()> {
        let mut line = serde_json::to_vec(&proto::Request::notify(call))?;
        line.push(b'\n');
        self.write(&line).await
    }

    /// Notifications have no answers, so the only lines coming back are these, in order.
    pub async fn request(
        &mut self,
        call: &proto::Call,
    ) -> std::io::Result<Option<proto::Response>> {
        let id = proto::Id::Number(self.next_id);
        self.next_id += 1;
        let mut line = serde_json::to_vec(&proto::Request::call(id, call))?;
        line.push(b'\n');
        self.write(&line).await?;
        let mut answer = String::new();
        let n = tokio::time::timeout(ANSWER, self.stream.read_line(&mut answer))
            .await
            .map_err(|_| timed_out("answer"))??;
        if n == 0 {
            return Err(std::io::ErrorKind::UnexpectedEof.into());
        }
        match serde_json::from_str::<proto::Response>(&answer) {
            Ok(r) => Ok(Some(r)),
            Err(e) => {
                tracing::warn!(error = %e, raw = %answer.trim(), "unparsable answer");
                Ok(None)
            }
        }
    }

    /// Whether this robot is on wheels. `None` for an answer that did not say.
    pub async fn ask_roller(&mut self) -> std::io::Result<Option<bool>> {
        Ok(self
            .request(&proto::Call::RobotMode)
            .await?
            .and_then(|a| a.result_as::<proto::ModeResult>().ok())
            .map(|m| m.mode == "roller"))
    }
}
