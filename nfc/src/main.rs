//! `nfcd` — owns the NFC reader, and starts a gamepad pairing when a pairing tag is touched.
//!
//! ## What it does
//!
//! Polls the reader five times a second, asking as ISO 14443A and then as ISO 15693, so a
//! Type 2 tag (an NTAG) and a Type 5 one (an ST25TV, an ICODE) both answer. When a tag arrives
//! whose NDEF says [`ndef::PAIRING`]: if a pad is already connected nothing happens, otherwise
//! `configd` is asked to pair whichever pad is in pairing mode — with a quack from `robotd` when
//! the tag is read and a different sound once the pad is paired. The rule and why it is that rule
//! are in [`nfc::pairing`].
//!
//! ## Writing a tag
//!
//! `nfc.write` on [`proto::socket::NFC`] — `robotctl nfc-flash <text>`, or `duckctl nfc-flash`
//! over Bluetooth — writes a text onto the next tag touched, or onto one already lying on the
//! reader, then reads it back. The robot asks for a tag with the pad-lost sound, the questioning
//! quack, when a write starts waiting. While a write waits, a touched tag is written rather than acted
//! on, and the tag just written is not then read as a touch: writing a pairing tag does not pair.
//! The quack says it took. An NTAG and, on the beta board's reader, a Type 5 tag are written —
//! a blank Type 5 one formatted first; see [`tag::write_ndef_area`] and [`vtag::write_ndef_area`]
//! for what is never touched.
//!
//! ## Why its own daemon
//!
//! The reader is a device someone has to own, and nothing else here wants it: `configd` is in the
//! recovery path and must not wait on a serial port, and `padd` is unprivileged on purpose. So this
//! owns the port and is an ordinary client of the two sockets for everything else — the same
//! `pad.status`, `pad.pair` and `robot.sound` that `robotctl pad pair <mac>` and `robotctl quack`
//! send. It holds no Bluetooth access at all.
//!
//! ## No reader is the ordinary case
//!
//! Most robots have none fitted. A missing port is one line in the journal and a retry every few
//! seconds, so plugging a reader in is all it takes — no restart, nothing to enable.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use nfc::clrc663::Clrc663;
use nfc::pairing::{self, Outcome, Robot, Touches};
use nfc::reader::{Exchange, Reader};
use nfc::serial::Serial;
use nfc::spi::Spidev;
use nfc::st25r100::{Antenna, Proto, St25r100};
use nfc::{Error, ndef, tag, vtag};

/// Between attempts to open a reader that is not there, doubling to the cap.
///
/// Absent is forever on most robots, so this settles at one quiet attempt every half minute; a
/// reader plugged in is found within that.
const RETRY_MIN: Duration = Duration::from_secs(2);
const RETRY_MAX: Duration = Duration::from_secs(30);

/// How long `pad.pair` may take to answer. `configd` looks for 15 s by default and then says so;
/// this only has to outlast that, so a wedged `configd` does not stop the reader for good.
const PAIR_ANSWER: Duration = Duration::from_secs(40);
const QUICK_ANSWER: Duration = Duration::from_secs(5);

#[derive(Parser, Debug)]
#[command(
    name = "nfcd",
    about = "NFC reader daemon: touch a pairing tag, pair the gamepad in pairing mode",
    version
)]
struct Args {
    /// The reader: a serial port for the CLRC663 on USB, or `/dev/spidev*` for the beta board's
    /// ST25R100. Absent, the board decides: `/dev/spidev1.0` on a beta, `/dev/ttyACM0` otherwise.
    #[arg(long)]
    device: Option<PathBuf>,

    /// Polls a second. Detecting a tag alone costs ~35 ms on the USB reader, so 20 is the ceiling.
    /// The beta board asks twice on each of two antennas, ~110 ms with nothing there: keep it at
    /// 5 or below.
    #[arg(long, default_value_t = 5.0)]
    hz: f64,

    #[arg(long, default_value = proto::socket::CONFIG)]
    config_socket: PathBuf,

    #[arg(long, default_value = proto::socket::ROBOT)]
    robot_socket: PathBuf,

    /// Where `nfc.write` is served.
    #[arg(long, default_value = proto::socket::NFC)]
    socket: PathBuf,
}

/// Who may ask for a tag to be written: the group that may drive the robot. `btd` is in it, which
/// is how `duckctl` reaches this over Bluetooth.
const SOCKET_GROUP: &str = "robot";
const SOCKET_MODE: u32 = 0o660;
/// Past a write's own timeout, how long its asker waits for the poll loop to answer — a write
/// under way when the time ran out.
const ANSWER_MARGIN: Duration = Duration::from_secs(5);
/// "Touch a tag": the questioning quack a lost pad gets, so whoever ran the command hears that the
/// robot is waiting, and for what.
const WAITING: proto::SoundTag = proto::SoundTag::Inquire;

fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    duck_ipc_proto::log_startup_identity!("nfcd");

    if !(args.hz > 0.0 && args.hz <= 20.0) {
        tracing::error!(hz = args.hz, "--hz must be above 0 and at most 20");
        return std::process::ExitCode::FAILURE;
    }
    let period = Duration::from_secs_f64(1.0 / args.hz);
    let mut robot = Sockets {
        config: args.config_socket.clone(),
        robot: args.robot_socket.clone(),
    };

    let reader_up = Arc::new(AtomicBool::new(false));
    let (asked, jobs) = mpsc::channel();
    let asker = Sockets {
        config: args.config_socket.clone(),
        robot: args.robot_socket.clone(),
    };
    if let Err(e) = serve(&args.socket, asked, reader_up.clone(), asker) {
        // Tags are still read; only writing one is gone.
        tracing::warn!(socket = %args.socket.display(), error = %e, "not serving nfc.write");
    }
    let mut writes = Writes {
        asked: jobs,
        waiting: None,
    };

    let device = args.device.clone().unwrap_or_else(default_device);
    let mut backoff = RETRY_MIN;
    // The last reason there was no reader, so a reader that stays absent is one line, not one a
    // retry.
    let mut said: Option<String> = None;
    loop {
        match open(&device) {
            Ok(mut chip) => {
                tracing::warn!(device = %device.display(), hz = args.hz, "reader up — touch a tag");
                said = None;
                backoff = RETRY_MIN;
                reader_up.store(true, Ordering::Release);
                let why = watch(&mut chip, period, &mut robot, &mut writes);
                reader_up.store(false, Ordering::Release);
                tracing::warn!(device = %device.display(), error = %why, "reader lost");
            }
            Err(e) => {
                let why = e.to_string();
                if said.as_deref() != Some(why.as_str()) {
                    tracing::info!(device = %device.display(), error = %why, "no NFC reader; retrying");
                    said = Some(why);
                }
                sleep(backoff);
                backoff = (backoff * 2).min(RETRY_MAX);
            }
        }
    }
}

/// The reader this board has, when nobody named one.
fn default_device() -> PathBuf {
    match robotd_params::board::Board::detected() {
        Some(board) if board.is_beta_main_board() => PathBuf::from("/dev/spidev1.0"),
        _ => PathBuf::from("/dev/ttyACM0"),
    }
}

/// Either reader, behind the one interface the tag layer uses.
enum Chip {
    /// The bench reader: a CLRC663 on USB serial.
    Usb(Clrc663<Serial>),
    /// The beta board's ST25R100, with its two antennas polled in turn.
    Spi(St25r100<Spidev>),
}

/// Which protocol a tag answered on, so the read that follows knows its memory layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tech {
    /// ISO 14443A: a Type 2 tag (NTAG), read a page at a time.
    A,
    /// ISO 15693: a Type 5 tag (ST25TV, ICODE), read a block at a time.
    V,
}

impl Chip {
    fn begin(&mut self) -> nfc::Result<()> {
        match self {
            Chip::Usb(c) => c.begin(),
            Chip::Spi(c) => c.begin(),
        }
    }

    fn field_off(&mut self) -> nfc::Result<()> {
        match self {
            Chip::Usb(c) => c.field_off(),
            Chip::Spi(c) => c.field_off(),
        }
    }

    /// Look for a tag on every antenna in turn, asking as ISO 14443A first and then as ISO
    /// 15693; the first one seen wins, and the field stays on that antenna for whatever is read
    /// next.
    fn select(&mut self) -> nfc::Result<(Vec<u8>, Tech)> {
        let Chip::Spi(c) = self else {
            return tag::select(self).map(|uid| (uid, Tech::A));
        };
        // Whatever the last touch left the chip framing, the 14443A ask starts from its own.
        c.set_proto(Proto::A)?;
        let mut last = Error::NoTag;
        for antenna in [Antenna::One, Antenna::Two] {
            c.set_antenna(antenna)?;
            match tag::select(c) {
                Ok(uid) => return Ok((uid, Tech::A)),
                Err(e) if !e.is_tag() => return Err(e),
                _ => {}
            }
            // The 14443A error is superseded by whatever the 15693 ask says. The field is cycled
            // so it comes back with the AM regulator ISO 15693 is sent through.
            c.set_proto(Proto::V)?;
            c.reset_field()?;
            match vtag::inventory(c) {
                Ok(uid) => return Ok((uid, Tech::V)),
                Err(e) if e.is_tag() => last = e,
                Err(e) => return Err(e),
            }
            c.set_proto(Proto::A)?;
        }
        Err(last)
    }
}

impl Reader for Chip {
    fn reset_field(&mut self) -> nfc::Result<()> {
        match self {
            Chip::Usb(c) => Reader::reset_field(c),
            Chip::Spi(c) => c.reset_field(),
        }
    }

    fn transceive(&mut self, data: &[u8], exchange: Exchange) -> nfc::Result<Vec<u8>> {
        match self {
            Chip::Usb(c) => Reader::transceive(c, data, exchange),
            Chip::Spi(c) => c.transceive(data, exchange),
        }
    }
}

fn open(device: &Path) -> nfc::Result<Chip> {
    let mut chip = if device.to_string_lossy().starts_with("/dev/spidev") {
        Chip::Spi(St25r100::new(Spidev::open(device)?, Antenna::One))
    } else {
        Chip::Usb(Clrc663::new(Serial::open(device)?))
    };
    chip.begin()?;
    Ok(chip)
}

/// Poll until the reader stops answering, acting on every touch. Returns why it stopped.
fn watch(chip: &mut Chip, period: Duration, robot: &mut dyn Robot, writes: &mut Writes) -> Error {
    let mut touches = Touches::default();
    loop {
        let start = Instant::now();
        writes.expire();
        let touched = match chip.select() {
            Ok(found) => Some(found),
            Err(e) if e.is_tag() => None,
            Err(e) => {
                // A desynchronised link comes back with a soft reset; an unplugged one does not,
                // and is the caller's to reopen.
                if matches!(e, Error::Desync(_)) && chip.begin().is_ok() {
                    tracing::info!(error = %e, "reader out of step; reset");
                    continue;
                }
                return e;
            }
        };

        if let Some((uid, tech)) = touched.as_ref()
            && let Some(job) = writes.take()
        {
            let lost = write_touched(chip, robot, uid, *tech, job);
            // The tag just written stays where it is: it is not a touch to act on.
            touches.seen(Some(uid));
            if let Some(e) = lost {
                return e;
            }
        } else if let Some((uid, tech)) =
            touched.as_ref().map(|(uid, tech)| (uid.as_slice(), *tech))
            && touches.seen(Some(uid))
        {
            let id = hex(uid);
            let read = match tech {
                Tech::A => tag::read_ndef_area(chip),
                Tech::V => vtag::read_ndef_area(chip, uid),
            };
            match read.map(|data| ndef::parse(&data)) {
                Err(e) if !e.is_tag() => return e,
                Err(e) => {
                    tracing::info!(tag = %id, error = %e, "tag not read; trying again");
                    touches.unread();
                }
                Ok(Err(why)) => tracing::warn!(tag = %id, %why, "tag holds no readable NDEF"),
                Ok(Ok(records)) if !ndef::asks_to_pair(&records) => {
                    tracing::warn!(tag = %id, ?records, "not a pairing tag")
                }
                Ok(Ok(_)) => {
                    // Off while the pairing blocks: no reason to power the antenna for the
                    // fifteen seconds nobody is reading it.
                    if let Err(e) = chip.field_off() {
                        return e;
                    }
                    tracing::warn!(tag = %id, "pairing tag touched — looking for a pad in pairing mode");
                    report(pairing::on_touch(robot));
                }
            }
        } else if touched.is_none() {
            touches.seen(None);
        }

        if let Some(rest) = period.checked_sub(start.elapsed()) {
            sleep(rest);
        }
    }
}

/// A `nfc.write` waiting for a tag.
struct Job {
    text: String,
    until: Instant,
    answer: mpsc::Sender<proto::NfcWriteResult>,
}

/// The writes asked for, in order, and the one waiting for a tag now.
struct Writes {
    asked: mpsc::Receiver<Job>,
    waiting: Option<Job>,
}

impl Writes {
    /// Take up the next write asked for, answering any whose time ran out with no tag.
    fn expire(&mut self) {
        loop {
            if self.waiting.is_none() {
                self.waiting = self.asked.try_recv().ok();
            }
            let Some(job) = self.waiting.take_if(|job| Instant::now() >= job.until) else {
                return;
            };
            tracing::warn!(text = %job.text, "no tag touched to write");
            let _ = job
                .answer
                .send(not_written(None, "no tag touched in time".into()));
        }
    }

    fn take(&mut self) -> Option<Job> {
        self.waiting.take()
    }
}

fn not_written(uid: Option<String>, reason: String) -> proto::NfcWriteResult {
    proto::NfcWriteResult {
        written: false,
        uid,
        reason: Some(reason),
    }
}

/// Write a job's text onto the tag just selected and answer it. Returns the error when it was the
/// reader rather than the tag that failed, which ends [`watch`].
fn write_touched(
    chip: &mut Chip,
    robot: &mut dyn Robot,
    uid: &[u8],
    tech: Tech,
    job: Job,
) -> Option<Error> {
    let id = hex(uid);
    let written = ndef::text_tlv(&job.text)
        .map_err(Error::Tag)
        .and_then(|data| match tech {
            Tech::A => tag::write_ndef_area(chip, &data),
            Tech::V => vtag::write_ndef_area(chip, uid, &data),
        });
    match written {
        Ok(()) => {
            tracing::warn!(tag = %id, text = %job.text, "tag written");
            if let Err(why) = robot.sound(pairing::HEARD) {
                tracing::info!(%why, "the robot would not quack for the tag");
            }
            let _ = job.answer.send(proto::NfcWriteResult {
                written: true,
                uid: Some(id),
                reason: None,
            });
            None
        }
        Err(e) => {
            tracing::warn!(tag = %id, text = %job.text, error = %e, "tag not written");
            let _ = job.answer.send(not_written(Some(id), e.to_string()));
            (!e.is_tag()).then_some(e)
        }
    }
}

/// Serve `nfc.write` on `socket`, on threads of its own: each asker waits for its tag, and the
/// poll loop answers through the [`Job`] it is handed.
fn serve(
    socket: &Path,
    asked: mpsc::Sender<Job>,
    reader_up: Arc<AtomicBool>,
    robot: Sockets,
) -> std::io::Result<()> {
    if let Some(parent) = socket.parent() {
        // `RuntimeDirectory=nfcd` has made it on a board; tried anyway for a run by hand.
        let _ = std::fs::create_dir_all(parent);
    }
    // systemd removes the directory with the unit, so a socket here is one a killed nfcd left.
    let _ = std::fs::remove_file(socket);
    let listener = UnixListener::bind(socket)?;
    std::fs::set_permissions(socket, std::fs::Permissions::from_mode(SOCKET_MODE))?;
    if let Err(e) = give_to_group(socket, SOCKET_GROUP) {
        tracing::warn!(error = %e, group = SOCKET_GROUP, "nfc.write stays private to nfcd");
    }
    tracing::info!(socket = %socket.display(), "serving nfc.write");
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    let asked = asked.clone();
                    let reader_up = reader_up.clone();
                    let mut robot = robot.clone();
                    std::thread::spawn(move || {
                        if let Err(e) = client(stream, &asked, &reader_up, &mut robot) {
                            tracing::debug!(error = %e, "nfc.write client ended");
                        }
                    });
                }
                Err(e) => {
                    tracing::warn!(error = %e, "accept failed; retrying in one second");
                    sleep(Duration::from_secs(1));
                }
            }
        }
    });
    Ok(())
}

/// One connection: requests answered in turn until it closes.
fn client(
    stream: UnixStream,
    asked: &mpsc::Sender<Job>,
    reader_up: &AtomicBool,
    robot: &mut Sockets,
) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream.try_clone()?);
    let mut stream = stream;
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Ok(());
        }
        let response = match serde_json::from_str::<proto::Request>(line.trim()) {
            Err(e) => proto::Response::err(
                None,
                proto::Error::new(proto::code::PARSE_ERROR, e.to_string()),
            ),
            Ok(request) => {
                let id = request.id.clone();
                match request.as_call() {
                    Ok(proto::Call::Hello(_)) => proto::Response::ok(
                        id,
                        &proto::HelloResult {
                            api_version: proto::API_VERSION,
                            daemon_version: proto::semver::Version::parse(env!(
                                "CARGO_PKG_VERSION"
                            ))
                            .ok(),
                            revision: proto::build_info!().revision.map(str::to_owned),
                        },
                    ),
                    Ok(proto::Call::NfcWrite(params)) => proto::Response::ok(
                        id,
                        &write_when_touched(params, asked, reader_up, robot),
                    ),
                    Ok(_) => proto::Response::err(
                        id,
                        proto::Error::new(
                            proto::code::METHOD_NOT_FOUND,
                            "nfcd serves nfc.write and nothing else",
                        ),
                    ),
                    Err(e) => proto::Response::err(id, e),
                }
            }
        };
        let mut out = serde_json::to_vec(&response).map_err(std::io::Error::other)?;
        out.push(b'\n');
        stream.write_all(&out)?;
    }
}

/// Hand a write to the poll loop and wait for its answer.
fn write_when_touched(
    params: proto::NfcWriteParams,
    asked: &mpsc::Sender<Job>,
    reader_up: &AtomicBool,
    robot: &mut Sockets,
) -> proto::NfcWriteResult {
    if let Err(why) = ndef::text_tlv(&params.text) {
        return not_written(None, why);
    }
    if !reader_up.load(Ordering::Acquire) {
        return not_written(None, "no NFC reader answering on this robot".into());
    }
    let wait = params.timeout();
    let (answer, answered) = mpsc::channel();
    tracing::warn!(text = %params.text, seconds = wait.as_secs(), "touch a tag to write it");
    let job = Job {
        text: params.text,
        until: Instant::now() + wait,
        answer,
    };
    // Before the job is handed over, so a tag already on the reader is asked for before its quack
    // says it took.
    if let Err(why) = robot.sound(WAITING) {
        tracing::info!(%why, "the robot would not quack to ask for a tag");
    }
    if asked.send(job).is_err() {
        return not_written(None, "nfcd is stopping".into());
    }
    answered
        .recv_timeout(wait + ANSWER_MARGIN)
        .unwrap_or_else(|_| not_written(None, "no answer from the reader in time".into()))
}

/// Hand the socket to `group`, as `tofd` does its own. A missing group is the caller's warning.
fn give_to_group(socket: &Path, group: &str) -> std::io::Result<()> {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;

    let name = CString::new(group).map_err(std::io::Error::other)?;
    // SAFETY: `getgrnam` reads the group database and returns a pointer into storage it owns; the
    // name is a valid C string for the length of the call, and no other thread of this process
    // reads the group database.
    let entry = unsafe { libc::getgrnam(name.as_ptr()) };
    if entry.is_null() {
        return Err(std::io::Error::other(format!(
            "no {group} group on this system"
        )));
    }
    // SAFETY: checked non-null above; `getgrnam` fills the whole `struct group` when it returns one.
    let gid = unsafe { (*entry).gr_gid };
    let path = CString::new(socket.as_os_str().as_bytes()).map_err(std::io::Error::other)?;
    // SAFETY: a valid C string path; `-1` as the owner leaves it alone.
    if unsafe { libc::chown(path.as_ptr(), u32::MAX, gid) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

fn report(outcome: Outcome) {
    match outcome {
        Outcome::AlreadyConnected(pad) => {
            tracing::warn!(driving = %pad.mac, name = %pad.name, "a pad is already connected — nothing to do")
        }
        Outcome::Paired(pad) => tracing::warn!(mac = %pad.mac, name = %pad.name, "paired"),
        Outcome::PairedSilently(pad, why) => {
            tracing::warn!(mac = %pad.mac, name = %pad.name, %why, "paired, but the robot would not make a sound")
        }
        Outcome::NotPaired(reason, detail) => tracing::warn!(
            ?reason,
            ?detail,
            "not paired — is the pad in pairing mode? Lift the tag and touch again to retry"
        ),
        Outcome::Failed(why) => tracing::error!(%why, "could not ask the robot"),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(":")
}

/// `configd` and `robotd`, a connection per call.
///
/// Fresh each time because touches are rare and either daemon may have restarted since the last
/// one: a held connection would be a dead one exactly when someone touches a tag.
#[derive(Clone)]
struct Sockets {
    config: PathBuf,
    robot: PathBuf,
}

impl Robot for Sockets {
    fn pads(&mut self) -> Result<Vec<proto::Pad>, String> {
        let status: proto::PadStatusResult =
            call(&self.config, &proto::Call::PadStatus, QUICK_ANSWER)?;
        Ok(status.pads)
    }

    fn pair(&mut self) -> Result<proto::PadPairResult, String> {
        let params = proto::PadPairParams::default();
        call(&self.config, &proto::Call::PadPair(params), PAIR_ANSWER)
    }

    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String> {
        let sound = proto::Call::RobotSound(proto::SoundParams { tag, hold: None });
        let outcome: proto::IntentResult = call(&self.robot, &sound, QUICK_ANSWER)?;
        if outcome.accepted {
            Ok(())
        } else {
            Err(outcome.reason.unwrap_or_else(|| "refused".into()))
        }
    }
}

fn call<T: for<'de> serde::Deserialize<'de>>(
    socket: &Path,
    call: &proto::Call,
    answer_within: Duration,
) -> Result<T, String> {
    let fail = |e: std::io::Error| format!("{}: {e}", socket.display());
    let mut stream = UnixStream::connect(socket).map_err(fail)?;
    stream.set_read_timeout(Some(answer_within)).map_err(fail)?;
    let mut line = serde_json::to_vec(&proto::Request::call(proto::Id::Number(1), call))
        .map_err(|e| e.to_string())?;
    line.push(b'\n');
    stream.write_all(&line).map_err(fail)?;

    let mut answer = String::new();
    BufReader::new(stream)
        .read_line(&mut answer)
        .map_err(fail)?;
    let response: proto::Response =
        serde_json::from_str(&answer).map_err(|e| format!("unparsable answer: {e}"))?;
    if let Some(error) = response.error {
        return Err(error.message);
    }
    response
        .result_as()
        .map_err(|e| format!("unexpected answer: {e}"))
}
