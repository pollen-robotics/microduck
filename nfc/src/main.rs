//! `nfcd` — owns the NFC reader, and starts a gamepad pairing when a pairing tag is touched.
//!
//! ## What it does
//!
//! Polls the reader five times a second, asking as ISO 14443A and then as ISO 15693, so a
//! Type 2 tag (an NTAG) and a Type 5 one (an ST25TV, an ICODE) both answer. When a tag arrives
//! whose NDEF says [`ndef::PAIRING`]: if a pad is already connected nothing happens, otherwise
//! `configd` is asked to pair whichever pad is in pairing mode — with a quack from `robotd` when
//! the tag is read and a different sound once the pad is paired. The rule and why it is that rule
//! are in [`nfc::pairing`]. A tag naming a mode instead — `walk_mode`, `roller_skating_mode`, … —
//! asks `robotd` to switch to it; which names there are is [`nfc::modes`]. An e-ink display
//! ([`nfc::eink`]), or a `take_photo` tag, poses the robot for a picture taken with `mediad`'s
//! camera and kept in `/var/lib/robot/photos`; the sequence is [`nfc::photo`].
//!
//! `nfcd eink write` is the bench's: a picture written to an e-ink display on one antenna, with
//! `nfcd` itself stopped, since it holds the reader. It is how the write ([`nfc::eink`]) is tried
//! on a robot before the photo sequence does it.
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
//! ## Reading a tag
//!
//! `nfc.read` on the same socket — `robotctl nfc-read` — reads the next tag touched, or one
//! already on the reader, and answers with its UID and NDEF records. It waits, quacks and keeps
//! the tag from being acted on just as a write does: reading a pairing tag does not pair. A tag
//! that moves during the read is read again on the next poll, until the wait runs out.
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

use clap::{Parser, Subcommand, ValueEnum};
use duck_ipc_proto as proto;
use nfc::clrc663::Clrc663;
use nfc::modes::{self, Driver, ModeTag};
use nfc::pairing::{self, Outcome, Robot, Touches};
use nfc::photo;
use nfc::reader::{Exchange, Reader};
use nfc::serial::Serial;
use nfc::spi::Spidev;
use nfc::st25r100::{Antenna, Proto, St25r100};
use nfc::{Error, eink, ndef, tag, vtag};

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

/// Where the ST25R100's two antennas are: RFO2 under the top of the head, RFO1 in the beak. Found
/// on two robots (2026-11-16): an e-ink display written with `nfcd eink write` on duck-d556, and a
/// tag touched to each on cream-dvt-usine, `tag in the field` saying which antenna saw it.
///
/// The first reading, on an earlier face-board-v1, had them the other way round, and so did a
/// `nfc-test` run on duck-d325 (`NFC_ANT=1`, RFO1, wrote a display taken to be on the head). If a
/// robot disagrees, touch each antenna with `nfcd` running and read `antenna=` in that line.
const HEAD: Antenna = Antenna::Two;
const BEAK: Antenna = Antenna::One;

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

    /// `mediad`'s local frame socket, where the photo tag's picture comes from.
    #[arg(long, default_value = proto::socket::MEDIA)]
    media_socket: PathBuf,

    /// Where the photo tag's pictures are kept, one JPEG each, named for the local time.
    #[arg(long, default_value = "/var/lib/robot/photos")]
    photo_dir: PathBuf,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// An e-ink display, from the bench. Stop the `nfcd` service first: it holds the reader.
    #[command(subcommand)]
    Eink(EinkCommand),
}

#[derive(Subcommand, Debug)]
enum EinkCommand {
    /// Write a picture to the display on one antenna, and redraw it. Keep the display still
    /// until `refresh done`.
    Write(EinkWrite),
}

#[derive(clap::Args, Debug)]
struct EinkWrite {
    /// A JPEG or a PNG: its centre square, scaled to 240 × 240. None, the six-ink test pattern.
    image: Option<PathBuf>,

    /// The antenna the display is on.
    #[arg(long, value_enum, default_value_t = Place::Beak)]
    on: Place,

    /// The refresh's P1, hex: the panel's power-up before it draws, in 100 ms. `nfc-test`'s
    /// NFC_REFRESH_P1.
    #[arg(long, default_value = "85", value_parser = hex_byte)]
    p1: u8,

    /// Between two packets at first, in ms; a slow packet raises it. `nfc-test`'s
    /// NFC_D2_DELAY_MS.
    #[arg(long, default_value_t = 0)]
    pace_ms: u64,

    /// The time the display has to answer each block: FWT at this FWI, 14 being 4.9 s.
    #[arg(long, default_value_t = 14, value_parser = clap::value_parser!(u8).range(0..=14))]
    fwi: u8,

    #[arg(long, value_enum, default_value_t = DitherArg::Atkinson)]
    dither: DitherArg,

    /// A box blur of this radius before the dither. `nfc-test`'s NFC_BLUR.
    #[arg(long, default_value_t = 0)]
    blur: u32,

    /// How long to wait for a display on the antenna, in seconds.
    #[arg(long, default_value_t = 10)]
    find_within: u64,

    /// How long the redraw may take, in seconds.
    #[arg(long, default_value_t = 300)]
    refresh_within: u64,
}

#[derive(ValueEnum, Clone, Copy, Debug)]
enum Place {
    Head,
    Beak,
}

#[derive(ValueEnum, Clone, Copy, Debug)]
enum DitherArg {
    Atkinson,
    None,
}

fn hex_byte(s: &str) -> Result<u8, String> {
    u8::from_str_radix(s.trim_start_matches("0x"), 16).map_err(|e| format!("{s:?}: {e}"))
}

/// Who may ask for a tag to be written or read: the group that may drive the robot. `btd` is in it, which
/// is how `duckctl` reaches this over Bluetooth.
const SOCKET_GROUP: &str = "robot";
const SOCKET_MODE: u32 = 0o660;
/// Past a job's own timeout, how long its asker waits for the poll loop to answer — a write or
/// read under way when the time ran out.
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
    if let Some(Command::Eink(EinkCommand::Write(write))) = &args.command {
        let device = args.device.clone().unwrap_or_else(default_device);
        return eink_write(&device, write);
    }
    duck_ipc_proto::log_startup_identity!("nfcd");

    if !(args.hz > 0.0 && args.hz <= 20.0) {
        tracing::error!(hz = args.hz, "--hz must be above 0 and at most 20");
        return std::process::ExitCode::FAILURE;
    }
    let period = Duration::from_secs_f64(1.0 / args.hz);
    let mut robot = Sockets {
        config: args.config_socket.clone(),
        robot: args.robot_socket.clone(),
        media: args.media_socket.clone(),
        photos: args.photo_dir.clone(),
    };

    let reader_up = Arc::new(AtomicBool::new(false));
    let (asked, jobs) = mpsc::channel();
    let asker = Sockets {
        config: args.config_socket.clone(),
        robot: args.robot_socket.clone(),
        media: args.media_socket.clone(),
        photos: args.photo_dir.clone(),
    };
    if let Err(e) = serve(&args.socket, asked, reader_up.clone(), asker) {
        // Tags are still acted on; only writing and reading one on request is gone.
        tracing::warn!(socket = %args.socket.display(), error = %e, "not serving nfc.write");
    }
    let mut waiting = Jobs {
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
                let why = watch(&mut chip, period, &mut robot, &mut waiting);
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

/// `nfcd eink write`: the picture rendered, the display found on the antenna, written step by
/// step, the field off at the end.
fn eink_write(device: &Path, args: &EinkWrite) -> std::process::ExitCode {
    use nfc::eink::{Link, Progress, Settings, Writer, picture};
    let fail = |why: String| {
        tracing::error!(%why, "not written");
        std::process::ExitCode::FAILURE
    };
    let image = match &args.image {
        Some(path) => match image::open(path) {
            Ok(image) => image.to_rgb8(),
            Err(e) => return fail(format!("{}: {e}", path.display())),
        },
        None => {
            tracing::info!("no picture given: the test pattern");
            picture::test_pattern()
        }
    };
    let dither = match args.dither {
        DitherArg::Atkinson => picture::Dither::Atkinson,
        DitherArg::None => picture::Dither::None,
    };
    let packed = picture::render(&image, dither, args.blur);

    let mut chip = match open(device) {
        Ok(chip) => chip,
        Err(e) => return fail(format!("{}: {e}", device.display())),
    };
    let antenna = match args.on {
        Place::Head => HEAD,
        Place::Beak => BEAK,
    };
    if let Chip::Spi(c) = &mut chip
        && let Err(e) = c.set_proto(Proto::A).and_then(|()| c.set_antenna(antenna))
    {
        return fail(e.to_string());
    }
    tracing::info!(?antenna, on = ?args.on, "waiting for a display");
    let until = Instant::now() + Duration::from_secs(args.find_within);
    let outcome = loop {
        let e = match Link::open(&mut chip, args.fwi) {
            Ok(link) => break Ok(link),
            Err(e) => e,
        };
        if Instant::now() >= until {
            break Err(format!("no display: {e}"));
        }
        if e.is_tag() {
            tracing::debug!(error = %e, "no display yet");
        } else if matches!(e, Error::Desync(_)) {
            // What the daemon's poll does with it: a soft reset, the antenna chosen again.
            tracing::info!(error = %e, "reader out of step; reset");
            let reset = chip.begin().and_then(|()| match &mut chip {
                Chip::Spi(c) => c.set_proto(Proto::A).and_then(|()| c.set_antenna(antenna)),
                Chip::Usb(_) => Ok(()),
            });
            if let Err(e) = reset {
                break Err(format!("the reader did not come back: {e}"));
            }
        } else {
            break Err(format!("no display: {e}"));
        }
        sleep(Duration::from_millis(100));
    };
    let mut link = match outcome {
        Ok(link) => link,
        Err(why) => return fail(why),
    };
    tracing::info!(fwi = args.fwi, "display found");

    let settings = Settings {
        refresh_p1: args.p1,
        pace: Duration::from_millis(args.pace_ms),
        refresh_within: Duration::from_secs(args.refresh_within),
        ..Settings::default()
    };
    let mut writer = Writer::new(packed, settings);
    let started = Instant::now();
    let mut shown = None;
    let result = loop {
        match writer.step(&mut link, Instant::now()) {
            Ok(Progress::Done) => break Ok(()),
            Ok(progress) => {
                let stage = std::mem::discriminant(&progress);
                if shown != Some(stage) {
                    tracing::info!(?progress, "…");
                    shown = Some(stage);
                }
                sleep(writer.idle(Instant::now()));
            }
            Err(why) => break Err(why),
        }
    };
    let _ = chip.field_off();
    match result {
        Ok(()) => {
            tracing::info!(took = ?started.elapsed(), "written and drawn");
            std::process::ExitCode::SUCCESS
        }
        Err(why) => fail(why),
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
    /// ISO 14443A, a MIFARE Classic by its SAK: not read, and said so once rather than retried.
    Classic,
    /// ISO 15693: a Type 5 tag (ST25TV, ICODE), read a block at a time.
    V,
    /// ISO 14443-4 by its SAK: an e-ink display, as far as [`nfc::eink`] can tell. Not read.
    Eink,
}

impl Tech {
    /// What an ISO 14443A tag is, from the SAK it selected with.
    fn of(found: tag::Selected) -> (Vec<u8>, Tech) {
        let tech = if eink::is_eink(found.sak) {
            Tech::Eink
        } else if tag::is_mifare_classic(found.sak) {
            Tech::Classic
        } else {
            Tech::A
        };
        (found.uid, tech)
    }
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
    /// next. Which antenna saw it, on the board that has two.
    fn select(&mut self) -> nfc::Result<(Vec<u8>, Tech, Option<Antenna>)> {
        let Chip::Spi(c) = self else {
            return tag::select_tag(self).map(|found| {
                let (uid, tech) = Tech::of(found);
                (uid, tech, None)
            });
        };
        // Whatever the last touch left the chip framing, the 14443A ask starts from its own.
        c.set_proto(Proto::A)?;
        let mut last = Error::NoTag;
        for antenna in [Antenna::One, Antenna::Two] {
            c.set_antenna(antenna)?;
            match tag::select_tag(c) {
                Ok(found) => {
                    let (uid, tech) = Tech::of(found);
                    return Ok((uid, tech, Some(antenna)));
                }
                Err(e) if !e.is_tag() => return Err(e),
                _ => {}
            }
            // The 14443A error is superseded by whatever the 15693 ask says. The field is cycled
            // so it comes back with the AM regulator ISO 15693 is sent through.
            c.set_proto(Proto::V)?;
            c.reset_field()?;
            match vtag::inventory(c) {
                Ok(uid) => return Ok((uid, Tech::V, Some(antenna))),
                Err(e) if e.is_tag() => last = e,
                Err(e) => return Err(e),
            }
            c.set_proto(Proto::A)?;
        }
        Err(last)
    }
}

impl Chip {
    /// Whether an e-ink display ([`eink::is_eink`]) answers on `antenna` alone. The bench reader
    /// has one antenna, so it is asked there whatever `antenna` says.
    fn eink_at(&mut self, antenna: Antenna) -> nfc::Result<bool> {
        if let Chip::Spi(c) = self {
            c.set_proto(Proto::A)?;
            c.set_antenna(antenna)?;
        }
        match tag::select_tag(self) {
            Ok(found) => Ok(eink::is_eink(found.sak)),
            Err(e) if e.is_tag() => Ok(false),
            Err(e) => Err(e),
        }
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
fn watch(chip: &mut Chip, period: Duration, robot: &mut Sockets, waiting: &mut Jobs) -> Error {
    let mut touches = Touches::default();
    loop {
        let start = Instant::now();
        waiting.expire();
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

        if let Some((uid, tech, _)) = touched.as_ref()
            && let Some(job) = waiting.take()
        {
            let lost = match job.ask {
                Ask::Write { text, answer } => write_touched(chip, robot, uid, *tech, text, answer),
                Ask::Read { answer } => match read_touched(chip, robot, uid, *tech) {
                    Ok(result) => {
                        let _ = answer.send(result);
                        None
                    }
                    // Moved or at the edge of the field: asked again on the next poll, while
                    // there is time.
                    Err(e @ (Error::NoTag | Error::Tag(_) | Error::Collision(_))) => {
                        tracing::info!(tag = %hex(uid), error = %e, "tag not read; trying again");
                        waiting.again(Job {
                            ask: Ask::Read { answer },
                            until: job.until,
                        });
                        None
                    }
                    Err(e) => {
                        let _ = answer.send(not_read(Some(hex(uid)), e.to_string()));
                        (!e.is_tag()).then_some(e)
                    }
                },
            };
            // The tag just written or read stays where it is: it is not a touch to act on.
            touches.seen(Some(uid));
            if let Some(e) = lost {
                return e;
            }
        } else if let Some((uid, tech, antenna)) = touched
            .as_ref()
            .map(|(uid, tech, antenna)| (uid.as_slice(), *tech, *antenna))
            && touches.seen(Some(uid))
        {
            let id = hex(uid);
            let place = match antenna {
                Some(HEAD) => "head",
                Some(BEAK) => "beak",
                None => "reader",
            };
            tracing::warn!(tag = %id, ?tech, ?antenna, place, "tag in the field");
            // On the head, a display starts the photo; in the beak, it is what the sequence waits
            // for, and starts nothing. The bench reader has one antenna, the head's.
            if tech == Tech::Eink {
                if antenna == Some(BEAK) {
                    tracing::warn!(tag = %id, "e-ink in the beak — it starts nothing there");
                } else {
                    tracing::warn!(tag = %id, "e-ink on the head — hold still");
                    report_photo(photo::on_touch(&mut Session {
                        sockets: &mut *robot,
                        chip: &mut *chip,
                    }));
                }
                continue_after(period, start);
                continue;
            }
            match read_ndef(chip, uid, tech) {
                Err(e) if !e.is_tag() => return e,
                // The same answer every time it is asked: said once, until the tag is lifted.
                Err(Error::Unreadable(why)) => {
                    tracing::warn!(tag = %id, ?tech, %why, "tag cannot be read")
                }
                Err(e) => {
                    tracing::info!(tag = %id, error = %e, "tag not read; trying again");
                    touches.unread();
                }
                Ok(Err(why)) => tracing::warn!(tag = %id, %why, "tag holds no readable NDEF"),
                Ok(Ok(records)) if photo::in_records(&records) => {
                    tracing::warn!(tag = %id, "photo tag touched — hold still");
                    report_photo(photo::on_touch(&mut Session {
                        sockets: &mut *robot,
                        chip: &mut *chip,
                    }));
                }
                Ok(Ok(records)) if !ndef::asks_to_pair(&records) => {
                    match ModeTag::in_records(&records) {
                        Some(mode) => {
                            tracing::warn!(tag = %id, mode = mode.text(), "mode tag touched");
                            report_mode(mode, modes::on_touch(robot, mode));
                        }
                        None => {
                            tracing::warn!(tag = %id, ?records, "neither a pairing nor a mode tag")
                        }
                    }
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

        continue_after(period, start);
    }
}

/// The rest of this poll's period.
fn continue_after(period: Duration, start: Instant) {
    if let Some(rest) = period.checked_sub(start.elapsed()) {
        sleep(rest);
    }
}

/// The tag's NDEF records, or why its memory holds none (the inner error); the outer error is
/// the tag or the reader failing to read at all.
fn read_ndef(
    chip: &mut Chip,
    uid: &[u8],
    tech: Tech,
) -> nfc::Result<Result<Vec<ndef::Record>, String>> {
    let data = match tech {
        Tech::A => tag::read_ndef_area(chip),
        Tech::Classic => Err(Error::Unreadable(
            "a MIFARE Classic card: not read here — use an NTAG (Type 2) or a Type 5 tag".into(),
        )),
        Tech::V => vtag::read_ndef_area(chip, uid),
        Tech::Eink => Err(Error::Unreadable(
            "an e-ink display: it holds a picture, not NDEF".into(),
        )),
    }?;
    Ok(ndef::parse(&data))
}

/// A `nfc.write` or `nfc.read` waiting for a tag.
struct Job {
    ask: Ask,
    until: Instant,
}

/// What a [`Job`] does with the tag, and where its answer goes.
enum Ask {
    Write {
        text: String,
        answer: mpsc::Sender<proto::NfcWriteResult>,
    },
    Read {
        answer: mpsc::Sender<proto::NfcReadResult>,
    },
}

/// The jobs asked for, in order, and the one waiting for a tag now.
struct Jobs {
    asked: mpsc::Receiver<Job>,
    waiting: Option<Job>,
}

impl Jobs {
    /// Take up the next job asked for, answering any whose time ran out with no tag.
    fn expire(&mut self) {
        loop {
            if self.waiting.is_none() {
                self.waiting = self.asked.try_recv().ok();
            }
            let Some(job) = self.waiting.take_if(|job| Instant::now() >= job.until) else {
                return;
            };
            let reason = "no tag touched in time".to_owned();
            match job.ask {
                Ask::Write { text, answer } => {
                    tracing::warn!(%text, "no tag touched to write");
                    let _ = answer.send(not_written(None, reason));
                }
                Ask::Read { answer } => {
                    tracing::warn!("no tag touched to read");
                    let _ = answer.send(not_read(None, reason));
                }
            }
        }
    }

    fn take(&mut self) -> Option<Job> {
        self.waiting.take()
    }

    /// Put a job back to wait for the next poll, ahead of any asked after it.
    fn again(&mut self, job: Job) {
        self.waiting = Some(job);
    }
}

fn not_read(uid: Option<String>, reason: String) -> proto::NfcReadResult {
    proto::NfcReadResult {
        read: false,
        uid,
        records: Vec::new(),
        reason: Some(reason),
    }
}

/// Read the tag just selected for a `nfc.read`, quacking when it was. A tag that answered wrongly
/// is the caller's to retry; the answer is everything else, a blank tag included.
fn read_touched(
    chip: &mut Chip,
    robot: &mut dyn Robot,
    uid: &[u8],
    tech: Tech,
) -> nfc::Result<proto::NfcReadResult> {
    let id = hex(uid);
    let parsed = read_ndef(chip, uid, tech)?;
    if let Err(why) = robot.sound(pairing::HEARD) {
        tracing::info!(%why, "the robot would not quack for the tag");
    }
    Ok(match parsed {
        Ok(records) => {
            tracing::warn!(tag = %id, ?records, "tag read");
            proto::NfcReadResult {
                read: true,
                uid: Some(id),
                records: records
                    .into_iter()
                    .map(|record| match record {
                        ndef::Record::Text(s) => proto::NfcRecord::Text(s),
                        ndef::Record::Uri(s) => proto::NfcRecord::Uri(s),
                    })
                    .collect(),
                reason: None,
            }
        }
        Err(why) => {
            tracing::warn!(tag = %id, %why, "tag read; it holds no readable NDEF");
            proto::NfcReadResult {
                read: true,
                uid: Some(id),
                records: Vec::new(),
                reason: Some(format!("no readable NDEF: {why}")),
            }
        }
    })
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
    text: String,
    answer: mpsc::Sender<proto::NfcWriteResult>,
) -> Option<Error> {
    let id = hex(uid);
    let written = ndef::text_tlv(&text)
        .map_err(Error::Tag)
        .and_then(|data| match tech {
            Tech::A => tag::write_ndef_area(chip, &data),
            Tech::V => vtag::write_ndef_area(chip, uid, &data),
            Tech::Classic => Err(Error::Tag(
                "a MIFARE Classic card: only NTAG and Type 5 tags are written".into(),
            )),
            Tech::Eink => Err(Error::Tag(
                "an e-ink display: only NTAG and Type 5 tags are written".into(),
            )),
        });
    match written {
        Ok(()) => {
            tracing::warn!(tag = %id, %text, "tag written");
            if let Err(why) = robot.sound(pairing::HEARD) {
                tracing::info!(%why, "the robot would not quack for the tag");
            }
            let _ = answer.send(proto::NfcWriteResult {
                written: true,
                uid: Some(id),
                reason: None,
            });
            None
        }
        Err(e) => {
            tracing::warn!(tag = %id, %text, error = %e, "tag not written");
            let _ = answer.send(not_written(Some(id), e.to_string()));
            (!e.is_tag()).then_some(e)
        }
    }
}

/// Serve `nfc.write` and `nfc.read` on `socket`, on threads of its own: each asker waits for its tag, and the
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
                    Ok(proto::Call::NfcRead(params)) => {
                        proto::Response::ok(id, &read_when_touched(params, asked, reader_up, robot))
                    }
                    Ok(_) => proto::Response::err(
                        id,
                        proto::Error::new(
                            proto::code::METHOD_NOT_FOUND,
                            "nfcd serves nfc.write and nfc.read, and nothing else",
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
        ask: Ask::Write {
            text: params.text,
            answer,
        },
        until: Instant::now() + wait,
    };
    // Before the job is handed over, so a tag already on the reader is asked for before its quack
    // says it took.
    if let Err(why) = Robot::sound(robot, WAITING) {
        tracing::info!(%why, "the robot would not quack to ask for a tag");
    }
    if asked.send(job).is_err() {
        return not_written(None, "nfcd is stopping".into());
    }
    answered
        .recv_timeout(wait + ANSWER_MARGIN)
        .unwrap_or_else(|_| not_written(None, "no answer from the reader in time".into()))
}

/// Hand a read to the poll loop and wait for its answer.
fn read_when_touched(
    params: proto::NfcReadParams,
    asked: &mpsc::Sender<Job>,
    reader_up: &AtomicBool,
    robot: &mut Sockets,
) -> proto::NfcReadResult {
    if !reader_up.load(Ordering::Acquire) {
        return not_read(None, "no NFC reader answering on this robot".into());
    }
    let wait = params.timeout();
    let (answer, answered) = mpsc::channel();
    tracing::warn!(seconds = wait.as_secs(), "touch a tag to read it");
    let job = Job {
        ask: Ask::Read { answer },
        until: Instant::now() + wait,
    };
    // As for a write: the ask is heard before a tag already on the reader is.
    if let Err(why) = Robot::sound(robot, WAITING) {
        tracing::info!(%why, "the robot would not quack to ask for a tag");
    }
    if asked.send(job).is_err() {
        return not_read(None, "nfcd is stopping".into());
    }
    answered
        .recv_timeout(wait + ANSWER_MARGIN)
        .unwrap_or_else(|_| not_read(None, "no answer from the reader in time".into()))
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

fn report_photo(outcome: photo::Outcome) {
    match outcome {
        photo::Outcome::Taken(path) => {
            tracing::warn!(photo = %path.display(), "photo taken — sitting, beak open")
        }
        photo::Outcome::NoPhoto(why) => {
            tracing::error!(%why, "no photo — sitting, beak open, all the same")
        }
        photo::Outcome::NotStanding(policy) => {
            tracing::warn!(policy, "the robot did not stand, or is busy — no photo")
        }
        photo::Outcome::Failed(why) => tracing::error!(%why, "the photo sequence stopped"),
    }
}

fn report_mode(tag: ModeTag, outcome: modes::Outcome) {
    let tag = tag.text();
    match outcome {
        modes::Outcome::Switched(mode, switch) => {
            let on = switch.map(modes::Switch::name);
            tracing::warn!(tag, mode, on, "mode switched")
        }
        modes::Outcome::Refused(mode, why) => {
            tracing::warn!(tag, mode, %why, "the robot refused the mode")
        }
        modes::Outcome::Failed(why) => tracing::error!(tag, %why, "could not ask the robot"),
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
    media: PathBuf,
    photos: PathBuf,
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

impl Driver for Sockets {
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String> {
        Robot::sound(self, tag)
    }

    fn set_mode(&mut self, mode: &str) -> Result<proto::IntentResult, String> {
        let params = proto::SetModeParams { mode: mode.into() };
        call(
            &self.robot,
            &proto::Call::RobotSetMode(params),
            QUICK_ANSWER,
        )
    }

    fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String> {
        let params = proto::LaserTrackParams { active };
        call(
            &self.robot,
            &proto::Call::RobotLaserTrack(params),
            QUICK_ANSWER,
        )
    }

    fn soccer(&mut self, active: bool) -> Result<proto::IntentResult, String> {
        let params = proto::SoccerParams {
            active,
            autonomous: false,
        };
        call(&self.robot, &proto::Call::RobotSoccer(params), QUICK_ANSWER)
    }

    fn autonomous(&mut self, active: bool) -> Result<proto::IntentResult, String> {
        let params = proto::AutonomyParams { active };
        call(
            &self.robot,
            &proto::Call::RobotAutonomous(params),
            QUICK_ANSWER,
        )
    }
}

/// A photo sequence's view of the robot: the sockets, and the reader, which it asks for the display
/// in the beak.
struct Session<'a> {
    sockets: &'a mut Sockets,
    chip: &'a mut Chip,
}

impl photo::Driver for Session<'_> {
    fn sound(&mut self, tag: proto::SoundTag) -> Result<(), String> {
        Robot::sound(&mut *self.sockets, tag)
    }

    fn laser_track(&mut self, active: bool) -> Result<proto::LaserTrackResult, String> {
        Driver::laser_track(&mut *self.sockets, active)
    }

    fn stop(&mut self) -> Result<proto::IntentResult, String> {
        call(&self.sockets.robot, &proto::Call::RobotStop, QUICK_ANSWER)
    }

    fn policy(&mut self) -> Result<String, String> {
        state_policy(&self.sockets.robot, QUICK_ANSWER)
    }

    fn start(&mut self) -> Result<proto::IntentResult, String> {
        let params = proto::EnableParams {
            on: true,
            toggle: false,
        };
        call(
            &self.sockets.robot,
            &proto::Call::RobotEnable(params),
            QUICK_ANSWER,
        )
    }

    fn sit_toggle(&mut self) -> Result<proto::IntentResult, String> {
        let params = proto::DoParams {
            skill: "sit_toggle".into(),
        };
        call(
            &self.sockets.robot,
            &proto::Call::RobotDo(params),
            QUICK_ANSWER,
        )
    }

    fn look(&mut self, at: proto::LookParams) -> Result<proto::LookResult, String> {
        call(
            &self.sockets.robot,
            &proto::Call::RobotLook(at),
            QUICK_ANSWER,
        )
    }

    fn head(&mut self, head: proto::HeadParams) -> Result<(), String> {
        notify(&self.sockets.robot, &proto::Call::RobotHead(head))
    }

    fn flashlight(&mut self, brightness: f64) -> Result<proto::IntentResult, String> {
        let params = proto::FlashlightParams {
            on: brightness > 0.0,
            toggle: false,
            color: proto::FlashlightColor::White,
            brightness: (brightness > 0.0).then_some(brightness),
        };
        call(
            &self.sockets.robot,
            &proto::Call::RobotFlashlight(params),
            QUICK_ANSWER,
        )
    }

    fn mouth(&mut self, open: f64) -> Result<(), String> {
        notify(
            &self.sockets.robot,
            &proto::Call::RobotMouth(proto::MouthParams { open }),
        )
    }

    fn capture(&mut self) -> Result<Vec<u8>, String> {
        photo::capture(&self.sockets.media, QUICK_ANSWER)
    }

    fn eink_in_beak(&mut self) -> Result<bool, String> {
        self.chip.eink_at(BEAK).map_err(|e| e.to_string())
    }

    fn save(&mut self, jpeg: &[u8]) -> Result<PathBuf, String> {
        let name = photo::local_name(std::time::SystemTime::now());
        photo::save(&self.sockets.photos, &name, jpeg)
    }

    fn sleep(&mut self, d: Duration) {
        sleep(d);
    }
}

/// A continuous intent, sent as the notification it is: no answer to wait for.
fn notify(socket: &Path, call: &proto::Call) -> Result<(), String> {
    let fail = |e: std::io::Error| format!("{}: {e}", socket.display());
    let mut stream = UnixStream::connect(socket).map_err(fail)?;
    let mut request = proto::Request::call(proto::Id::Number(1), call);
    request.id = None;
    let mut line = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
    line.push(b'\n');
    stream.write_all(&line).map_err(fail)
}

/// The `policy` of the first `robot.state` frame on a `robot.subscribe` connection. Read as a bare
/// field rather than the whole `RobotState`, so a frame from another version still answers.
fn state_policy(socket: &Path, answer_within: Duration) -> Result<String, String> {
    let fail = |e: std::io::Error| format!("{}: {e}", socket.display());
    let mut stream = UnixStream::connect(socket).map_err(fail)?;
    stream.set_read_timeout(Some(answer_within)).map_err(fail)?;
    let subscribe = proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(10) });
    let request = proto::Request::call(proto::Id::Number(1), &subscribe);
    let mut line = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
    line.push(b'\n');
    stream.write_all(&line).map_err(fail)?;
    let deadline = Instant::now() + answer_within;
    let mut lines = BufReader::new(stream).lines();
    while Instant::now() < deadline {
        let line = lines
            .next()
            .ok_or("robotd closed the subscription")?
            .map_err(fail)?;
        let message: serde_json::Value =
            serde_json::from_str(&line).map_err(|e| format!("unparsable frame: {e}"))?;
        if let Some(error) = message.get("error") {
            return Err(error["message"].as_str().unwrap_or("refused").to_owned());
        }
        if message["method"] == proto::method::ROBOT_STATE {
            return message["params"]["policy"]
                .as_str()
                .map(str::to_owned)
                .ok_or_else(|| "a robot.state frame without a policy".into());
        }
    }
    Err("no robot.state frame".into())
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
