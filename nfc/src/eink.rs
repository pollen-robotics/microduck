//! An e-ink display touched to the reader: what starts the photo sequence, and what the photo is
//! written to.
//!
//! The displays are cards that speak ISO 14443-4 (NFC Forum Type 4, "IsoDep" on a phone), and say
//! so in the SAK they select with: bit 6, `20`. The ones seen so far answer `28` — ISO 14443-4 and
//! a MIFARE Classic 1K emulation — with ATQA `0004`. Every display of one model has the same chip,
//! so the same SAK, which is what lets any of them start the sequence with nothing written on it.
//!
//! **The SAK is the display's kind, not its identity.** Anything else that speaks ISO 14443-4
//! selects the same way: a bank card, a transit card, a phone emulating a card. Only a display
//! answers the application select [`Writer`] starts with, so a write tells them apart — the
//! touch that starts the sequence does not ask.
//!
//! [`is_eink`] is checked before [`crate::tag::is_mifare_classic`], because a `28` also has the
//! Classic bit: taken for a Classic, a display would be turned away as unreadable.
//!
//! ## Writing a picture
//!
//! The model is FMSC's `E701_CA`, 1.54" and 240 × 240 in six inks. Its commands are the vendor's,
//! as `nfc-test` speaks them (microduck-test, `nfc/nfc-test.c` at `f0c591f`, which is where every
//! number below was measured), each an APDU over [`crate::isodep`]:
//!
//! | APDU                          | what it does                                               |
//! |-------------------------------|------------------------------------------------------------|
//! | `00 A4 04 00 07 D2760000850101` | selects the display's application                        |
//! | `F0 D2 i k n` + `n` bytes     | packet `k` of image `i`: four images of 7 200 bytes ([`picture::PLANE`]), 250 bytes a packet |
//! | `F0 D4 P1 80 00`              | redraws the panel; `P1` × 100 ms of power-up first          |
//! | `F0 DE 00 00 01`              | still drawing? `00 90 00` once done                         |
//!
//! The picture itself is [`picture`]'s.
//!
//! Most of the work is getting through a weak field. A display draws its power from the reader,
//! and the redraw needs far more than the radio does: it starts answering late — the reader gives
//! it `FWI` 14, 4.9 s, for every block rather than the 309 ms its ATS asks for — and then mid-way
//! drops out altogether, browned out, and must be selected again with the field kept on, or the
//! redraw it is powering stops. A packet that fails is sent again, the display re-selected every
//! fifth failure; a packet that took more than [`SLOW`] slows the pace between packets, which lets
//! the display's reservoir fill.
//!
//! **One step at a time.** [`Writer::step`] does one exchange — a packet, or one ask whether the
//! redraw is done — and returns, so its caller can do something between two: the photo sequence
//! keeps the robot's head where it is, which a write held for the 20-odd seconds a redraw takes
//! would not let it.

pub mod picture;

use std::time::{Duration, Instant};

use crate::isodep::IsoDep;
use crate::reader::Reader;
use crate::{Error, Result, tag};

/// Whether a SAK says ISO 14443-4, which is how a display is told from a tag.
pub fn is_eink(sak: u8) -> bool {
    sak & 0x20 != 0
}

/// The display's application: the NFC Forum Type 4 one, `D2760000850101`.
const SELECT: [u8; 12] = [
    0x00, 0xa4, 0x04, 0x00, 0x07, 0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,
];
const BUSY: [u8; 5] = [0xf0, 0xde, 0x00, 0x00, 0x01];
/// The bytes of a picture one write packet carries.
pub const PACKET: usize = 250;

/// How many times the application select is asked before the display is taken to be something
/// else, and the pause between two.
const SELECT_TRIES: u32 = 10;
const SELECT_AGAIN: Duration = Duration::from_millis(50);
/// Unanswered selects in a row before the display is selected again at the ISO 14443 level.
const SELECT_REJOIN_EVERY: u32 = 3;
/// How many times one packet is sent before the write fails, and every how many the display is
/// selected again.
const PACKET_TRIES: u32 = 40;
const REJOIN_EVERY: u32 = 5;
/// A packet taking longer than this is a display short of power.
pub const SLOW: Duration = Duration::from_millis(150);
/// The pace a slow packet starts, doubled by each slow one after, to at most [`PACE_MAX`].
const PACE_MIN: Duration = Duration::from_millis(5);
const PACE_MAX: Duration = Duration::from_millis(50);
/// Between two asks whether the redraw is done; after a failed ask, the first few times.
const POLL_EVERY: Duration = Duration::from_millis(500);
const POLL_AGAIN: Duration = Duration::from_millis(300);
/// Failed asks in a row that are waited out before the display is selected again.
const POLL_ERRORS: u32 = 5;
/// Between two attempts to select a display that dropped out.
const REJOIN_AGAIN: Duration = Duration::from_millis(50);

/// What [`Writer`] needs from the display: an APDU and its answer, and a way back in after it
/// stopped answering. [`Link`] is the one on a reader; the tests have theirs.
pub trait Card {
    /// One APDU, and the display's answer, status word included.
    fn apdu(&mut self, apdu: &[u8]) -> Result<Vec<u8>>;
    /// Select the display again, with the field left on.
    fn rejoin(&mut self) -> Result<()>;
}

/// How [`Writer`] writes, where the caller has a say.
#[derive(Debug, Clone)]
pub struct Settings {
    /// `F0 D4`'s `P1`: the panel's power-up before it draws, in 100 ms. The vendor's apps send
    /// `85`; `FF` is the longest, which a weak field may need.
    pub refresh_p1: u8,
    /// Between two packets, at first. A slow packet raises it.
    pub pace: Duration,
    /// How long the redraw may take, from the refresh command to its end.
    pub refresh_within: Duration,
    /// How long a display that dropped out has to answer again.
    pub rejoin_within: Duration,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            refresh_p1: 0x85,
            pace: Duration::ZERO,
            refresh_within: Duration::from_secs(300),
            rejoin_within: Duration::from_secs(60),
        }
    }
}

/// Where a write is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress {
    /// Selecting the display's application.
    Selecting,
    /// `sent` bytes of the picture's [`picture::BYTES`] are on the display.
    Writing { sent: usize },
    /// The display is drawing.
    Refreshing,
    /// The picture is on the panel.
    Done,
}

#[derive(Debug, Clone)]
enum State {
    Select {
        tries: u32,
    },
    Write {
        plane: usize,
        packet: usize,
        attempts: u32,
    },
    Refresh,
    Poll {
        until: Instant,
        errors: u32,
    },
    /// The display stopped answering: select it again, then carry on where it was.
    Rejoin {
        then: Box<State>,
        select: bool,
        until: Instant,
    },
    Done,
}

/// A picture being written to a display, one exchange per [`Writer::step`].
#[derive(Debug)]
pub struct Writer {
    picture: Vec<u8>,
    settings: Settings,
    pace: Duration,
    state: State,
    not_before: Option<Instant>,
    failed: Option<String>,
    /// When the first packet went, and the refresh command.
    wrote_from: Option<Instant>,
    drew_from: Option<Instant>,
    polls: u32,
    /// The last answer that said the display is still drawing, so a new one is logged once.
    drawing: Option<Vec<u8>>,
}

impl Writer {
    /// A write of `picture`, [`picture::BYTES`] of it as [`picture::render`] packs them.
    pub fn new(picture: Vec<u8>, settings: Settings) -> Self {
        assert_eq!(picture.len(), picture::BYTES, "a packed picture");
        Writer {
            picture,
            pace: settings.pace,
            settings,
            state: State::Select { tries: 0 },
            not_before: None,
            failed: None,
            wrote_from: None,
            drew_from: None,
            polls: 0,
            drawing: None,
        }
    }

    /// Where the write is.
    pub fn progress(&self) -> Progress {
        progress(&self.state)
    }

    /// How long from `now` until [`Writer::step`] has something to do: a pause between packets,
    /// or between two asks during the redraw.
    pub fn idle(&self, now: Instant) -> Duration {
        self.not_before
            .map_or(Duration::ZERO, |at| at.saturating_duration_since(now))
    }

    /// One exchange with the display, unless it is not yet time for the next; where the write is
    /// then. A write that failed says why, and goes on saying it.
    pub fn step(
        &mut self,
        card: &mut dyn Card,
        now: Instant,
    ) -> std::result::Result<Progress, String> {
        if let Some(why) = &self.failed {
            return Err(why.clone());
        }
        if self.not_before.is_some_and(|at| now < at) {
            return Ok(self.progress());
        }
        self.not_before = None;
        let state = std::mem::replace(&mut self.state, State::Done);
        match self.advance(state, card, now) {
            Ok(next) => {
                self.state = next;
                Ok(self.progress())
            }
            Err(why) => {
                self.failed = Some(why.clone());
                Err(why)
            }
        }
    }

    fn wait(&mut self, now: Instant, d: Duration) {
        if !d.is_zero() {
            self.not_before = Some(now + d);
        }
    }

    fn advance(
        &mut self,
        state: State,
        card: &mut dyn Card,
        now: Instant,
    ) -> std::result::Result<State, String> {
        Ok(match state {
            State::Select { tries } => match card.apdu(&SELECT) {
                Ok(answer) if sw_ok(&answer) => State::Write {
                    plane: 0,
                    packet: 0,
                    attempts: 0,
                },
                answer => {
                    let tries = tries + 1;
                    if tries >= SELECT_TRIES {
                        let what = if answer.is_ok() {
                            "not a display, or not one this speaks: its application select answered"
                        } else {
                            "the display stopped answering:"
                        };
                        return Err(format!("{what} {}", said(&answer)));
                    }
                    // No answer is a display out of reach, not a wrong one: selected again every
                    // few, as a packet that goes unanswered is.
                    if answer.is_err() && tries.is_multiple_of(SELECT_REJOIN_EVERY) {
                        tracing::warn!(
                            tries,
                            "the display does not answer the select; selecting it again"
                        );
                        State::Rejoin {
                            then: Box::new(State::Select { tries }),
                            select: false,
                            until: now + self.settings.rejoin_within,
                        }
                    } else {
                        self.wait(now, SELECT_AGAIN);
                        State::Select { tries }
                    }
                }
            },
            State::Write {
                plane,
                packet,
                attempts,
            } => self.write(card, now, plane, packet, attempts)?,
            State::Refresh => {
                let p1 = self.settings.refresh_p1;
                match card.apdu(&[0xf0, 0xd4, p1, 0x80, 0x00]) {
                    // The display answers `90 00`, or a status byte and `90 00`: started, either.
                    Ok(answer) if refresh_started(&answer) => {}
                    Ok(answer) => {
                        return Err(format!("the display refused the refresh: {}", hex(&answer)));
                    }
                    // Often no answer at all: the redraw took the power the answer needed. Asked.
                    Err(e) => {
                        tracing::info!(error = %e, "no answer to the refresh; asking whether it is drawing")
                    }
                }
                tracing::info!(
                    p1 = format!("{p1:02x}"),
                    "refreshing: keep the display still"
                );
                self.drew_from = Some(now);
                State::Poll {
                    until: now + self.settings.refresh_within,
                    errors: 0,
                }
            }
            State::Poll { until, errors } => {
                if now >= until {
                    return Err(format!(
                        "the display was still drawing after {:?}",
                        self.settings.refresh_within
                    ));
                }
                self.polls += 1;
                // Only a `90 00` is an answer about the redraw: anything else — a display that
                // browned out and lost its application, say — is as good as none.
                let asked = card.apdu(&BUSY).and_then(|answer| {
                    if sw_ok(&answer) {
                        Ok(answer)
                    } else {
                        Err(Error::Tag(format!("it answered {}", hex(&answer))))
                    }
                });
                match asked {
                    Ok(answer) if answer.len() >= 3 && answer[0] == 0x00 => {
                        let took = self.drew_from.map(|at| now.saturating_duration_since(at));
                        tracing::info!(polls = self.polls, ?took, "refresh done");
                        State::Done
                    }
                    Ok(answer) => {
                        if self.drawing.as_ref() != Some(&answer) {
                            tracing::info!(answer = hex(&answer), "drawing");
                            self.drawing = Some(answer);
                        }
                        self.wait(now, POLL_EVERY);
                        State::Poll { until, errors: 0 }
                    }
                    Err(e) => {
                        let errors = errors + 1;
                        if errors <= POLL_ERRORS {
                            tracing::debug!(error = %e, errors, "no answer while drawing");
                            self.wait(now, POLL_AGAIN);
                            State::Poll { until, errors }
                        } else {
                            tracing::warn!(error = %e, "the display stopped answering while drawing; selecting it again");
                            // Its application selected again too: one that browned out has
                            // forgotten it, and answers nothing else.
                            State::Rejoin {
                                then: Box::new(State::Poll { until, errors: 0 }),
                                select: true,
                                until,
                            }
                        }
                    }
                }
            }
            State::Rejoin {
                then,
                select,
                until,
            } => {
                if now >= until {
                    return Err("the display did not answer again".into());
                }
                let back = card.rejoin().and_then(|()| {
                    if !select {
                        return Ok(());
                    }
                    let answer = card.apdu(&SELECT)?;
                    if sw_ok(&answer) {
                        Ok(())
                    } else {
                        Err(Error::Tag(format!(
                            "application select answered {}",
                            hex(&answer)
                        )))
                    }
                });
                match back {
                    Ok(()) => {
                        tracing::info!("the display is back");
                        *then
                    }
                    Err(e) => {
                        tracing::debug!(error = %e, "the display is not back yet");
                        self.wait(now, REJOIN_AGAIN);
                        State::Rejoin {
                            then,
                            select,
                            until,
                        }
                    }
                }
            }
            State::Done => State::Done,
        })
    }

    fn write(
        &mut self,
        card: &mut dyn Card,
        now: Instant,
        plane: usize,
        packet: usize,
        attempts: u32,
    ) -> std::result::Result<State, String> {
        let at = plane * picture::PLANE + packet * PACKET;
        let end =
            (plane * picture::PLANE + (packet + 1) * PACKET).min((plane + 1) * picture::PLANE);
        let data = &self.picture[at..end];
        let mut apdu = Vec::with_capacity(5 + data.len());
        apdu.extend([0xf0, 0xd2, plane as u8, packet as u8, data.len() as u8]);
        apdu.extend_from_slice(data);

        let sent = Instant::now();
        let answer = card.apdu(&apdu);
        let took = sent.elapsed();
        let ok = answer.as_ref().is_ok_and(|a| sw_ok(a));
        if took > SLOW {
            // Short of power: give the display longer between packets.
            self.pace = (self.pace * 2).clamp(PACE_MIN, PACE_MAX);
            tracing::info!(plane, packet, ?took, ok, pace = ?self.pace, "slow packet");
        }
        if !ok {
            let attempts = attempts + 1;
            if attempts >= PACKET_TRIES {
                return Err(format!(
                    "packet {packet} of image {plane} failed {attempts} times: {}",
                    said(&answer)
                ));
            }
            let again = State::Write {
                plane,
                packet,
                attempts,
            };
            if attempts.is_multiple_of(REJOIN_EVERY) {
                tracing::warn!(
                    plane,
                    packet,
                    "the display stopped taking packets; selecting it again"
                );
                return Ok(State::Rejoin {
                    then: Box::new(again),
                    select: true,
                    until: now + self.settings.rejoin_within,
                });
            }
            return Ok(again);
        }

        self.wrote_from.get_or_insert(now);
        self.wait(now, self.pace);
        if end < (plane + 1) * picture::PLANE {
            return Ok(State::Write {
                plane,
                packet: packet + 1,
                attempts: 0,
            });
        }
        tracing::info!(plane, packets = packet + 1, "image written");
        if plane + 1 < picture::BYTES / picture::PLANE {
            return Ok(State::Write {
                plane: plane + 1,
                packet: 0,
                attempts: 0,
            });
        }
        let took = self.wrote_from.map(|at| now.saturating_duration_since(at));
        tracing::info!(bytes = picture::BYTES, ?took, "picture written");
        Ok(State::Refresh)
    }
}

fn progress(state: &State) -> Progress {
    match state {
        State::Select { .. } => Progress::Selecting,
        State::Write { plane, packet, .. } => Progress::Writing {
            sent: plane * picture::PLANE + packet * PACKET,
        },
        State::Refresh | State::Poll { .. } => Progress::Refreshing,
        State::Rejoin { then, .. } => progress(then),
        State::Done => Progress::Done,
    }
}

fn sw_ok(answer: &[u8]) -> bool {
    answer.ends_with(&[0x90, 0x00])
}

fn refresh_started(answer: &[u8]) -> bool {
    match answer {
        [0x90, 0x00] => true,
        [status, 0x90, 0x00] => *status <= 0x01,
        _ => false,
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn said(answer: &Result<Vec<u8>>) -> String {
    match answer {
        Ok(bytes) => hex(bytes),
        Err(e) => e.to_string(),
    }
}

/// A display selected and in ISO 14443-4, every block given [`crate::isodep::fwt`] of `fwi`. It
/// holds no reader, so it can be kept between two [`Writer::step`]s while the reader is lent out
/// for each: [`Joined::on`] is the [`Card`] a step talks to.
pub struct Joined {
    dep: IsoDep,
    fwi: u8,
}

impl Joined {
    /// The display in the field, selected with the field reset first, as any tag is.
    pub fn open<R: Reader + ?Sized>(chip: &mut R, fwi: u8) -> Result<Self> {
        let found = tag::select_tag(chip)?;
        let dep = join(chip, found.sak, fwi)?;
        Ok(Joined { dep, fwi })
    }

    /// This display on `chip`, for one step.
    pub fn on<'a, R: Reader + ?Sized>(&'a mut self, chip: &'a mut R) -> Link<'a, R> {
        Link { chip, joined: self }
    }
}

/// A [`Joined`] display and the reader it is on: the [`Card`] [`Writer`] talks to.
pub struct Link<'a, R: Reader + ?Sized> {
    chip: &'a mut R,
    joined: &'a mut Joined,
}

fn join<R: Reader + ?Sized>(chip: &mut R, sak: u8, fwi: u8) -> Result<IsoDep> {
    if !is_eink(sak) {
        return Err(Error::Tag(format!(
            "SAK {sak:02x}: not ISO 14443-4, not a display"
        )));
    }
    let (mut dep, _) = IsoDep::activate(chip)?;
    dep.set_fwi(fwi);
    Ok(dep)
}

impl<R: Reader + ?Sized> Card for Link<'_, R> {
    fn apdu(&mut self, apdu: &[u8]) -> Result<Vec<u8>> {
        self.joined.dep.exchange(&mut *self.chip, apdu)
    }

    fn rejoin(&mut self) -> Result<()> {
        // A display that browned out has forgotten the session and will not answer this; one that
        // only lost a frame goes to HALT, which the WUPA wakes it from.
        let _ = self.joined.dep.deselect(&mut *self.chip);
        let found = tag::select_in_field(&mut *self.chip)?;
        self.joined.dep = join(&mut *self.chip, found.sak, self.joined.fwi)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[test]
    fn a_display_is_iso_14443_4_and_a_tag_is_not() {
        // 28: the display seen so far. 20: ISO 14443-4 alone.
        for sak in [0x28, 0x20] {
            assert!(is_eink(sak), "{sak:#x}");
        }
        // An NTAG, the Classics.
        for sak in [0x00, 0x08, 0x18, 0x09, 0x88] {
            assert!(!is_eink(sak), "{sak:#x}");
        }
    }

    /// A display as a script: an answer for each APDU, or by default `90 00`; and what it got.
    #[derive(Default)]
    struct Display {
        got: Vec<Vec<u8>>,
        rejoins: usize,
        answers: VecDeque<Result<Vec<u8>>>,
        /// After the refresh command, how many asks answer "drawing" before "done".
        drawing: usize,
    }

    impl Card for Display {
        fn apdu(&mut self, apdu: &[u8]) -> Result<Vec<u8>> {
            self.got.push(apdu.to_vec());
            if let Some(answer) = self.answers.pop_front() {
                return answer;
            }
            if apdu == BUSY {
                if self.drawing > 0 {
                    self.drawing -= 1;
                    return Ok(vec![0x01, 0x90, 0x00]);
                }
                return Ok(vec![0x00, 0x90, 0x00]);
            }
            Ok(vec![0x90, 0x00])
        }

        fn rejoin(&mut self) -> Result<()> {
            self.rejoins += 1;
            Ok(())
        }
    }

    fn picture() -> Vec<u8> {
        (0..picture::BYTES).map(|i| (i % 251) as u8).collect()
    }

    /// Steps until the write is done or failed, the clock moved on by whatever it waits.
    fn run(writer: &mut Writer, display: &mut Display) -> std::result::Result<Instant, String> {
        let mut now = Instant::now();
        for _ in 0..10_000 {
            if writer.step(display, now)? == Progress::Done {
                return Ok(now);
            }
            now += writer.idle(now).max(Duration::from_millis(1));
        }
        panic!("never done");
    }

    /// Steps until the refresh command has gone.
    fn until_refresh_sent(writer: &mut Writer, display: &mut Display, now: Instant) {
        while display.got.last().is_none_or(|a| a[..2] != [0xf0, 0xd4]) {
            writer.step(display, now).unwrap();
        }
    }

    #[test]
    fn a_write_is_select_four_images_of_29_packets_refresh_and_asks() {
        let mut display = Display {
            drawing: 3,
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        run(&mut writer, &mut display).unwrap();
        let got = &display.got;
        assert_eq!(got[0], SELECT);
        let packets: Vec<_> = got.iter().filter(|a| a[..2] == [0xf0, 0xd2]).collect();
        assert_eq!(packets.len(), 4 * 29);
        assert_eq!(packets[0][..5], [0xf0, 0xd2, 0, 0, 250]);
        assert_eq!(&packets[0][5..], &picture()[..250]);
        // The 29th packet of an image is its last 200 bytes; the next image starts at packet 0.
        assert_eq!(packets[28][..5], [0xf0, 0xd2, 0, 28, 200]);
        assert_eq!(packets[29][..5], [0xf0, 0xd2, 1, 0, 250]);
        assert_eq!(&packets[29][5..], &picture()[7200..7450]);
        let last = packets.last().unwrap();
        assert_eq!(&last[5..], &picture()[picture::BYTES - 200..]);
        let after: Vec<_> = got.iter().skip(1 + 4 * 29).collect();
        assert_eq!(after[0], &vec![0xf0, 0xd4, 0x85, 0x80, 0x00]);
        assert_eq!(after.len(), 1 + 4, "three asks drawing, one done");
        assert!(after[1..].iter().all(|a| **a == BUSY));
    }

    #[test]
    fn the_asks_are_half_a_second_apart() {
        let mut display = Display {
            drawing: 4,
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        let mut now = Instant::now();
        until_refresh_sent(&mut writer, &mut display, now);
        // The refresh went; the first ask goes on the next step, then one every 500 ms.
        let from = now;
        while writer.step(&mut display, now).unwrap() != Progress::Done {
            now += writer.idle(now).max(Duration::from_millis(1));
        }
        assert_eq!(now - from, Duration::from_millis(2000));
    }

    #[test]
    fn something_that_is_not_a_display_fails_at_the_select() {
        let mut display = Display {
            answers: (0..10).map(|_| Ok(vec![0x6a, 0x82])).collect(),
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        let why = run(&mut writer, &mut display).unwrap_err();
        assert!(why.contains("6a82"), "{why}");
        assert_eq!(display.got.len(), 10);
        assert!(
            writer.step(&mut display, Instant::now()).is_err(),
            "and stays failed"
        );
    }

    #[test]
    fn an_unanswered_select_rejoins_the_display_and_a_refused_one_does_not() {
        let mut display = Display {
            answers: (0..4).map(|_| Err(Error::NoTag)).collect(),
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        run(&mut writer, &mut display).unwrap();
        assert_eq!(display.rejoins, 1);

        let mut display = Display {
            answers: (0..10).map(|_| Err(Error::NoTag)).collect(),
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        let why = run(&mut writer, &mut display).unwrap_err();
        assert!(why.contains("stopped answering"), "{why}");
        assert_eq!(display.rejoins, 3);
    }

    #[test]
    fn a_failing_packet_is_sent_again_and_the_display_rejoined_every_fifth_time() {
        let mut answers: VecDeque<_> = VecDeque::from([Ok(vec![0x90, 0x00])]); // the select
        answers.extend((0..5).map(|_| Err(Error::NoTag)));
        answers.push_back(Ok(vec![0x90, 0x00])); // the select after the rejoin
        answers.extend((0..2).map(|_| Err(Error::NoTag)));
        let mut display = Display {
            answers,
            ..Display::default()
        };
        let mut writer = Writer::new(picture(), Settings::default());
        run(&mut writer, &mut display).unwrap();
        assert_eq!(display.rejoins, 1);
        // Five failures, the rejoin's select, two more, then packet 0 goes.
        assert_eq!(display.got[1], display.got[9]);
        assert_eq!(display.got[6], SELECT, "selected again after the fifth");
    }

    #[test]
    fn a_display_that_drops_out_while_drawing_is_rejoined_and_its_application_selected() {
        let mut display = Display::default();
        let mut writer = Writer::new(picture(), Settings::default());
        let mut now = Instant::now();
        until_refresh_sent(&mut writer, &mut display, now);
        display.answers.extend((0..6).map(|_| Err(Error::NoTag)));
        let selects = display.got.iter().filter(|a| **a == SELECT).count();
        while writer.step(&mut display, now).unwrap() != Progress::Done {
            now += writer.idle(now).max(Duration::from_millis(1));
        }
        assert_eq!(display.rejoins, 1);
        assert_eq!(
            display.got.iter().filter(|a| **a == SELECT).count(),
            selects + 1
        );
    }

    #[test]
    fn no_answer_to_the_refresh_is_asked_about_and_a_refusal_fails() {
        let mut display = Display::default();
        display.answers.push_back(Ok(vec![0x90, 0x00]));
        display
            .answers
            .extend((0..116).map(|_| Ok(vec![0x90, 0x00])));
        display.answers.push_back(Err(Error::NoTag));
        let mut writer = Writer::new(picture(), Settings::default());
        run(&mut writer, &mut display).unwrap();

        let mut display = Display::default();
        display
            .answers
            .extend((0..117).map(|_| Ok(vec![0x90, 0x00])));
        display.answers.push_back(Ok(vec![0x6d, 0x00]));
        let mut writer = Writer::new(picture(), Settings::default());
        assert!(run(&mut writer, &mut display).unwrap_err().contains("6d00"));
    }

    /// After a brown-out the display was asked for five minutes and never said done. An error
    /// status is no answer, and the display is selected again.
    #[test]
    fn an_error_status_while_drawing_is_no_answer() {
        let mut display = Display::default();
        let mut writer = Writer::new(picture(), Settings::default());
        let mut now = Instant::now();
        until_refresh_sent(&mut writer, &mut display, now);
        display.answers.extend((0..6).map(|_| Ok(vec![0x6d, 0x00])));
        while writer.step(&mut display, now).unwrap() != Progress::Done {
            now += writer.idle(now).max(Duration::from_millis(1));
        }
        assert_eq!(display.rejoins, 1);
    }

    #[test]
    fn a_redraw_that_never_ends_fails_in_its_time() {
        let mut display = Display {
            drawing: usize::MAX,
            ..Display::default()
        };
        let settings = Settings {
            refresh_within: Duration::from_secs(3),
            ..Settings::default()
        };
        let mut writer = Writer::new(picture(), settings);
        assert!(
            run(&mut writer, &mut display)
                .unwrap_err()
                .contains("still drawing")
        );
    }

    #[test]
    fn the_refresh_answers_the_vendor_apps_accept() {
        assert!(refresh_started(&[0x90, 0x00]));
        assert!(refresh_started(&[0x00, 0x90, 0x00]));
        assert!(refresh_started(&[0x01, 0x90, 0x00]));
        assert!(!refresh_started(&[0x02, 0x90, 0x00]));
        assert!(!refresh_started(&[0x6d, 0x00]));
    }
}
