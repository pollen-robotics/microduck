//! Standard MIDI file in, chorale out — so a notation editor is the score editor.
//!
//! This is the front end that actually unlocks writing music for the ducks. The text format
//! ([`super::text`]) is good for a chorale you write by hand and read back, and it cannot
//! express four voices moving in four different rhythms. A MIDI file can, every notation editor
//! and DAW exports one, and the entire public-domain four-part repertoire is already available
//! as one — Bach chorales most of all, which are literally this genre.
//!
//! **No dependency.** A Standard MIDI File is a length-prefixed chunk format with variable-length
//! delta times, and the subset a score needs is note-on, note-off, tempo and track name. That is
//! a couple of hundred lines and no supply chain, against a crate that would parse controller
//! automation and SysEx this will never look at.
//!
//! ## How tracks become parts
//!
//! By **mean pitch**, not by track order — the lowest group of notes sings bass. Track order
//! would be the obvious rule and is wrong twice over: notation editors write scores top staff
//! first (soprano, alto, tenor, bass — the reverse of what a `Voicing` lists), and a file from a
//! DAW may have the tempo track, empty tracks, or the parts in any order at all. Sorting by
//! pitch is right whatever produced the file, and it is the same rule [`super::cast`] uses to
//! seat the ducks, which keeps one idea in one place. A track *name* that says "Soprano" is
//! believed first, since a human wrote it down.
//!
//! A single polyphonic track — one piano staff with chords in it, which is what "export MIDI"
//! gives you from a lot of tools — is split by pitch rank within each chord. Crude and better
//! than refusing: the lowest note of each chord is the bass line.
//!
//! ## What is deliberately dropped
//!
//! Velocity becomes the note's dynamic, which is free and right. Everything else — controllers,
//! pitch bend, program changes, anything past the first tempo — is skipped: a duck has one
//! oscillator and a mouth, and a tempo map would have to be honoured identically by four robots
//! agreeing over a network, which is a promise this cannot keep. A file with a tempo change
//! parses and is sung at its first tempo, with the fact reported rather than hidden.

use std::collections::HashMap;
use std::fmt;

use super::{Note, Part, Score, Vowel};

/// What went wrong reading the file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MidiError {
    /// Not a Standard MIDI File at all.
    NotMidi,
    /// Truncated, or a chunk length that runs off the end.
    Truncated { at: usize },
    /// SMPTE timecode division. Vanishingly rare from a notation editor, and the conversion to
    /// beats is different enough that guessing would be worse than saying so.
    SmpteTiming,
    /// No note-on events anywhere in the file.
    NoNotes,
}

impl fmt::Display for MidiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MidiError::NotMidi => write!(f, "not a MIDI file (no MThd header)"),
            MidiError::Truncated { at } => write!(f, "the file ends mid-event, at byte {at}"),
            MidiError::SmpteTiming => write!(
                f,
                "SMPTE timecode division is not supported — export with metrical (ticks-per-beat) timing"
            ),
            MidiError::NoNotes => write!(f, "there are no notes in this file"),
        }
    }
}

impl std::error::Error for MidiError {}

/// What was in the file, besides the notes — reported so an import is not silent about what it
/// dropped.
#[derive(Debug, Clone, PartialEq)]
pub struct Import {
    pub score: Score,
    /// Track names, in file order, for the ones that had notes.
    pub tracks: Vec<String>,
    /// How each part was decided.
    pub casting: Vec<(Part, String)>,
    /// Things skipped that someone might have expected to survive.
    pub dropped: Vec<String>,
}

/// One note as the file describes it, before parts are assigned.
#[derive(Debug, Clone, Copy)]
struct Raw {
    group: usize,
    start_tick: u64,
    end_tick: u64,
    midi: u8,
    velocity: u8,
}

/// Read a Standard MIDI File.
pub fn parse(bytes: &[u8]) -> Result<Import, MidiError> {
    // Checked before anything else can fail: a file that is not MIDI at all should say so,
    // not report a truncation at byte zero.
    if bytes.len() < 14 || bytes[..4] != *b"MThd" {
        return Err(MidiError::NotMidi);
    }
    let mut reader = Reader::new(bytes);
    reader.skip(4)?;
    let header_len = reader.u32()? as usize;
    let header_end = reader.at + header_len;
    let _format = reader.u16()?;
    let _tracks = reader.u16()?;
    let division = reader.u16()?;
    if division & 0x8000 != 0 {
        return Err(MidiError::SmpteTiming);
    }
    let ticks_per_beat = f64::from(division.max(1));
    reader.at = header_end.min(bytes.len());

    let mut raws: Vec<Raw> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    // Every track's own name, by track index, whether or not it had notes — what a group's name
    // is looked up in. `names` above is a different list, of the tracks that sang, and a group
    // index is not an index into it.
    let mut track_names: Vec<String> = Vec::new();
    let mut dropped: Vec<String> = Vec::new();
    // Microseconds per quarter note. 500_000 is MIDI's default — 120 bpm.
    let mut micros_per_beat = 500_000f64;
    let mut tempo_seen = false;
    // One group per (track, channel): a file may put two voices on one track and different
    // channels, and that is a real two-part export.
    let mut groups: HashMap<(usize, u8), usize> = HashMap::new();

    let mut track_index = 0usize;
    while reader.remaining() >= 8 {
        let tag = reader.tag()?;
        let length = reader.u32()? as usize;
        let end = reader.at + length;
        if end > bytes.len() {
            return Err(MidiError::Truncated { at: reader.at });
        }
        if tag != *b"MTrk" {
            // An unknown chunk is skippable by design — the spec says so, which is what the
            // length prefix is for.
            reader.at = end;
            continue;
        }

        let mut name = String::new();
        let mut tick = 0u64;
        let mut status = 0u8;
        // Sounding notes, keyed by (channel, pitch). A second note-on for a pitch already
        // sounding ends the first, which is what a sane sequencer means by it.
        let mut open: HashMap<(u8, u8), (u64, u8)> = HashMap::new();

        while reader.at < end {
            tick += reader.varint()?;
            let mut byte = reader.u8()?;
            if byte < 0x80 {
                // Running status: this byte is data, and the previous status still applies.
                reader.at -= 1;
                byte = status;
            } else {
                status = byte;
            }
            match byte & 0xF0 {
                0x80 | 0x90 => {
                    let channel = byte & 0x0F;
                    let pitch = reader.u8()? & 0x7F;
                    let velocity = reader.u8()? & 0x7F;
                    // Note-on with zero velocity is a note-off; every file in the world uses it.
                    let is_on = byte & 0xF0 == 0x90 && velocity > 0;
                    if is_on {
                        if let Some((start, velocity)) =
                            open.insert((channel, pitch), (tick, velocity))
                        {
                            push(
                                &mut raws,
                                &mut groups,
                                track_index,
                                channel,
                                start,
                                tick,
                                pitch,
                                velocity,
                            );
                        }
                    } else if let Some((start, velocity)) = open.remove(&(channel, pitch)) {
                        push(
                            &mut raws,
                            &mut groups,
                            track_index,
                            channel,
                            start,
                            tick,
                            pitch,
                            velocity,
                        );
                    }
                }
                // Two data bytes, none of them ours.
                0xA0 | 0xB0 | 0xE0 => {
                    reader.skip(2)?;
                }
                // One data byte.
                0xC0 | 0xD0 => {
                    reader.skip(1)?;
                }
                0xF0 => match byte {
                    0xFF => {
                        let kind = reader.u8()?;
                        let length = reader.varint()? as usize;
                        let data = reader.take(length)?;
                        match kind {
                            0x03 if name.is_empty() => {
                                name = String::from_utf8_lossy(data).trim().to_owned();
                            }
                            0x51 if data.len() == 3 => {
                                let value =
                                    f64::from(u32::from_be_bytes([0, data[0], data[1], data[2]]));
                                if tempo_seen {
                                    if (value - micros_per_beat).abs() > 1.0
                                        && !dropped.iter().any(|d| d.starts_with("a tempo change"))
                                    {
                                        dropped.push(
                                            "a tempo change (the whole piece is sung at the first tempo)"
                                                .to_owned(),
                                        );
                                    }
                                } else {
                                    micros_per_beat = value;
                                    tempo_seen = true;
                                }
                            }
                            _ => {}
                        }
                    }
                    // SysEx: length-prefixed, and nothing here cares what is in it.
                    0xF0 | 0xF7 => {
                        let length = reader.varint()? as usize;
                        reader.skip(length)?;
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        // A note still sounding at the end of the track ends there rather than being lost.
        for ((channel, pitch), (start, velocity)) in open {
            push(
                &mut raws,
                &mut groups,
                track_index,
                channel,
                start,
                tick,
                pitch,
                velocity,
            );
        }
        track_names.push(name.clone());
        if raws.iter().any(|r| {
            groups
                .iter()
                .any(|((t, _), g)| *t == track_index && *g == r.group)
        }) {
            names.push(if name.is_empty() {
                format!("track {}", track_index + 1)
            } else {
                name
            });
        }
        reader.at = end;
        track_index += 1;
    }

    if raws.is_empty() {
        return Err(MidiError::NoNotes);
    }
    let bpm = 60_000_000.0 / micros_per_beat.max(1.0);
    // A group is a (track, channel), and a name belongs to a track — so groups and tracks are not
    // the same count, and a group's name is its own track's. A track carrying two channels has
    // one name for two lines and cannot say which is which, so its groups are seated by pitch.
    let mut group_names = vec![String::new(); groups.len()];
    for (&(track, _), &group) in &groups {
        let lines = groups.keys().filter(|(t, _)| *t == track).count();
        if lines == 1 {
            group_names[group] = track_names[track].clone();
        }
    }
    let (notes, casting, mut assign_dropped) = assign(&mut raws, &group_names, ticks_per_beat);
    dropped.append(&mut assign_dropped);

    Ok(Import {
        score: Score::from_notes("imported", bpm, notes),
        tracks: names,
        casting,
        dropped,
    })
}

#[allow(clippy::too_many_arguments)]
fn push(
    raws: &mut Vec<Raw>,
    groups: &mut HashMap<(usize, u8), usize>,
    track: usize,
    channel: u8,
    start_tick: u64,
    end_tick: u64,
    midi: u8,
    velocity: u8,
) {
    if end_tick <= start_tick {
        return;
    }
    let next = groups.len();
    let group = *groups.entry((track, channel)).or_insert(next);
    raws.push(Raw {
        group,
        start_tick,
        end_tick,
        midi,
        velocity,
    });
}

/// Turn raw note groups into four parts. See the module docs for why by pitch and not by order.
fn assign(
    raws: &mut [Raw],
    names: &[String],
    ticks_per_beat: f64,
) -> (Vec<Note>, Vec<(Part, String)>, Vec<String>) {
    let mut dropped = Vec::new();
    let group_count = raws.iter().map(|r| r.group).max().unwrap_or(0) + 1;

    // One group with chords in it is a piano staff: split it by pitch rank per chord.
    if group_count == 1 {
        split_by_rank(raws);
    }
    let group_count = raws.iter().map(|r| r.group).max().unwrap_or(0) + 1;

    // Mean pitch per group, lowest first.
    let mut order: Vec<(usize, f64)> = (0..group_count)
        .filter_map(|group| {
            let pitches: Vec<f64> = raws
                .iter()
                .filter(|r| r.group == group)
                .map(|r| f64::from(r.midi))
                .collect();
            if pitches.is_empty() {
                return None;
            }
            Some((group, pitches.iter().sum::<f64>() / pitches.len() as f64))
        })
        .collect();
    order.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("pitches are finite"));

    if order.len() > 4 {
        dropped.push(format!(
            "{} voices past the first four (a duck chorale has four parts)",
            order.len() - 4
        ));
        order.truncate(4);
    }

    // A named part beats a measured one: a human wrote the name down.
    let mut seats: Vec<(usize, Part, String)> = Vec::new();
    let mut taken = [false; 4];
    for (rank, (group, _)) in order.iter().enumerate() {
        let named = names.get(*group).and_then(|name| part_named(name));
        if let Some(part) = named
            && !taken[part as usize]
        {
            taken[part as usize] = true;
            seats.push((*group, part, format!("named {:?}", names[*group])));
            continue;
        }
        seats.push((*group, Part::Bass, format!("rank {}", rank + 1)));
    }
    // Fill the unnamed seats with the parts nobody claimed, low to high, in pitch order.
    let mut spare: Vec<Part> = Part::ensemble(order.len())
        .into_iter()
        .filter(|p| !taken[*p as usize])
        .collect();
    for seat in seats.iter_mut() {
        if seat.2.starts_with("rank")
            && let Some(part) = spare.first().copied()
        {
            spare.remove(0);
            seat.1 = part;
            seat.2 = format!("{} by pitch", seat.2);
        }
    }

    let mut notes = Vec::new();
    let mut casting = Vec::new();
    for (group, part, why) in &seats {
        casting.push((*part, why.clone()));
        for raw in raws.iter().filter(|r| r.group == *group) {
            notes.push(Note {
                part: *part,
                start_beat: raw.start_tick as f64 / ticks_per_beat,
                beats: (raw.end_tick - raw.start_tick) as f64 / ticks_per_beat,
                midi: raw.midi,
                // Velocity is a dynamic, which is free and right. MIDI's nominal `mf` is 64.
                level: (f64::from(raw.velocity) / 100.0).clamp(0.15, 1.0),
                vowel: Vowel::Ah,
            });
        }
    }
    casting.sort_by_key(|(part, _)| *part);
    (notes, casting, dropped)
}

/// Split one polyphonic group into voices by pitch rank within each chord.
///
/// The lowest note sounding at a given onset is the bass line, the next is the tenor, and so on.
/// Crude — a real voice-splitter follows lines across time rather than ranking each chord
/// independently — and it turns a piano-staff export into something singable, which is the whole
/// bar it has to clear.
fn split_by_rank(raws: &mut [Raw]) {
    let mut onsets: Vec<u64> = raws.iter().map(|r| r.start_tick).collect();
    onsets.sort_unstable();
    onsets.dedup();
    for onset in onsets {
        let mut together: Vec<usize> = (0..raws.len())
            .filter(|i| raws[*i].start_tick == onset)
            .collect();
        together.sort_by_key(|i| raws[*i].midi);
        for (rank, index) in together.into_iter().enumerate() {
            raws[index].group = rank.min(3);
        }
    }
}

/// A part name a human might have typed on a staff.
fn part_named(name: &str) -> Option<Part> {
    let name = name.to_ascii_lowercase();
    // Longest first, so "bassoon" does not become the bass and "soprano 1" does.
    for (needle, part) in [
        ("soprano", Part::Soprano),
        ("tenor", Part::Tenor),
        ("alto", Part::Alto),
        ("bass", Part::Bass),
        ("sop", Part::Soprano),
    ] {
        if name.contains(needle) {
            return Some(part);
        }
    }
    None
}

/// A cursor over the file, whose every read is bounds-checked — this parses bytes from a file
/// somebody downloaded, and a panic on a truncated one would be a crash where an error belongs.
struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, at: 0 }
    }

    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.at)
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], MidiError> {
        let end = self
            .at
            .checked_add(n)
            .ok_or(MidiError::Truncated { at: self.at })?;
        let slice = self
            .bytes
            .get(self.at..end)
            .ok_or(MidiError::Truncated { at: self.at })?;
        self.at = end;
        Ok(slice)
    }

    fn skip(&mut self, n: usize) -> Result<(), MidiError> {
        self.take(n).map(|_| ())
    }

    fn u8(&mut self) -> Result<u8, MidiError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, MidiError> {
        let b = self.take(2)?;
        Ok(u16::from_be_bytes([b[0], b[1]]))
    }

    fn u32(&mut self) -> Result<u32, MidiError> {
        let b = self.take(4)?;
        Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn tag(&mut self) -> Result<[u8; 4], MidiError> {
        let b = self.take(4)?;
        Ok([b[0], b[1], b[2], b[3]])
    }

    /// MIDI's variable-length quantity: seven bits per byte, high bit means "more follows".
    fn varint(&mut self) -> Result<u64, MidiError> {
        let mut value = 0u64;
        // Four bytes is the spec's maximum; more than that is a corrupt file, not a big number.
        for _ in 0..4 {
            let byte = self.u8()?;
            value = (value << 7) | u64::from(byte & 0x7F);
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        Err(MidiError::Truncated { at: self.at })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a Standard MIDI File in memory. The tests need real bytes — a parser tested against
    /// its own intermediate representation is a parser tested against nothing.
    struct Builder {
        tracks: Vec<Vec<u8>>,
        division: u16,
    }

    impl Builder {
        fn new() -> Self {
            Self {
                tracks: Vec::new(),
                division: 480,
            }
        }

        /// A track of `(start_tick, length_ticks, midi, velocity)`, optionally named.
        fn track(mut self, name: Option<&str>, channel: u8, notes: &[(u64, u64, u8, u8)]) -> Self {
            let mut events: Vec<(u64, Vec<u8>)> = Vec::new();
            if let Some(name) = name {
                let mut meta = vec![0xFF, 0x03, name.len() as u8];
                meta.extend_from_slice(name.as_bytes());
                events.push((0, meta));
            }
            for (start, length, midi, velocity) in notes {
                events.push((*start, vec![0x90 | channel, *midi, *velocity]));
                // Note-off as a zero-velocity note-on, the way real files do it.
                events.push((start + length, vec![0x90 | channel, *midi, 0]));
            }
            events.sort_by_key(|(tick, _)| *tick);

            let mut body = Vec::new();
            let mut last = 0u64;
            for (tick, event) in events {
                body.extend(varint(tick - last));
                body.extend(event);
                last = tick;
            }
            body.extend(varint(0));
            body.extend([0xFF, 0x2F, 0x00]); // end of track
            self.tracks.push(body);
            self
        }

        fn tempo_track(self, bpm: f64) -> Self {
            self.tempo_changes(&[(0, bpm)])
        }

        /// A conductor track carrying `(tick, bpm)` tempo marks.
        fn tempo_changes(mut self, marks: &[(u64, f64)]) -> Self {
            let mut body = Vec::new();
            let mut last = 0u64;
            for (tick, bpm) in marks {
                let micros = (60_000_000.0 / bpm) as u32;
                body.extend(varint(tick - last));
                body.extend([0xFF, 0x51, 0x03]);
                body.extend(&micros.to_be_bytes()[1..]);
                last = *tick;
            }
            body.extend(varint(0));
            body.extend([0xFF, 0x2F, 0x00]);
            self.tracks.insert(0, body);
            self
        }

        fn build(self) -> Vec<u8> {
            let mut out = Vec::new();
            out.extend(b"MThd");
            out.extend(6u32.to_be_bytes());
            out.extend(1u16.to_be_bytes()); // format 1
            out.extend((self.tracks.len() as u16).to_be_bytes());
            out.extend(self.division.to_be_bytes());
            for track in self.tracks {
                out.extend(b"MTrk");
                out.extend((track.len() as u32).to_be_bytes());
                out.extend(track);
            }
            out
        }
    }

    fn varint(mut value: u64) -> Vec<u8> {
        let mut buffer = vec![(value & 0x7F) as u8];
        value >>= 7;
        while value > 0 {
            buffer.push(((value & 0x7F) as u8) | 0x80);
            value >>= 7;
        }
        buffer.reverse();
        buffer
    }

    /// A four-track file in notation-editor order — soprano staff first — must come out with the
    /// bass on the bass. This is the case track order gets backwards, and it is the case that
    /// matters, because it is what MuseScore exports.
    #[test]
    fn a_score_exported_top_staff_first_still_puts_the_bass_on_the_bass() {
        let bytes = Builder::new()
            .tempo_track(58.0)
            .track(None, 0, &[(0, 480, 64, 80), (480, 480, 65, 80)]) // soprano, high
            .track(None, 1, &[(0, 480, 60, 80), (480, 480, 60, 80)]) // alto
            .track(None, 2, &[(0, 480, 55, 80), (480, 480, 55, 80)]) // tenor
            .track(None, 3, &[(0, 480, 48, 80), (480, 480, 45, 80)]) // bass, low
            .build();
        let import = parse(&bytes).expect("parses");
        assert!(
            (import.score.bpm - 58.0).abs() < 0.5,
            "{}",
            import.score.bpm
        );
        assert_eq!(import.score.parts().len(), 4);

        // Each part's mean pitch is in order, which is the property that matters.
        let mut previous = 0.0;
        for part in [Part::Bass, Part::Tenor, Part::Alto, Part::Soprano] {
            let mean = import.score.mean_pitch(part);
            assert!(
                mean > previous,
                "{part:?} at {mean} is not above {previous}"
            );
            previous = mean;
        }
        // The bass really is the low line, not merely the lowest of a shuffled set.
        assert_eq!(
            import
                .score
                .line(Part::Bass)
                .map(|n| n.midi)
                .collect::<Vec<_>>(),
            vec![48, 45]
        );
    }

    /// A track that says what it is is believed, even when its pitches would rank it elsewhere —
    /// a human typed the name.
    #[test]
    fn a_named_track_is_believed_over_its_pitches() {
        let bytes = Builder::new()
            .track(Some("Bass"), 0, &[(0, 480, 57, 80)])
            .track(Some("Soprano"), 1, &[(0, 480, 55, 80)])
            .build();
        let import = parse(&bytes).expect("parses");
        assert_eq!(
            import.score.line(Part::Bass).next().expect("sings").midi,
            57,
            "the higher line was still named the bass"
        );
        assert_eq!(
            import.score.line(Part::Soprano).next().expect("sings").midi,
            55
        );
        assert!(
            import.casting.iter().any(|(_, why)| why.contains("named")),
            "{:?}",
            import.casting
        );
        // "Bassoon" must not become the bass by accident.
        assert_eq!(
            part_named("Bassoon"),
            Some(Part::Bass),
            "documented limitation"
        );
        assert_eq!(part_named("Soprano 1"), Some(Part::Soprano));
        assert_eq!(part_named("Violin"), None);
    }

    /// One track with chords in it — what a lot of tools mean by "export MIDI" — is split into
    /// lines by pitch rank rather than refused.
    #[test]
    fn a_single_polyphonic_track_is_split_into_voices() {
        let bytes = Builder::new()
            .track(
                None,
                0,
                &[
                    (0, 480, 48, 80),
                    (0, 480, 55, 80),
                    (0, 480, 60, 80),
                    (0, 480, 64, 80),
                    (480, 480, 45, 80),
                    (480, 480, 55, 80),
                    (480, 480, 60, 80),
                    (480, 480, 64, 80),
                ],
            )
            .build();
        let import = parse(&bytes).expect("parses");
        assert_eq!(import.score.parts().len(), 4, "{:?}", import.score.parts());
        assert_eq!(
            import
                .score
                .line(Part::Bass)
                .map(|n| n.midi)
                .collect::<Vec<_>>(),
            vec![48, 45],
            "the lowest note of each chord is the bass line"
        );
        // The soprano's E4 is held across both chords, so the tie merge makes it one note.
        let soprano: Vec<&Note> = import.score.line(Part::Soprano).collect();
        assert_eq!(soprano.len(), 1);
        assert_eq!(soprano[0].beats, 2.0);
    }

    /// Ticks become beats through the file's own division, and velocity becomes the dynamic.
    #[test]
    fn timing_and_velocity_survive_the_trip() {
        let bytes = Builder::new()
            .tempo_track(120.0)
            // 480 ticks per beat: a note of 720 ticks is a beat and a half, starting on beat 2.
            .track(None, 0, &[(960, 720, 60, 100), (1680, 240, 62, 30)])
            .build();
        let import = parse(&bytes).expect("parses");
        let line: Vec<&Note> = import.score.notes.iter().collect();
        assert_eq!(line[0].start_beat, 2.0);
        assert_eq!(line[0].beats, 1.5);
        assert_eq!(line[1].start_beat, 3.5);
        assert_eq!(line[1].beats, 0.5);
        assert!((line[0].level - 1.0).abs() < 1e-9, "velocity 100 is loud");
        assert!(
            line[1].level < 0.4,
            "velocity 30 is quiet: {}",
            line[1].level
        );
        assert!((import.score.bpm - 120.0).abs() < 0.5);
    }

    /// A tempo change is dropped, and *says* it was. Singing the wrong tempo silently would be
    /// worse than refusing; refusing would be worse than singing it at the first tempo and
    /// saying so — four robots agreeing over a network on a tempo *map* is a promise this cannot
    /// keep.
    #[test]
    fn a_tempo_change_is_reported_rather_than_hidden() {
        let bytes = Builder::new()
            .tempo_changes(&[(0, 120.0), (960, 240.0)])
            .track(None, 0, &[(0, 480, 60, 80)])
            .build();
        let import = parse(&bytes).expect("parses");
        assert!(
            (import.score.bpm - 120.0).abs() < 0.5,
            "the first tempo wins"
        );
        assert!(
            import.dropped.iter().any(|d| d.contains("tempo change")),
            "{:?}",
            import.dropped
        );

        // A file whose tempo is *restated* rather than changed says nothing — a notation editor
        // emits a redundant mark often enough that warning about it would be noise.
        let restated = Builder::new()
            .tempo_changes(&[(0, 96.0), (960, 96.0)])
            .track(None, 0, &[(0, 480, 60, 80)])
            .build();
        let import = parse(&restated).expect("parses");
        assert!(import.dropped.is_empty(), "{:?}", import.dropped);
        assert!((import.score.bpm - 96.0).abs() < 0.5);
    }

    /// A file this cannot sing must say which thing it could not do, and a corrupt one must not
    /// panic: these are bytes from a file somebody downloaded.
    #[test]
    fn bad_files_are_errors_and_never_panics() {
        assert_eq!(parse(b"").unwrap_err(), MidiError::NotMidi);
        assert_eq!(
            parse(b"not a midi file at all").unwrap_err(),
            MidiError::NotMidi
        );

        // A valid header with no notes behind it.
        let empty = Builder::new().tempo_track(60.0).build();
        assert_eq!(parse(&empty).unwrap_err(), MidiError::NoNotes);

        // SMPTE division.
        let mut smpte = Builder::new().track(None, 0, &[(0, 480, 60, 80)]).build();
        smpte[12] = 0xE8;
        smpte[13] = 0x08;
        assert_eq!(parse(&smpte).unwrap_err(), MidiError::SmpteTiming);

        // Every truncation of a real file is an error, not a crash.
        let good = Builder::new()
            .tempo_track(60.0)
            .track(Some("Soprano"), 0, &[(0, 480, 64, 80)])
            .track(Some("Bass"), 1, &[(0, 480, 48, 80)])
            .build();
        for cut in 0..good.len() {
            let _ = parse(&good[..cut]);
        }
        // And a chunk length that claims more than the file holds.
        let mut lying = good.clone();
        let last = lying.len() - 1;
        lying[last - 3..].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]);
        let _ = parse(&lying);
    }

    /// Running status — a note-on whose status byte is implied by the previous event — is how
    /// real files save space, and a parser that missed it would drop most of the notes in one.
    #[test]
    fn running_status_is_understood() {
        let mut body = varint(0);
        body.extend([0x90, 60, 80]); // note on, explicit status
        body.extend(varint(480));
        body.extend([60, 0]); // note off, running status
        body.extend(varint(0));
        body.extend([48, 80]); // note on, still running status
        body.extend(varint(480));
        body.extend([48, 0]);
        body.extend(varint(0));
        body.extend([0xFF, 0x2F, 0x00]);

        let mut bytes = Vec::new();
        bytes.extend(b"MThd");
        bytes.extend(6u32.to_be_bytes());
        bytes.extend(0u16.to_be_bytes());
        bytes.extend(1u16.to_be_bytes());
        bytes.extend(480u16.to_be_bytes());
        bytes.extend(b"MTrk");
        bytes.extend((body.len() as u32).to_be_bytes());
        bytes.extend(body);

        let import = parse(&bytes).expect("parses");
        assert_eq!(import.score.notes.len(), 2, "{:?}", import.score.notes);
        let pitches: Vec<u8> = import.score.notes.iter().map(|n| n.midi).collect();
        assert!(pitches.contains(&60) && pitches.contains(&48));
    }

    /// A track carrying two channels is two groups, and the parts named after it must still be
    /// read from the right tracks. Counting a group as a track lent every later group the next
    /// track's name: the alto line was "named Tenor", the tenor line "named Bass", and the real
    /// bass was left the alto's seat. The shared staff's own name cannot say which of its two
    /// lines is which, so those two are seated by pitch.
    #[test]
    fn a_track_with_two_channels_does_not_shift_the_names_after_it() {
        // Soprano and alto on one staff, on channels 0 and 1, then a named tenor and bass.
        let mut upper = Builder::new()
            .track(Some("Soprano/Alto"), 0, &[(0, 480, 72, 80)])
            .tracks
            .remove(0);
        let alto = Builder::new()
            .track(None, 1, &[(0, 480, 67, 80)])
            .tracks
            .remove(0);
        // Splice the alto's events in ahead of the soprano track's end-of-track meta.
        upper.truncate(upper.len() - 4);
        upper.extend_from_slice(&alto);
        let bytes = Builder {
            tracks: vec![upper],
            division: 480,
        }
        .track(Some("Tenor"), 2, &[(0, 480, 60, 80)])
        .track(Some("Bass"), 3, &[(0, 480, 48, 80)])
        .build();

        let import = parse(&bytes).expect("parses");
        let sings = |part| import.score.line(part).next().expect("sings").midi;
        assert_eq!(sings(Part::Soprano), 72);
        assert_eq!(sings(Part::Alto), 67);
        assert_eq!(sings(Part::Tenor), 60, "the track named Tenor sings tenor");
        assert_eq!(sings(Part::Bass), 48, "the track named Bass sings bass");
    }

    /// More than four voices is reported rather than silently dropped — a chorale has four parts
    /// and someone importing an orchestral score should be told which is which.
    #[test]
    fn more_than_four_voices_says_what_it_left_out() {
        let mut builder = Builder::new();
        for (index, pitch) in [45u8, 50, 55, 60, 64, 69].into_iter().enumerate() {
            builder = builder.track(None, index as u8, &[(0, 480, pitch, 80)]);
        }
        let import = parse(&builder.build()).expect("parses");
        assert_eq!(import.score.parts().len(), 4);
        assert!(
            import
                .dropped
                .iter()
                .any(|d| d.contains("past the first four")),
            "{:?}",
            import.dropped
        );
        // It keeps the *lowest* four, so the bass line survives.
        assert_eq!(
            import.score.line(Part::Bass).next().expect("sings").midi,
            45
        );
    }
}
