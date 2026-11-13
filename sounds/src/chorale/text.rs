//! The text score format: a chorale you can write in an editor and diff in a review.
//!
//! Scores started as `Gesture` literals in Rust, which is fine for the one piece that ships
//! and wrong for making songs: every change is a recompile, and nobody who writes music is
//! going to touch a `vec![Gesture::Chord { .. }]`. This is the same gestures in a line-oriented
//! text file — one gesture per line, note names rather than MIDI numbers, running state for the
//! things a score marks once and means until further notice (the dynamic, the vowel).
//!
//! `scores/wistful.duckscore` is the grammar's own documentation and is embedded as
//! [`super::Score::wistful`], so the shipped piece and the worked example are the same file and
//! cannot drift apart.
//!
//! ## What this is not
//!
//! Not a general music format. It has no bars, no key signature, no time signature and no
//! per-voice rhythmic independence beyond what [`Gesture`] can express — a score where all four
//! voices move in different rhythms is not writable here, and is exactly what the MIDI importer
//! ([`super::midi`]) is for. The two front ends are deliberately different shapes: this one is
//! for writing a chorale by hand and reading it back, that one is for anything a notation
//! editor can produce.
//!
//! ## Errors name the line
//!
//! Every failure carries the line number and the text of the line, because the alternative —
//! "invalid score" — sends someone back to stare at forty lines of chords. A score is
//! hand-written data, so a parse error is the normal way to find out you mistyped a note name.

use std::fmt;

use super::{Gesture, Part, Score, SoloNote, Voicing, Vowel, dynamic};

/// What went wrong, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    /// 1-based, as an editor counts.
    pub line: usize,
    pub text: String,
    pub problem: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "line {}: {}\n  {}",
            self.line,
            self.problem,
            self.text.trim()
        )
    }
}

impl std::error::Error for ParseError {}

/// Parse a score.
pub fn parse(source: &str) -> Result<Score, ParseError> {
    let mut name = "untitled".to_owned();
    let mut bpm = 60.0f64;
    let mut gestures: Vec<Gesture> = Vec::new();
    // Running state: what a score marks once and means until it says otherwise.
    let mut level = dynamic("mf").expect("mf is a dynamic");
    let mut vowel = Vowel::Ah;

    for (index, raw) in source.lines().enumerate() {
        let line = index + 1;
        let fail = |problem: String| ParseError {
            line,
            text: raw.to_owned(),
            problem,
        };
        // Comments run to end of line, so a chord can be annotated with what it is.
        let content = without_comment(raw).trim();
        if content.is_empty() {
            continue;
        }

        if let Some(rest) = content.strip_prefix("name:") {
            name = rest.trim().to_owned();
            continue;
        }
        if let Some(rest) = content.strip_prefix("bpm:") {
            bpm = rest
                .trim()
                .parse()
                .map_err(|_| fail(format!("{:?} is not a tempo", rest.trim())))?;
            if !(20.0..=300.0).contains(&bpm) {
                return Err(fail(format!("{bpm} bpm is not a tempo anyone can sing")));
            }
            continue;
        }

        let words: Vec<&str> = content.split_whitespace().collect();
        match words[0] {
            "dynamic" => {
                let mark = words.get(1).ok_or_else(|| fail("dynamic what?".into()))?;
                level = dynamic(mark).ok_or_else(|| {
                    fail(format!("{mark:?} is not a dynamic (ppp pp p mp mf f ff)"))
                })?;
            }
            "vowel" => {
                let named = words.get(1).ok_or_else(|| fail("vowel what?".into()))?;
                vowel = Vowel::parse(named)
                    .ok_or_else(|| fail(format!("{named:?} is not a vowel (ah eh ee oh oo mm)")))?;
            }
            "rest" => {
                let beats = beats_of(words.get(1), &fail)?;
                gestures.push(Gesture::Rest { beats });
            }
            "chord" => {
                let beats = beats_of(words.get(1), &fail)?;
                let voicing = voicing_of(&words[2..], &fail)?;
                gestures.push(Gesture::Chord {
                    voicing,
                    beats,
                    vowel,
                    level,
                });
            }
            "build" => {
                let beats = beats_of(words.get(1), &fail)?;
                if words.get(2) != Some(&"stagger") {
                    return Err(fail("build needs `stagger <beats>`".into()));
                }
                let stagger = beats_of(words.get(3), &fail)?;
                // `top` is optional and sits between the stagger and the voicing.
                let (from_top, voicing_from) = match words.get(4) {
                    Some(&"top") => (true, 5),
                    _ => (false, 4),
                };
                let voicing = voicing_of(&words[voicing_from.min(words.len())..], &fail)?;
                gestures.push(Gesture::Build {
                    voicing,
                    beats,
                    stagger,
                    from_top,
                    vowel,
                    level,
                });
            }
            "solo" => gestures.push(solo(&words, vowel, level, &fail)?),
            other => {
                return Err(fail(format!(
                    "{other:?} is not a gesture (chord build solo rest dynamic vowel)"
                )));
            }
        }
    }

    if gestures.is_empty() {
        return Err(ParseError {
            line: 0,
            text: String::new(),
            problem: "a score with no gestures in it is not a score".into(),
        });
    }
    Ok(Score::from_gestures(&name, bpm, &gestures))
}

/// The line up to its comment, if it has one.
///
/// A comment is a `#` that starts a word, not any `#` at all: `#` is also the sharp sign, and a
/// note written `F#4` — the spelling the format documents — would otherwise be cut to `F`, which
/// is no note at all.
fn without_comment(line: &str) -> &str {
    let mut previous = None;
    for (at, c) in line.char_indices() {
        if c == '#' && previous.is_none_or(char::is_whitespace) {
            return &line[..at];
        }
        previous = Some(c);
    }
    line
}

/// `solo <part> under B T A S [hum <vowel>] sing <note>[:beats][/vowel] ...`
fn solo(
    words: &[&str],
    running_vowel: Vowel,
    level: f64,
    fail: &impl Fn(String) -> ParseError,
) -> Result<Gesture, ParseError> {
    let named = words.get(1).ok_or_else(|| fail("solo who?".into()))?;
    let part = part_of(named)
        .ok_or_else(|| fail(format!("{named:?} is not a part (bass tenor alto soprano)")))?;
    let keyword = |word: &str| words.iter().position(|w| *w == word);
    let under_at = keyword("under").ok_or_else(|| fail("solo needs `under B T A S`".into()))?;
    let sing_at = keyword("sing").ok_or_else(|| fail("solo needs `sing <notes>`".into()))?;
    if sing_at <= under_at {
        return Err(fail("`under` comes before `sing`".into()));
    }
    // `hum <vowel>` is optional; a choir behind a soloist hums unless told otherwise.
    let (under_vowel, under_end) = match keyword("hum") {
        Some(at) if at > under_at && at < sing_at => {
            let named = words
                .get(at + 1)
                .ok_or_else(|| fail("hum on what vowel?".into()))?;
            let vowel =
                Vowel::parse(named).ok_or_else(|| fail(format!("{named:?} is not a vowel")))?;
            (vowel, at)
        }
        _ => (Vowel::Mm, sing_at),
    };
    let under = voicing_of(&words[under_at + 1..under_end], fail)?;

    let mut line = Vec::new();
    for word in &words[sing_at + 1..] {
        // `E4`, `E4:2`, `E4/ee`, `E4:2/ee` — a pitch, then optionally how long, then optionally
        // on what. Beats default to one, the vowel to whatever is running.
        let (pitch_and_beats, vowel) = match word.split_once('/') {
            Some((head, named)) => (
                head,
                Vowel::parse(named).ok_or_else(|| fail(format!("{named:?} is not a vowel")))?,
            ),
            None => (*word, running_vowel),
        };
        let (pitch, beats) = match pitch_and_beats.split_once(':') {
            Some((pitch, beats)) => (
                pitch,
                beats
                    .parse::<f64>()
                    .map_err(|_| fail(format!("{beats:?} is not a length")))?,
            ),
            None => (pitch_and_beats, 1.0),
        };
        if beats <= 0.0 {
            return Err(fail(format!("a note cannot last {beats} beats")));
        }
        let midi = midi_of(pitch)
            .ok_or_else(|| fail(format!("{pitch:?} is not a note name")))?
            .ok_or_else(|| fail("a soloist cannot sing a rest — use `rest`".into()))?;
        line.push(SoloNote { midi, beats, vowel });
    }
    if line.is_empty() {
        return Err(fail("a solo with no notes in it".into()));
    }
    Ok(Gesture::Solo {
        part,
        notes: line,
        under,
        under_vowel,
        level,
    })
}

fn beats_of(word: Option<&&str>, fail: &impl Fn(String) -> ParseError) -> Result<f64, ParseError> {
    let word = word.ok_or_else(|| fail("expected a number of beats".into()))?;
    let beats: f64 = word
        .parse()
        .map_err(|_| fail(format!("{word:?} is not a number of beats")))?;
    if beats <= 0.0 || !beats.is_finite() {
        return Err(fail(format!("{beats} is not a length")));
    }
    Ok(beats)
}

/// Four note names, low to high. `-` is a voice that does not sing.
fn voicing_of(words: &[&str], fail: &impl Fn(String) -> ParseError) -> Result<Voicing, ParseError> {
    if words.len() != 4 {
        return Err(fail(format!(
            "expected four voices low to high (bass tenor alto soprano), got {}",
            words.len()
        )));
    }
    let mut voicing: Voicing = [None; 4];
    for (slot, word) in voicing.iter_mut().zip(words) {
        *slot = midi_of(word).ok_or_else(|| fail(format!("{word:?} is not a note name")))?;
    }
    Ok(voicing)
}

fn part_of(word: &str) -> Option<Part> {
    Some(match word.to_ascii_lowercase().as_str() {
        "bass" | "b" => Part::Bass,
        "tenor" | "t" => Part::Tenor,
        "alto" | "a" => Part::Alto,
        "soprano" | "s" => Part::Soprano,
        _ => return None,
    })
}

/// A scientific pitch name to MIDI. `Ok(None)` is the tacet marker `-`.
///
/// C4 is middle C, which is MIDI 60 — the convention a notation editor uses, so a note copied
/// off a staff means what it looks like. (The other convention in circulation puts middle C at
/// C3; picking the editor's is the one that avoids an octave of confusion per score.)
fn midi_of(word: &str) -> Option<Option<u8>> {
    if word == "-" {
        return Some(None);
    }
    let mut chars = word.chars();
    let step = match chars.next()?.to_ascii_uppercase() {
        'C' => 0,
        'D' => 2,
        'E' => 4,
        'F' => 5,
        'G' => 7,
        'A' => 9,
        'B' => 11,
        _ => return None,
    };
    let mut rest = chars.as_str();
    let mut accidental = 0i32;
    while let Some(first) = rest.chars().next() {
        match first {
            '#' | 's' => accidental += 1,
            'b' | 'f' => accidental -= 1,
            _ => break,
        }
        rest = &rest[first.len_utf8()..];
    }
    let octave: i32 = rest.parse().ok()?;
    let midi = (octave + 1) * 12 + step + accidental;
    if !(0..=127).contains(&midi) {
        return None;
    }
    Some(Some(midi as u8))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shipped piece must parse, because it *is* the default score — and it must still be
    /// the piece it was, which the counts and the shape check.
    #[test]
    fn the_embedded_default_score_parses() {
        let score = Score::wistful();
        assert_eq!(score.name, "wistful");
        assert_eq!(score.bpm, 58.0);
        assert!(
            (30.0..90.0).contains(&score.duration_s()),
            "{}s",
            score.duration_s()
        );
        assert_eq!(score.parts().len(), 4, "all four voices sing");
        // The dynamics and vowels actually vary, or the marks are decoration.
        let levels: Vec<f64> = score.notes.iter().map(|n| n.level).collect();
        assert!(
            levels.iter().any(|l| *l != levels[0]),
            "nothing ever changes dynamic"
        );
        assert!(
            score.notes.iter().any(|n| n.vowel == Vowel::Mm),
            "nobody ever hums"
        );
        assert!(score.notes.iter().any(|n| n.vowel == Vowel::Ah));
    }

    /// Middle C is C4 is 60 — the notation editor's convention, so a note copied off a staff
    /// means what it looks like.
    #[test]
    fn note_names_follow_the_editors_octave_numbering() {
        assert_eq!(midi_of("C4"), Some(Some(60)));
        assert_eq!(midi_of("A4"), Some(Some(69)), "concert A");
        assert_eq!(midi_of("C3"), Some(Some(48)));
        assert_eq!(midi_of("A2"), Some(Some(45)));
        assert_eq!(midi_of("D5"), Some(Some(74)));
        // Accidentals, both spellings, and enharmonics that must agree.
        assert_eq!(midi_of("F#4"), Some(Some(66)));
        assert_eq!(midi_of("Gb4"), Some(Some(66)));
        assert_eq!(midi_of("Ab3"), Some(Some(56)));
        assert_eq!(midi_of("G#3"), Some(Some(56)));
        assert_eq!(midi_of("c4"), Some(Some(60)), "case does not matter");
        // The tacet marker, and things that are not notes.
        assert_eq!(midi_of("-"), Some(None));
        assert_eq!(midi_of("H4"), None);
        assert_eq!(midi_of("C"), None, "a note needs an octave");
        assert_eq!(midi_of(""), None);
        assert_eq!(midi_of("C99"), None, "off the keyboard");
    }

    /// Running state is the point of the format: a dynamic or a vowel is marked once and means
    /// what it says until it is marked again.
    #[test]
    fn a_mark_holds_until_it_is_changed() {
        let score = parse(
            "name: t\nbpm: 60\n\
             dynamic p\nvowel oo\n\
             chord 1  C3 G3 C4 E4\n\
             chord 1  A2 G3 C4 E4\n\
             dynamic ff\nvowel ee\n\
             chord 1  F3 A3 C4 F4\n",
        )
        .expect("parses");
        let bass: Vec<&super::super::Note> = score.line(Part::Bass).collect();
        // Three different bass notes, so nothing ties and the marks are plainly visible.
        assert_eq!(bass.len(), 3, "{bass:?}");
        assert_eq!(bass[0].level, dynamic("p").unwrap());
        assert_eq!(bass[1].level, dynamic("p").unwrap(), "still piano");
        assert_eq!(bass[2].level, dynamic("ff").unwrap());
        assert_eq!(bass[0].vowel, Vowel::Oo);
        assert_eq!(bass[1].vowel, Vowel::Oo);
        assert_eq!(bass[2].vowel, Vowel::Ee);
    }

    /// A change of vowel on the same pitch re-articulates rather than tying — that is what a
    /// singer does with a new syllable, and the tie merge has to know it.
    #[test]
    fn a_new_vowel_on_the_same_note_is_a_new_note() {
        let tied =
            parse("name: t\nbpm: 60\nchord 1 C3 G3 C4 E4\nchord 1 C3 G3 C4 E4\n").expect("parses");
        assert_eq!(tied.line(Part::Bass).count(), 1, "one tied note");

        let resung = parse(
            "name: t\nbpm: 60\nvowel ah\nchord 1 C3 G3 C4 E4\nvowel ee\nchord 1 C3 G3 C4 E4\n",
        )
        .expect("parses");
        assert_eq!(
            resung.line(Part::Bass).count(),
            2,
            "a new syllable is a new note"
        );

        // A dynamic mid-tie is a crescendo, not a re-attack — see `Score::from_notes`.
        let swell = parse(
            "name: t\nbpm: 60\ndynamic p\nchord 1 C3 G3 C4 E4\ndynamic ff\nchord 1 C3 G3 C4 E4\n",
        )
        .expect("parses");
        assert_eq!(swell.line(Part::Bass).count(), 1, "a crescendo is one note");
    }

    /// A solo: the soloist takes the running vowel, the choir hums by default, and `-` voices
    /// are genuinely silent.
    #[test]
    fn a_solo_hums_underneath_by_default() {
        let score = parse(
            "name: t\nbpm: 60\nvowel ah\n\
             solo soprano under C3 - C4 - sing E4:1 F4:2 G4/ee\n",
        )
        .expect("parses");
        let soprano: Vec<&super::super::Note> = score.line(Part::Soprano).collect();
        assert_eq!(soprano.len(), 3);
        assert_eq!(soprano[0].vowel, Vowel::Ah, "the running vowel");
        assert_eq!(soprano[1].beats, 2.0);
        assert_eq!(soprano[2].vowel, Vowel::Ee, "overridden per note");
        assert_eq!(soprano[2].beats, 1.0, "a length defaults to one beat");

        let alto: Vec<&super::super::Note> = score.line(Part::Alto).collect();
        assert_eq!(alto.len(), 1, "held under the whole solo");
        assert_eq!(alto[0].beats, 4.0);
        assert_eq!(alto[0].vowel, Vowel::Mm, "the choir hums");
        assert_eq!(score.line(Part::Tenor).count(), 0, "tacet");

        // And `hum` overrides that.
        let open =
            parse("name: t\nbpm: 60\nsolo alto under C3 - - - hum oo sing C4:2\n").expect("parses");
        assert_eq!(
            open.line(Part::Bass).next().expect("sings").vowel,
            Vowel::Oo
        );
    }

    /// A build's optional `top`, which sits between the stagger and the voicing and must not be
    /// mistaken for a note name.
    #[test]
    fn a_build_reads_its_optional_direction() {
        let up = parse("name: t\nbpm: 60\nbuild 4 stagger 0.5 C3 G3 C4 E4\n").expect("parses");
        let down =
            parse("name: t\nbpm: 60\nbuild 4 stagger 0.5 top C3 G3 C4 E4\n").expect("parses");
        let first_in = |score: &Score| {
            let mut notes = score.notes.clone();
            notes.sort_by(|a, b| a.start_beat.partial_cmp(&b.start_beat).expect("finite"));
            notes[0].part
        };
        assert_eq!(first_in(&up), Part::Bass);
        assert_eq!(first_in(&down), Part::Soprano);
    }

    /// Errors name the line and quote it. "Invalid score" would send someone back to stare at
    /// forty lines of chords, which is the failure mode this format exists to avoid.
    #[test]
    fn errors_say_which_line_and_what() {
        let cases = [
            ("name: t\nbpm: 60\nchord 2 C3 G3 C4 H9\n", 3, "note name"),
            ("name: t\nbpm: 60\nchord 2 C3 G3 C4\n", 3, "four voices"),
            ("name: t\nbpm: 60\nchord two C3 G3 C4 E4\n", 3, "beats"),
            (
                "name: t\nbpm: 60\nwaffle 2 C3 G3 C4 E4\n",
                3,
                "not a gesture",
            ),
            (
                "name: t\nbpm: 60\ndynamic loud\nchord 1 C3 G3 C4 E4\n",
                3,
                "not a dynamic",
            ),
            (
                "name: t\nbpm: 60\nvowel argh\nchord 1 C3 G3 C4 E4\n",
                3,
                "not a vowel",
            ),
            ("name: t\nbpm: 900\nchord 1 C3 G3 C4 E4\n", 2, "tempo"),
            ("name: t\nbpm: 60\nbuild 4 C3 G3 C4 E4\n", 3, "stagger"),
            ("name: t\nbpm: 60\nsolo soprano sing C4\n", 3, "under"),
            (
                "name: t\nbpm: 60\nsolo nobody under C3 - - - sing C4\n",
                3,
                "not a part",
            ),
        ];
        for (source, line, needle) in cases {
            let error = parse(source).expect_err(&format!("{source:?} must not parse"));
            assert_eq!(error.line, line, "{source:?}");
            assert!(
                error.problem.contains(needle),
                "{source:?}: {:?} does not mention {needle:?}",
                error.problem
            );
            // And it quotes the offending line, so the message stands alone in a log.
            assert!(!format!("{error}").is_empty());
        }
        // An empty score is a score-level failure, not a line-level one.
        assert_eq!(parse("name: t\nbpm: 60\n").expect_err("empty").line, 0);
    }

    /// A sharp is written `F#4`, as the format's own documentation says, and a `#` inside a
    /// note name is not the start of a comment. Cutting the line at it turned every sharp into
    /// a note with no octave, so no score could contain one.
    #[test]
    fn a_sharp_is_a_note_and_not_a_comment() {
        let score = parse("name: t\nbpm: 60\nchord 2 D3 A3 D4 F#4   # D major\n")
            .expect("a sharp is a note name");
        assert_eq!(
            score.line(Part::Soprano).next().expect("sings").midi,
            66,
            "F#4 is the F sharp above middle C"
        );
        let solo = parse("name: t\nbpm: 60\nsolo soprano under D3 - - - sing C#5:2 D5\n")
            .expect("a sharp in a solo line too");
        assert_eq!(
            solo.line(Part::Soprano).map(|n| n.midi).collect::<Vec<_>>(),
            vec![73, 74]
        );
    }

    /// Comments and blank lines are not content — a score is meant to be annotated, and the
    /// shipped one documents its own format in them.
    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let score = parse(
            "# a comment\n\nname: t  # trailing\nbpm: 60\n\n\
             chord 2 C3 G3 C4 E4   # the tonic\n#chord 2 A2 G3 C4 E4\n",
        )
        .expect("parses");
        assert_eq!(score.name, "t");
        assert_eq!(
            score.line(Part::Bass).count(),
            1,
            "the commented chord is out"
        );
    }
}
