//! `duckctl` — the robot from a laptop.
//!
//! The phone app's stand-in, and the only way to test `btd` against a real radio.
//!
//! **Bluetooth is how it reaches a robot today, not what it is.** `mediad` gives a robot a second
//! transport that reaches a different set of methods by design — `robot.move` is refused over BLE
//! and permitted over WebRTC, `net.connect` the other way round — so the name says which robot
//! rather than which radio. It was called `duck-btctl` while BLE was the only answer.
//!
//! **Nothing on the robot depends on this crate**, which is what keeps `btleplug` out of a
//! release. It used to be an example of `btd` for the same purpose, obtained as a side effect of
//! the directory it sat in; a crate nobody depends on states it directly. `robotctl` is the tool
//! that ships, and it speaks unix sockets on the robot itself.
//!
//! `btleplug` rather than `bluer`, because this runs on a developer's machine: CoreBluetooth on
//! macOS, BlueZ on Linux, WinRT on Windows. `bluer` would restrict the client to Linux, which
//! defeats the point.
//!
//! It reuses `btd::framing` deliberately. The chunking here is the *client* half of the same
//! module the robot uses, so if the framing were asymmetric this would not work — which makes
//! it a real test of the protocol rather than a reimplementation that could agree with itself.
//!
//! ```text
//! cargo run -p duckctl -- scan          # robots in range, and their addresses
//! cargo run -p duckctl -- status
//! cargo run -p duckctl -- wifi scan
//! cargo run -p duckctl -- wifi connect "Pollen" --psk secret
//! cargo run -p duckctl -- name "Ducky"
//! cargo run -p duckctl -- call robot.health
//! ```
//!
//! `DUCK_ROBOT` and `DUCK_PIN` in the environment are the defaults for `--name` and `--pin`, for
//! the machine that talks to the same robot every day. See [`Target`].

use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

use btd::adv;
use btd::framing::{self, Reassembler};
use btd::gatt::{RPC_UUID, SERVICE_UUID};
use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, PeripheralProperties,
    ScanFilter, WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use clap::{Parser, Subcommand};
use futures::StreamExt;

/// How long to look for a robot before giving up.
///
/// Generous, because BLE discovery is genuinely slow and a robot advertises at whatever interval
/// BlueZ chose. Shorter than this and a laptop that was simply unlucky reports "no robot".
const SCAN_TIME: Duration = Duration::from_secs(8);
/// How often the scan results are re-read while waiting.
///
/// A single snapshot after a fixed sleep is what this used to do, and it failed intermittently:
/// BLE advertising is periodic and CoreBluetooth's view of a bonded peripheral comes and goes, so
/// whether the robot was in that one snapshot was partly luck — `no robot found` for a robot that
/// answered fine on the next attempt. Polling until something appears also makes the common case
/// finish in well under a second instead of always paying `SCAN_TIME`.
const SCAN_POLL: Duration = Duration::from_millis(250);

/// How long to wait with **nothing at all arriving** before giving up on a request.
///
/// Idle rather than total, and that distinction is what makes an update watchable. An apply takes
/// as long as the robot needs — download, verify, extract, swap, hooks, the health gate — so a
/// total budget either cuts off a working update or waits out a dead robot. But the useful signal
/// is already arriving: every progress notification is proof the robot is alive and working, so
/// each one starts the clock again. A stalled mirror still fails in seconds.
///
/// Longer than any single call except `net.connect`, which polls NetworkManager for up to 45s and
/// so gets its own budget below.
const REPLY_TIMEOUT: Duration = Duration::from_secs(15);
const SLOW_REPLY_TIMEOUT: Duration = Duration::from_secs(60);
/// An update's silences are longer than any other call's: a hook's phase notification arrives
/// *before* the hook rather than during it, so the budget is the longest gap an update can
/// legitimately have, not the longest an update can take.
///
/// **The gap is the pre-install hook's ceiling**, which is why this is derived from
/// [`duck_ipc_proto::UPDATE_MAX_SILENCE_SECONDS`] rather than being a number here. That hook
/// installs what a release needs and a board may not have — ONNX Runtime, and around 100 MB of apt
/// for `mediad`'s GStreamer stack on a board that never had it — and this was 180 seconds when that
/// ceiling was two minutes. A budget below the ceiling reports a working update as a robot that
/// stopped answering, and the operator's next move is to interrupt an update that was fine.
///
/// A minute of margin over it, for the reply that follows the hook.
const UPDATE_IDLE_TIMEOUT: Duration =
    Duration::from_secs(duck_ipc_proto::UPDATE_MAX_SILENCE_SECONDS + 60);
/// `update watch` follows progress until interrupted, so it has no deadline worth naming. A day
/// is an arbitrary bound that keeps the reply loop one shape instead of two.
const FOLLOW_TIMEOUT: Duration = Duration::from_secs(24 * 3600);

/// How often the wait checks that the link is still up.
///
/// Without it a dropped connection is indistinguishable from a robot gone quiet: the notification
/// stream simply stops yielding, so the wait runs to its idle budget and then reports a robot that
/// has "stopped answering". After an `update apply` that is wrong twice over — the robot answered,
/// and the link is what went away — and it takes `UPDATE_IDLE_TIMEOUT` to say so. Two seconds is
/// far below every budget here and costs one cheap CoreBluetooth query each time.
const LINK_POLL: Duration = Duration::from_secs(2);

/// Every step before the first reply gets its own budget and its own message.
///
/// btleplug bounds none of these, so without them a stall anywhere between "found the robot" and
/// "sent a request" prints `connecting to …` and then nothing at all — which says only that
/// something is wrong, not what. Each of connect, discovery and the first read fails differently
/// and wants a different next move, so each says which one it was.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(20);
const READ_TIMEOUT: Duration = Duration::from_secs(15);

/// How many devices a failure lists before summarising the rest.
///
/// A scan in an office reports dozens, and a wall of earbuds is as unreadable as no list at all.
/// Twelve fits a terminal; the count of what was dropped is printed rather than the list silently
/// ending, because "that was everything" and "that was the first twelve" want different next moves.
const LISTED_DEVICES: usize = 12;

/// One peripheral the Mac reported, kept for `scan` and for the failure message.
///
/// The name is held as it arrived — `None` when the advertisement carried none — rather than as the
/// address fallback the tiers use, because "reported without a name" is the diagnosis and the
/// fallback hides it.
struct Seen {
    peripheral: Peripheral,
    identity: String,
    local_name: Option<String>,
    services: usize,
    /// Whether this advertisement carried the duck service UUID, which is the strongest evidence a
    /// listing has: anything better needs a connection, and `scan` deliberately makes none.
    duck: bool,
    /// What the robot broadcast about its place on the network — see [`Address`], and `btd::adv`
    /// for why four bytes of IPv4 and not the SSID too.
    address: Address,
}

/// What a device said about its IPv4 address, which is three answers rather than two.
///
/// `Option<Ipv4Addr>` would collapse the two blanks into one, and they send the reader somewhere
/// different: a robot that broadcast `0.0.0.0` has no network, and a robot that broadcast nothing is
/// on a release from before this existed. The first is a wifi problem and the second is an update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Address {
    At(Ipv4Addr),
    /// The field was there and said `0.0.0.0`: this robot has no address, because it is on no
    /// network or because DHCP has not given it one yet.
    ///
    /// Not called `None`: it sits next to `Option`'s in [`Address::read`], and one of the two is
    /// about a robot's network while the other is about a missing field.
    Unassigned,
    /// No field at all — an older `btd`, or a device that is not a robot.
    Unsaid,
}

impl Address {
    /// Read from one advertisement, and **only for a robot**.
    ///
    /// `btd` files the address under company id `0xFFFF`, which the Bluetooth SIG leaves open to
    /// anyone, so four bytes from `0xFFFF` on an arbitrary device are four bytes of somebody else's
    /// business. Reading it only where the duck service UUID was also advertised is what keeps a
    /// beacon from being listed with an invented address.
    fn read(properties: &PeripheralProperties, duck: bool) -> Self {
        if !duck {
            return Self::Unsaid;
        }
        match adv::address_in(&properties.manufacturer_data) {
            Some(address) => Self::At(address),
            None if adv::has_address_field(&properties.manufacturer_data) => Self::Unassigned,
            None => Self::Unsaid,
        }
    }

    /// How it reads on the device's line in a listing, or nothing at all.
    ///
    /// `Unsaid` renders as nothing rather than as "unknown": every non-robot line is `Unsaid`, and a
    /// column of "unknown" against a room full of earbuds is noise. The robot on an older release is
    /// covered by the note under the list instead, which has room to say what to do about it.
    fn note(self) -> Option<String> {
        match self {
            Self::At(address) => Some(address.to_string()),
            Self::Unassigned => Some("no address".to_owned()),
            Self::Unsaid => None,
        }
    }
}

/// Whatever names this device on this platform.
///
/// **CoreBluetooth never discloses a peripheral's address**, so on macOS every device reports
/// `00:00:00:00:00:00` and a list keyed on it cannot tell one unnamed device from another — which is
/// the case the list exists for. The per-Mac `id` is stable and does distinguish them, so it stands
/// in. BlueZ reports the real address, and there it is the more useful of the two: it is what
/// `pad pair --mac` and `bluetoothctl` take.
fn identity(peripheral: &Peripheral, address: btleplug::api::BDAddr) -> String {
    if address.into_inner() == [0; 6] {
        peripheral.id().to_string()
    } else {
        address.to_string()
    }
}

/// Does this reported name answer to `wanted`?
///
/// **A peripheral can arrive under two names at once.** CoreBluetooth exposes the *cached GAP
/// name* — `CBPeripheral.name`, learned by reading `0x2A00` on an earlier connection — separately
/// from the local name in the advertisement, and btleplug reports them joined when they differ:
/// `radxa-zero3 [duck-c51b]` (`corebluetooth/internal.rs`, `on_discovered_peripheral`).
///
/// They used to differ on every robot, because the two names came from different places: the GAP
/// name is BlueZ's adapter alias, hostname-derived and therefore `radxa-zero3` on every board
/// flashed from one image, while the advertisement carried the name `configd` owns. `btd` now sets
/// the alias to the advertised name (`btd/src/bluez.rs`, `advertise`), so a robot on a current
/// release reports one name however it is asked.
///
/// **This still has to accept both**, and will for as long as a bench has robots on it. A board on
/// an older release has the old alias; so does a client that cached the old GAP name before the
/// robot was updated, until `bluetoothctl remove <mac>` or forgetting it in macOS Bluetooth
/// settings clears that. Matching the joined string exactly meant **both** spellings a person
/// would type were rejected — and the failure then listed the robot as evidence it was not in
/// range.
///
/// So either half is accepted. The advertised half is the robot's real name, and the one the phone
/// app has to match on; the GAP half is accepted because it is what macOS Bluetooth settings shows.
fn answers_to(reported: &str, wanted: &str) -> bool {
    if reported == wanted {
        return true;
    }
    // `rsplit_once`, so a GAP name that itself contains a bracket keeps the *last* group as the
    // advertised half — which is the one btleplug appended.
    match reported.strip_suffix(']').and_then(|s| s.rsplit_once(" [")) {
        Some((gap, advertised)) => gap == wanted || advertised == wanted,
        None => false,
    }
}

/// The default PIN, which every robot has until somebody sets one.
const DEFAULT_PIN: &str = "000000";

/// Which robot to talk to, and whether anybody typed it.
///
/// A laptop reaches the same robot nearly every time, so the name belongs in the environment rather
/// than in every command line: `export DUCK_ROBOT=duck-c51b`, and `--name` stops being something to
/// remember. `--pin` gets the same treatment through `DUCK_PIN`, which a robot with a real PIN needs
/// more than this does.
///
/// **An empty value means unset**, and that is the reason this is not clap's own `env` support.
/// clap reads the variable with `env::var_os` and treats `DUCK_ROBOT=` as a value, so a variable
/// exported in a shell profile could only be escaped by unsetting it — and the command that needs
/// escaping is the one being typed now, on a bench that has somebody else's robot on it. Empty means
/// unset, so `DUCK_ROBOT= duckctl scan` is the escape hatch, in the shape a shell already has.
///
/// **Provenance is carried rather than recomputed.** A default makes the tool *stricter*: it
/// suppresses the already-connected fallback tier, and turns "the first robot found wins" into "no
/// robot named duck-c51b in range" — a confusing failure six weeks after editing a shell profile,
/// especially when the same message lists a robot sitting right there. So every message about a
/// robot nobody named says where the name came from.
struct Target {
    /// The name to look for, if any. Empty is not a name.
    name: Option<String>,
    /// Whether [`Self::name`] came from the environment rather than from `--name`.
    from_env: bool,
}

impl Target {
    /// `--name` if it was given, otherwise `DUCK_ROBOT` if it says anything.
    fn new(flag: Option<String>, var: Option<String>) -> Self {
        match flag {
            // An empty `--name` is still `--name`. The flag beats the environment in every case
            // including this one, so beating it with nothing is the second escape hatch: a command
            // line can drop the default without touching the shell it runs in.
            Some(name) => Self {
                name: Some(name).filter(|name| !name.is_empty()),
                from_env: false,
            },
            None => {
                let name = var.filter(|name| !name.is_empty());
                Self {
                    from_env: name.is_some(),
                    name,
                }
            }
        }
    }

    /// The name to match on, for the tiers and the search.
    fn wanted(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// Is this the device the name points at? Answers the question a listing is read to ask.
    fn marks(&self, local_name: Option<&str>) -> bool {
        match (self.wanted(), local_name) {
            (Some(wanted), Some(reported)) => answers_to(reported, wanted),
            _ => false,
        }
    }

    /// What to blame for the name, in a line that points at one device.
    fn source(&self) -> &'static str {
        if self.from_env {
            "DUCK_ROBOT"
        } else {
            "--name"
        }
    }

    /// Where the name came from, appended to a failure about a robot nobody asked for by name.
    ///
    /// Empty when `--name` was typed: whoever typed it does not need telling.
    fn provenance(&self) -> String {
        match &self.name {
            Some(name) if self.from_env => format!(
                "\n\nNothing on this command line said {name:?} — `DUCK_ROBOT` in this shell's \
                 environment did. `DUCK_ROBOT= duckctl …` ignores it for one command, and \
                 `unset DUCK_ROBOT` for the shell."
            ),
            _ => String::new(),
        }
    }

    /// The note after a rename that leaves `DUCK_ROBOT` naming a robot that no longer answers.
    ///
    /// The rename works and then every later command searches for the old name and fails, which
    /// looks like a robot that went away rather than a variable that went stale. Only for the
    /// environment: a `--name` typed once is not still in effect.
    fn stale_after_rename(&self, command: &Command) -> Option<String> {
        let Command::Name { name: new } = command else {
            return None;
        };
        let old = self
            .name
            .as_deref()
            .filter(|old| self.from_env && *old != new.as_str())?;
        Some(format!(
            "note: this robot now answers to {new:?}, and `DUCK_ROBOT` still says {old:?}. Every \
             later command looks for {old:?} until that changes."
        ))
    }
}

/// `--pin`, then `DUCK_PIN`, then the factory default.
///
/// Empty means unset here too, for the reason in [`Target`]: a `DUCK_PIN=` left over from a script
/// would otherwise authenticate with an empty string and be reported as a wrong PIN.
///
/// Unlike `--name`, an empty value is *skipped* rather than final. There is no "no PIN" state to
/// express — every request carries one — so `--pin ''` can only mean "not this one".
fn resolve_pin(flag: Option<String>, var: Option<String>) -> String {
    flag.filter(|pin| !pin.is_empty())
        .or(var.filter(|pin| !pin.is_empty()))
        .unwrap_or_else(|| DEFAULT_PIN.to_owned())
}

/// Which of the candidates to talk to, given what was asked for.
///
/// Generic over the payload so the rule can be tested: a `Peripheral` cannot be constructed off a
/// radio, and this is the one place where getting it wrong means acting on the wrong robot.
///
/// **A name that matches more than one candidate is refused, not resolved.** The two are
/// indistinguishable from here, so there is nothing to prefer and picking either means a write
/// landing on whichever the scan happened to report first — `net.connect` puts a wifi password on
/// that robot. `identity.rs` names the way this happens with nobody doing anything wrong: a board
/// whose bootloader leaves `serial-number` empty falls back to the hostname, so every board flashed
/// from one image answers to `radxa-zero3`.
///
/// Without a name the first candidate still wins. That path is unchanged on purpose — choosing
/// between robots nobody named is exactly what omitting `--name` asks for, and making it an error
/// would break the shorthand on any bench with two boards on it.
///
/// Both failures carry [`Target::provenance`]: a name from `DUCK_ROBOT` is a name nobody on this
/// command line typed, and that is worth saying most where the message is about which robot the
/// command would have landed on.
fn choose<T>(found: Vec<(T, String)>, target: &Target) -> Result<(T, String), String> {
    let Some(wanted) = target.wanted() else {
        // `run` returns early on an empty `found`, so there is at least one.
        return found
            .into_iter()
            .next()
            .ok_or_else(|| "no candidates, which `run` should have reported already".to_owned());
    };

    // Collected before the filter consumes `found`: robots *were* there, they just call themselves
    // something else, and naming them beats "not in range" for a robot that has been renamed since
    // whoever is typing last looked.
    let others: Vec<String> = found.iter().map(|(_, name)| name.clone()).collect();
    let mut matching: Vec<(T, String)> = found
        .into_iter()
        .filter(|(_, name)| answers_to(name, wanted))
        .collect();

    if matching.len() > 1 {
        let names: Vec<String> = matching.iter().map(|(_, name)| name.clone()).collect();
        return Err(format!(
            "{} robots answer to {wanted:?}: {}\nRefusing to guess between them — whichever the \
             scan reported first is not a choice. Rename one from the robot itself (`robotctl \
             system set-name`) and use the new name here.{}",
            names.len(),
            names.join(", "),
            target.provenance(),
        ));
    }

    matching.pop().ok_or_else(|| {
        format!(
            "no robot named {wanted:?} in range. These answered to the duck service: {}\nA name of \
             the form `alias [advertised]` is one robot reported under two names, and either half \
             works.{}",
            others.join(", "),
            target.provenance(),
        )
    })
}

/// Deliver a resolved address the way the command asked for it.
///
/// **`ip` prints the address and nothing else**, because the tool's split is diagnostics on stderr
/// and data on stdout: `ssh radxa@$(duckctl ip)` only works if that is the whole of what stdout
/// carries. Every note this command emits goes to stderr for the same reason.
fn deliver(command: &Command, address: &str) -> Result<(), Box<dyn std::error::Error>> {
    match command {
        Command::Ip => {
            println!("{address}");
            Ok(())
        }
        Command::Open { print, port } => {
            let url = console_url(address, *port);
            if *print {
                println!("{url}");
                return Ok(());
            }
            eprintln!("opening {url}");
            webbrowser::open(&url).map_err(|e| {
                format!(
                    "could not open a browser: {e}\nThe robot is at {url} — `duckctl open --print` \
                     gives the URL without launching anything."
                )
                .into()
            })
        }
        // `run` only calls this for the two above; every other command's answer is its JSON.
        _ => Err("this command does not resolve an address".into()),
    }
}

/// Where the console is, given where the robot is.
///
/// Plain `http`, because a robot on a LAN has no certificate to offer and `ws://` from an `https`
/// page is blocked outright as mixed content. `webrtc-console.md` §1.3 says what that costs — a
/// microphone, and on some browsers a gamepad, both being secure-context APIs — and when it changes.
fn console_url(address: &str, port: u16) -> String {
    format!("http://{address}:{port}/")
}

/// What to say when the robot is right there and has no address.
///
/// Two ways to arrive here and they are the same situation: an advertisement that carried `0.0.0.0`,
/// and a `net.status` with no `ip4`. The fix is over the radio in both cases, and it has to be —
/// `net.connect` is refused over WebRTC by design, because a robot that has never seen a network
/// cannot be configured over that network.
fn no_address(name: &str) -> String {
    format!(
        "{name} is in range and has no network address. Join it to a network over the same radio, \
         which needs no network of its own:\n  duckctl --name '{name}' wifi connect <ssid> --psk \
         <passphrase>\nThen `duckctl ip` again. `duckctl wifi status` says what the wifi is doing."
    )
}

/// Has discovery found what it came for, or should it keep listening until the deadline?
///
/// **Without a name, the first candidate wins**, and stopping there is the point: a bonded robot may
/// never re-advertise the service to this Mac, so waiting out the deadline for a better one would
/// just be eight seconds of nothing.
///
/// **With a name, "a candidate" is not the same thing as "the candidate".** The tiers are built
/// before the name is applied — `advertised` holds every robot carrying the service UUID, whoever it
/// is — so stopping at the first non-empty tier stops at whichever robot the radio happened to
/// report first. On a bench with two of them that is a coin flip, and the robot that loses it is
/// never scanned for at all: the failure then reads `no robot named "olducky" in range` and lists
/// the other robot as evidence, which is a claim about eight seconds made after two hundred
/// milliseconds. `scan`, which runs the deadline out, reports both — and the two commands
/// disagreeing about what is in range is the symptom.
///
/// So a name means: keep listening until something answers to it. The cost is that a named command
/// whose robot is out of range pays the full [`SCAN_TIME`] before failing, which is the right trade
/// — that failure's entire content is that the radio looked for eight seconds and found nothing.
fn worth_connecting<T>(
    advertised: &[(T, String)],
    named: &[(T, String)],
    connected: &[(T, String)],
    target: &Target,
) -> bool {
    let Some(wanted) = target.wanted() else {
        return !advertised.is_empty() || !named.is_empty() || !connected.is_empty();
    };
    // All three tiers, though only the first two can hold a match: `connected` is suppressed
    // entirely when a name is wanted, and a named device that is not advertising the service lands
    // in `named` before that tier is reached. Asking the same question of all three is one rule
    // instead of a rule plus the reason the third is exempt from it.
    any_answers(advertised, wanted) || any_answers(named, wanted) || any_answers(connected, wanted)
}

/// Does any of these answer to `wanted`? The one question both the scan loop and the failure below
/// ask, so that "keep listening" and "this is why it failed" cannot disagree about what a match is.
fn any_answers<T>(candidates: &[(T, String)], wanted: &str) -> bool {
    candidates.iter().any(|(_, name)| answers_to(name, wanted))
}

/// Devices as indented lines: what names each one, what it calls itself, where it is, what it is
/// doing.
///
/// Shared by `scan` and by the failure message, because identifying a robot in a list of earbuds is
/// the same problem whether the list is the answer or the diagnosis — and two renderings of it would
/// drift apart exactly where the reader is comparing one run against another.
async fn device_list(mut devices: Vec<&Seen>, target: &Target) -> String {
    // The named robot first, then named devices, then by identity: a device that reported a name is
    // the line worth reading, and sorting keeps a re-run's output comparable with the last one.
    //
    // The default's own line leads, because this list is truncated at `LISTED_DEVICES` and an office
    // holds more devices than that: sorted by identity alone, the one line the reader is looking for
    // lands in "… and 6 more" as often as not, and a marker nobody can see is not an answer.
    devices.sort_by(|a, b| {
        (!target.marks(a.local_name.as_deref()))
            .cmp(&!target.marks(b.local_name.as_deref()))
            .then(a.local_name.is_none().cmp(&b.local_name.is_none()))
            .then(a.identity.cmp(&b.identity))
    });

    let mut lines: Vec<String> = Vec::new();
    for device in devices.iter().take(LISTED_DEVICES) {
        let mut notes: Vec<String> = Vec::new();
        // The leading note, because it is what the line is read for: `scan` is how someone finds the
        // robot to ssh into or point a browser at, and the service count is diagnosis by comparison.
        notes.extend(device.address.note());
        if device.services > 0 {
            notes.push(format!("{} service(s)", device.services));
        }
        // Checked here rather than during the scan: it is one call per device once, instead of one
        // per device per 250ms poll, and nothing before the list is printed needs the answer.
        if device.peripheral.is_connected().await.unwrap_or(false) {
            notes.push("connected".to_owned());
        }
        let notes = if notes.is_empty() {
            String::new()
        } else {
            format!(" — {}", notes.join(", "))
        };
        // The line the reader is looking for, marked. `scan` with a default set is read to answer
        // "is my robot here", and that is otherwise a string comparison done by eye against a list
        // of hex — worse on macOS, where the robot is reported under two names joined together.
        let mark = if target.marks(device.local_name.as_deref()) {
            format!("  ← {}", target.source())
        } else {
            String::new()
        };
        lines.push(format!(
            "  {} {}{notes}{mark}",
            device.identity,
            device.local_name.as_deref().unwrap_or("(no name)"),
        ));
    }

    if devices.len() > LISTED_DEVICES {
        lines.push(format!("  … and {} more", devices.len() - LISTED_DEVICES));
    }
    lines.join("\n")
}

/// Whether the devices that are not robots are listed, or only counted.
///
/// The duck service UUID in the advertisement is the strongest evidence available to a listing —
/// anything better needs a connection, and connecting to 43 devices to ask each whether it is a
/// robot would be minutes of pairing prompts. So that block is not padding: a robot already bonded
/// with this Mac frequently advertises no services at all, and it is the reason `--name` exists.
///
/// But `scan` is read to answer "which robots can I talk to", and a dozen lines of earbuds above
/// the answer buries it. So the block is the diagnosis rather than the output, and it appears when
/// it is one:
///
/// - `--verbose`, which is the flag for asking what the radio actually saw.
/// - **No robot advertised the service**, verbose or not. That is precisely the case where the robot
///   is plausibly in the other list and hiding the evidence would leave nothing to act on.
///
/// Otherwise it is a count and how to expand it, because "that was every device" and "that was the
/// robots" want different next moves.
fn lists_others(verbose: bool, robots: usize) -> bool {
    verbose || robots == 0
}

/// What `scan` prints: the robots, and — per [`lists_others`] — everything else.
async fn listing(seen: &[Seen], verbose: bool, target: &Target) -> String {
    let (robots, others): (Vec<&Seen>, Vec<&Seen>) = seen.iter().partition(|d| d.duck);
    // Kept before `device_list` consumes the vector, since they decide the blocks below.
    let found = robots.len();
    let silent = robots
        .iter()
        .filter(|d| d.address == Address::Unsaid)
        .count();

    let mut out = if robots.is_empty() {
        "no robot advertised the duck service.".to_owned()
    } else {
        format!(
            "{} robot(s) advertising the duck service:\n{}",
            robots.len(),
            device_list(robots, target).await,
        )
    };

    // A robot whose line carries no address at all is on a release from before `btd` broadcast one,
    // and its line cannot say so: an absent field looks the same as a device that never had one. Said
    // once, below the list, where there is room for what to do about it — and only when it happened,
    // because on a bench of current robots this sentence is noise.
    if silent > 0 {
        out.push_str(&format!(
            "\n\n{silent} of them broadcast no address, which is a release from before `btd` \
             advertised one. `duckctl wifi status` still reports it; updating the robot puts it \
             in this list."
        ));
    }

    if !others.is_empty() {
        if lists_others(verbose, found) {
            let anonymous = others.iter().filter(|d| d.local_name.is_none()).count();
            out.push_str(&format!(
                "\n\n{} other device(s) in {SCAN_TIME:?}, {anonymous} with no name. A robot bonded \
                 with this Mac often stops advertising the service to it, so it can be one of \
                 these — `--name <its name>` connects to it anyway:\n{}",
                others.len(),
                device_list(others, target).await,
            ));
        } else {
            out.push_str(&format!(
                "\n\n{} other device(s) in range, not listed. A robot bonded with this Mac can be \
                 among them, advertising no service — `--verbose` lists them.",
                others.len(),
            ));
        }
    }
    out
}

/// Why the scan came back empty, in terms of what the radio actually reported.
///
/// Without this, two failures print the same sentence and want opposite next moves: an empty list is
/// a problem on *this* machine — Bluetooth off, the permission never granted, another scan holding
/// the radio — while a list the robot is missing from points at the robot.
///
/// And the robot can be *in* that list, unrecognisable. `btd` advertises flags (3 bytes), a 128-bit
/// service UUID (18) and the address field (8, see `btd::adv`), which is 29 of the 31 bytes a legacy
/// advertisement holds — so the name never travels in it. It goes in the scan response, a second
/// exchange that can be missed on its own. A device reported with no name and no services is
/// therefore a plausible robot, which is why the unnamed ones are listed rather than filtered out.
async fn nothing_found(seen: &[Seen], target: &Target) -> String {
    if seen.is_empty() {
        return format!(
            "no robot found — and the Mac reported no BLE devices at all in {SCAN_TIME:?}, not one \
             pair of earbuds. That points at this machine rather than the robot: is Bluetooth on, \
             and has this terminal been granted the Bluetooth permission?"
        );
    }

    let missed = match target.wanted() {
        Some(name) => format!(" and nothing was named {name:?}"),
        None => String::new(),
    };
    let mut message = format!(
        "no robot found. Nothing advertised the duck service{missed}. {}",
        radio_saw(seen, target).await,
    );
    // Before the generic advice, because "why is it looking for that name" comes first for a reader
    // who did not type one.
    message.push_str(&target.provenance());
    message.push_str(
        "\nIf the robot is one of the unnamed lines, it was reported without the name and the \
         service UUID this matches on, and retrying usually finds it. If it is absent entirely, \
         `journalctl -u btd -b` on the robot says whether the GATT application is registered.",
    );
    message
}

/// Everything the radio reported, as evidence under a failure.
///
/// The count of unnamed devices is in the summary rather than left to be inferred from the list:
/// named ones sort first, so truncation hides exactly the lines a robot could be hiding in, and "is
/// it plausibly one of those" is the question this list is read to answer.
async fn radio_saw(seen: &[Seen], target: &Target) -> String {
    let anonymous = seen.iter().filter(|d| d.local_name.is_none()).count();
    format!(
        "The Mac saw {} device(s) in {SCAN_TIME:?}, {anonymous} of them with no name:\n{}",
        seen.len(),
        device_list(seen.iter().collect(), target).await,
    )
}

/// What to add under `choose`'s "no robot named …" when nothing answered to the name.
///
/// `choose` is given names and nothing else — deliberately, since the rule it encodes is about
/// names — so its failure can only list the robots that *did* answer. Alone, that reads as "your
/// robot is not here" when the honest claim is narrower: nothing calling itself that was reported
/// in eight seconds. The two ways that happens want opposite next moves, and the list separates
/// them:
///
/// - **The robot is in the list, unnamed.** Its name travels in the scan response, a second
///   exchange that can be missed on its own — [`nothing_found`] has the byte budget that forces
///   that. Retrying finds it.
/// - **The robot is not in the list at all.** Nothing this client did can explain that: `btd`
///   advertises every 100–150ms, so eight seconds is fifty missed chances, and the robot was not
///   advertising. That is a fact about the robot rather than about the search — and a phone looking
///   for the same robot would find it just as absent.
async fn missed_the_named_robot(seen: &[Seen], target: &Target) -> String {
    let mut message = radio_saw(seen, target).await;
    message.push_str(
        "\n\nIf the robot is one of the unnamed lines, its name was in a scan response this scan \
         missed, and retrying usually finds it. If it is absent from that list entirely, it was \
         not advertising for the whole eight seconds — check `journalctl -u btd -b` on the robot, \
         and note that a robot stops advertising while a central is connected to it, so a link \
         left over from the previous command can be the reason.",
    );
    message
}

/// Run one step with a budget, naming it if the budget runs out.
async fn step<T>(
    what: &str,
    hint: &str,
    budget: Duration,
    f: impl std::future::Future<Output = Result<T, btleplug::Error>>,
) -> Result<T, Box<dyn std::error::Error>> {
    match tokio::time::timeout(budget, f).await {
        Ok(Ok(value)) => Ok(value),
        Ok(Err(e)) => Err(format!("{what} failed: {e}\n{hint}").into()),
        Err(_) => Err(format!("{what} timed out after {budget:?}\n{hint}").into()),
    }
}

#[derive(Parser)]
#[command(
    // Spelled out because clap would otherwise take it from the crate, and `--version` on the
    // installed binary answered `btd 0.5.1` — the daemon's name, for the laptop-side client.
    name = "duckctl",
    version,
    about = "Talk to a robot over BLE — the phone app's stand-in",
    long_about = "Finds a robot advertising the duck GATT service and speaks the same JSON-RPC \
                  lines every other transport uses. This is a development tool, and nothing on a \
                  robot depends on it, so it never ships to one."
)]
struct Cli {
    /// Connect to this robot by advertised name. Without it, `DUCK_ROBOT`; without that, the first
    /// robot found wins.
    ///
    /// The advertised name, which is what `system.info` reports and what `name` below sets: a
    /// board that has never been renamed answers to its derived default, `duck-7f3a`.
    ///
    /// `export DUCK_ROBOT=duck-c51b` in a shell profile makes that the robot every command talks
    /// to. `DUCK_ROBOT= duckctl …` ignores it for one command.
    //
    // The id is spelled out rather than derived from the field, because clap keys arguments by id
    // and the `name` subcommand has a positional argument that derives the same one. With both
    // called `name` the positional won, so `--name duck-c51b name leduckpierre` searched for
    // `leduckpierre` — the name it was about to set — and then reported the robot standing in
    // front of it as out of range. `value_name` keeps the help line reading `--name <ROBOT_NAME>`
    // rather than leaking the id into it.
    #[arg(long = "name", id = "robot", value_name = "ROBOT_NAME", global = true)]
    name: Option<String>,

    /// Print every line sent and received, and have `scan` list every device rather than the robots.
    #[arg(long, global = true)]
    verbose: bool,

    /// The robot's pairing PIN. Defaults to `DUCK_PIN`, then to `000000`.
    ///
    /// Six digits, shown by `robotctl system pin` on the robot. The factory default is `000000`
    /// and authenticates anyone who has read this repository, which is why a shipped robot needs a
    /// per-robot one — and why `export DUCK_PIN=…` is worth more than the name is.
    //
    // No `default_value`, because the default has to be applied *after* the environment or it would
    // shadow it: clap fills a `default_value` in and nothing downstream can then tell `000000` typed
    // from `000000` assumed. It is spelled out in the help text above instead.
    #[arg(long, global = true)]
    pin: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List robots in range with the address each one broadcast, and stop.
    Scan,
    /// The robot's IPv4 address on stdout, and nothing else.
    ///
    /// `ssh radxa@$(duckctl ip)`. Read from the advertisement `btd` already broadcasts, so no
    /// connection is made, no bond is needed and no PIN can be wrong — and it costs about a second
    /// rather than the tens `wifi status` does. A robot that is bonded to this machine and has
    /// stopped advertising the service to it is asked over BLE instead, which is slower and always
    /// answers.
    Ip,
    /// Open the robot's console in a browser.
    ///
    /// The page `mediad` serves: the camera, and the controls a WebRTC peer is allowed to drive.
    /// Finds the robot the way `ip` above does.
    Open {
        /// Print the URL instead of opening it — for a machine with no browser, or a script.
        #[arg(long)]
        print: bool,
        /// The console's port, for a robot started with a non-default `--web-port`.
        //
        // The same default as `mediad --web-port`, and the reason this command exists rather than a
        // documented `open "http://$(duckctl ip):8080"`: the port belongs in one place that nobody
        // has to read.
        #[arg(long, default_value_t = 8080)]
        port: u16,
    },
    /// Version handshake plus update status.
    Status,
    /// What the robot is running: the API version, the release, and the revision it was built from.
    ///
    /// `status` above answers this too, buried in an update report. This is the narrower question,
    /// and the first one to ask of a robot that is behaving unexpectedly — a `revision` of `null`
    /// means the release was built on somebody's laptop rather than by CI.
    Version,
    /// Updates: what is available, installing it, and going back.
    #[command(subcommand)]
    Update(Update),
    /// Name, serial and uptime.
    Info,
    /// Is the control loop healthy?
    Health,
    /// Wifi.
    #[command(subcommand)]
    Wifi(Wifi),
    /// Rename the robot.
    Name {
        /// What to call it from now on. `--name` above still names the robot to rename.
        #[arg(value_name = "NEW_NAME")]
        name: String,
    },
    /// Run a one-shot skill — a kick, the roulade, a bow.
    ///
    /// `robotctl robot do` on the robot. The robot has to be driving: press Start on the pad
    /// first, or it answers saying so. It needs no pad *input* though — the deadman zeroes the
    /// twist by itself, so a robot nobody is steering stands still and does the thing.
    ///
    /// `duckctl policy list` names the skills this robot has. They are config, so the list
    /// differs between robots and an unknown name is refused with the real one.
    Do {
        #[arg(value_name = "SKILL")]
        skill: String,
    },
    /// The Hugging Face account this robot belongs to.
    ///
    /// Signing in over Bluetooth is what a robot fresh out of a box needs: it has no network, so
    /// it has no console and no LAN to open one from. `login` prints a short code to approve on
    /// huggingface.co from any device — this tool does not have to stay connected while you do,
    /// which is the point of the flow. `status` afterwards says whether it worked.
    #[command(subcommand)]
    Account(Account),

    /// What each policy slot is running, and what to run instead.
    #[command(subcommand)]
    Policy(Policy),
    /// Which pad button runs which skill.
    #[command(subcommand)]
    Pad(Pad),
    /// Reboot it.
    Reboot,
    /// Send any method, for whatever is not wrapped above.
    Call {
        method: String,
        /// Parameters as JSON. Defaults to `{}`.
        params: Option<String>,
    },
}

/// The binding half of `robotctl pad`, named the same way.
///
/// Pairing a gamepad is not here: `pad.pair` and `pad.forget` are `configd`'s and reached with
/// `call`. These two are `robotd`'s, because checking a skill name needs the list of skills, and
/// that is the daemon with it.
#[derive(Subcommand)]
enum Pad {
    /// What each of the five one-shot buttons runs.
    ///
    /// `overridden` marks what somebody changed; `error` marks a button bound to a skill this
    /// robot does not have, which is a button that will do nothing when pressed.
    Bindings,
    /// Put a skill on a button.
    ///
    /// Takes effect within a second — `padd` re-reads the file, and nothing restarts. The name is
    /// checked against this robot's skills first, so a typo is refused with the real list rather
    /// than becoming a dead button.
    Bind {
        /// `a`, `x`, `lb`, `rb` or `dpad_down` — the bumpers, not the analog triggers.
        button: String,
        /// A skill this robot has, or `""` to leave the button doing nothing.
        skill: String,
    },
    /// Put a button back to what this robot ships with.
    Reset {
        /// Which button.
        button: String,
    },
}

/// `duckctl account …` — the account half of `robotctl account`, over the radio.
#[derive(Subcommand, Debug)]
enum Account {
    /// Start a login, and open the page to approve it on.
    Login {
        /// Print the code and the URL, and open nothing.
        ///
        /// Also the automatic behaviour when stderr is not a terminal, so a script does not
        /// launch a browser on somebody's desktop.
        #[arg(long)]
        no_open: bool,
        /// Sign in even though this robot already belongs to an account, or is already
        /// waiting for a code to be approved.
        #[arg(long)]
        force: bool,
    },
    /// Which account this robot belongs to, and whether a login is waiting for approval.
    Status,
    /// Forget the account.
    Logout,
}

/// The policy commands, named as `robotctl policy` names them.
///
/// Same words in the same order as on the robot, for the reason [`Update`] gives. What is missing
/// against `robotctl` is the Hub: `policy.check`, `policy.install` and `policy.search` are not
/// served over this transport — they reach the network on the robot's behalf and write to the
/// eMMC — so `load` here takes a path to a file already on the robot, never `org/repo`.
#[derive(Subcommand)]
enum Policy {
    /// What each slot is running, from where, and which skills this robot has.
    List,
    /// Put a policy in a slot, live.
    ///
    /// The robot returns to its home pose with torque on, loads it, and drives again. A file that
    /// is not `obs[1,61] -> actions[1,14]` is refused before anything changes, and a load that
    /// fails anyway keeps the policy that was running.
    Load {
        /// `walk`, `stand`, `sitstand`, `ground_pick`, `kick_left`, `kick_right` or `roulade`.
        slot: String,
        /// Absolute path on the robot. Relative is refused: the daemon's working directory is
        /// not the caller's, and a path meaning one file to each would be worse than a refusal.
        path: String,
    },
    /// Put one slot back to what this robot ships with.
    ///
    /// One slot, because the wire call takes one. Resetting every slot at once is
    /// `robotctl policy reset` on the robot.
    Reset {
        /// Which slot to clear.
        slot: String,
    },
    /// What else is published for this robot.
    ///
    /// Reaches the Hub, changes nothing. `microduck` is the useful query until there is a tag.
    Search {
        /// What to look for on the Hub.
        #[arg(default_value = "microduck")]
        query: String,
    },
    /// Is there a newer official policy set? Changes nothing.
    Check,
    /// Install an official policy set from the Hub.
    ///
    /// Takes the newest unless a revision is named. The robot returns to its home pose, re-reads
    /// every slot and drives again — and a slot pointing at a file you loaded yourself is left
    /// alone, because it points somewhere else entirely.
    Update {
        /// A tag like `v2`, or a branch. Omit for the newest.
        #[arg(long)]
        version: Option<String>,
    },
    /// Download somebody else's policy onto the robot, without loading it.
    ///
    /// **Not `robotctl policy add`**, which also writes a skill entry so `robot do` can run it —
    /// that needs `[[policy.skill]]`, and there is no wire method for it. This is the download
    /// half only; `policy load <slot> <path>` afterwards runs it in a slot, and the reply names
    /// the path to give it.
    ///
    /// Nothing a stranger publishes is verified by anybody: what makes it safe to try is the
    /// shape gate, the joint clamps and the fall reflex. Have the robot on its stand the first
    /// time.
    Fetch {
        /// `org/repo` on the Hub.
        repo: String,
        /// A revision in that repo. Omit for the default branch.
        #[arg(long)]
        revision: Option<String>,
        /// Which file, for a repo carrying more than one policy.
        #[arg(long)]
        file: Option<String>,
    },
    /// What this robot can be asked to do, and the timings behind each one.
    ///
    /// `built_in` names the two `robot.do` also answers to that are not table entries —
    /// `ground_pick` and `sit_toggle` are driven by the robot itself.
    Skills,
    /// Give a fetched policy a name, so `robot do` can run it.
    ///
    /// The other half of `policy fetch`: that downloads a file, this makes it a skill.
    /// One call — the robot writes its config and reloads, so nothing restarts.
    Skill {
        /// What `robot do` will answer to. An existing name replaces that entry.
        name: String,
        /// Seconds it runs. Required the first time; kept if omitted afterwards.
        #[arg(long)]
        duration: Option<f64>,
        /// The `.onnx` on the robot — the path `policy fetch` reported.
        #[arg(long)]
        path: Option<String>,
        /// The twist fed to the network while it runs, as `vx,vy,vyaw`. Zeros unless a policy
        /// reads its twist as something else — flamingo's is `[flag, side, 0]`.
        #[arg(long)]
        command: Option<String>,
        /// The twist driven on the way back, for a policy that does not end itself.
        #[arg(long)]
        unwind: Option<String>,
        /// Seconds spent coming back.
        #[arg(long)]
        unwind_s: Option<f64>,
    },
    /// Take a skill out. One this robot's release ships comes back.
    Unskill { name: String },
    /// Re-read every slot from the config file.
    ///
    /// For when something else changed what the robot should be running — a policy set installed
    /// underneath unchanged paths, or a hand-edited `robotd.toml`. `load` would correctly
    /// conclude there was nothing to do.
    Reload,
}

/// The update commands, named as `robotctl update` names them.
///
/// Deliberately the same words in the same order, so what someone learns on the robot transfers to
/// the radio and back. What differs is the component: `robotctl` takes it as a positional argument
/// because an operator may be updating a model bundle, and here it is a flag with a default,
/// because a phone has one component to care about and today a robot has exactly one.
#[derive(Subcommand)]
enum Update {
    /// Is there a newer release? Changes nothing.
    ///
    /// Answers `BUSY` rather than waiting if an update is already running.
    Check {
        #[arg(long, default_value = "daemon")]
        component: String,
    },
    /// Install a release, and report progress while it happens.
    ///
    /// Answers once, when it is finished. The connection then drops a few seconds later, because
    /// installing a daemon release restarts `btd` — see what this prints after the reply.
    Apply {
        #[arg(long, default_value = "daemon")]
        component: String,
        /// Exact version to install. Omit for whatever the source calls latest.
        #[arg(long, conflicts_with = "git_ref")]
        version: Option<duck_ipc_proto::semver::Version>,
        /// Install what a branch last built, e.g. `--ref my-branch`.
        ///
        /// A dev build, so a robot only accepts one if the team key is in its trusted set and
        /// `allow_dev_keys` is on: a customer robot refuses it.
        #[arg(long = "ref", value_name = "REF", conflicts_with = "version")]
        git_ref: Option<String>,
        /// Install the release candidate from the staging channel.
        ///
        /// Pair it with `--version` to name one candidate rather than the newest.
        #[arg(long, conflicts_with = "git_ref")]
        staging: bool,
        /// Verify everything, then stop before the symlink swap.
        #[arg(long)]
        dry_run: bool,
    },
    /// Per-component state: the version installed, the phase, health, and the last attempt.
    ///
    /// The same call as the top-level `status`, which keeps working because it is in every set of
    /// notes anybody has written down.
    Status,
    /// Which releases are on the board, and which one is active.
    Versions {
        #[arg(long, default_value = "daemon")]
        component: String,
    },
    /// Recent update attempts and outcomes — the record that survives a wiped journal.
    Log {
        #[arg(long, default_value_t = 20)]
        limit: u32,
    },
    /// Go back to the release installed before this one.
    ///
    /// The undo, for a release that installed, passed its health gate and then behaved worse.
    /// Discards nothing, and is gated and auto-reverted like any other transition.
    Rollback {
        #[arg(long, default_value = "daemon")]
        component: String,
    },
    /// Activate a release already on the board, without downloading anything.
    ///
    /// `versions` above lists what there is to choose from. Gated like an apply, so a selection
    /// that does not come up is reverted.
    Select {
        version: duck_ipc_proto::semver::Version,
        #[arg(long, default_value = "daemon")]
        component: String,
    },
    /// Follow progress until interrupted.
    ///
    /// Prints where any update in flight has got to — `updaterd` replays the latest progress to a
    /// new subscriber — and then everything that follows. It never receives a reply, so it ends
    /// with Ctrl-C.
    Watch,
}

#[derive(Subcommand)]
enum Wifi {
    /// What the wifi is doing — SSID, signal, addresses.
    Status,
    /// Networks the robot can see.
    Scan,
    /// Join a network.
    Connect {
        ssid: String,
        /// Omit for an open network.
        #[arg(long)]
        psk: Option<String>,
    },
    /// Forget a stored network.
    Forget { ssid: String },
}

/// Print the error rather than returning it, and that is not a style preference.
///
/// A `main` returning `Err` is reported by Rust's `Termination` impl, which **`Debug`-formats** the
/// error: every hint in this file is multi-line, and `Debug` on a string renders the newlines as
/// literal `\n` and wraps the lot in quotes. So the guidance written to be read as lines arrived as
/// one escaped blob — worst for the failure that lists what the radio saw, which is a dozen lines.
#[tokio::main]
async fn main() -> std::process::ExitCode {
    match run().await {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            std::process::ExitCode::FAILURE
        }
    }
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    // Read once, here, so everything below asks `target` rather than the environment: which robot
    // was chosen and who chose it is one decision, and a second reader of `DUCK_ROBOT` could
    // disagree with the first.
    let target = Target::new(cli.name.clone(), std::env::var("DUCK_ROBOT").ok());
    let pin = resolve_pin(cli.pin.clone(), std::env::var("DUCK_PIN").ok());

    // `scan` shares the discovery below and then stops, because a listing and a search look for the
    // same thing and differ only in what they do with it. It connects to nothing at all: that is
    // what makes it the safe command to reach for when a robot cannot be reached, and it is also why
    // it can only report what an advertisement carries.
    let list_only = matches!(cli.command, Command::Scan);
    // `ip` and `open` want one field out of an advertisement, so they read it the way `scan` does —
    // and unlike `scan` they connect after all when no advertisement carried one. Cheap read first,
    // call second: without the fallback these two commands would fail on exactly the laptops that
    // use them most, because a robot bonded to this Mac often stops advertising the service to it.
    let resolving = matches!(cli.command, Command::Ip | Command::Open { .. });

    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no Bluetooth adapter on this machine")?;

    // Disclosed before the eight seconds rather than in the failure afterwards: a search nobody
    // typed is worth saying out loud, and this is also how somebody who set `DUCK_ROBOT` months ago
    // finds out it is still in force.
    if let Some(name) = target.wanted().filter(|_| target.from_env) {
        eprintln!("looking for {name:?} — `DUCK_ROBOT` in this shell's environment");
    }
    eprintln!("scanning for up to {SCAN_TIME:?}…");
    // **Unfiltered on purpose.** This used to pass `ScanFilter { services: [SERVICE_UUID] }`, on the
    // theory that a busy office would otherwise drown the robot in headphones. But CoreBluetooth
    // honours that filter *strictly*: a peripheral whose current advertisement does not carry the
    // UUID is never reported at all. A bonded robot frequently reports with an empty service list —
    // so the `--name` fallback below could only ever match something the filtered scan had already
    // returned, which made it dead weight in exactly the case it exists for. That is the whole
    // explanation for `no robot found` on one run and success on the next.
    //
    // So: report everything, and discriminate here, where the rules are ours.
    adapter.start_scan(ScanFilter::default()).await?;

    // Candidates, strongest evidence first.
    //
    // The advertised service UUID is an *optimisation*, not the identity check — and treating it as
    // the latter broke as soon as the Mac bonded with the robot. The authoritative test is whether
    // it serves our characteristic, which is only knowable after connecting.
    let mut advertised: Vec<(Peripheral, String)> = Vec::new();
    // What each robot said about its address, beside the name it said it under — the two fields
    // `choose` needs, so `ip` inherits the collision rule every other command follows rather than
    // picking whichever robot the radio reported first.
    let mut addresses: Vec<(Address, String)> = Vec::new();
    let mut named: Vec<(Peripheral, String)> = Vec::new();
    let mut connected: Vec<(Peripheral, String)> = Vec::new();
    // Everything the Mac reported, kept only so a failure can say what was in range. `configd`
    // learned this on the other side — a failed `pad pair` lists what the radio saw, because the
    // escape hatch needs an address nobody has otherwise.
    let mut seen: Vec<Seen> = Vec::new();
    let deadline = Instant::now() + SCAN_TIME;

    loop {
        advertised.clear();
        addresses.clear();
        named.clear();
        connected.clear();
        // Cleared with the tiers, and rebuilt from the same sweep: `peripherals()` reports
        // everything known to this scan session rather than only what arrived since the last poll,
        // so the final sweep is the fullest one.
        seen.clear();

        for peripheral in adapter.peripherals().await? {
            let Some(properties) = peripheral.properties().await? else {
                continue;
            };
            let name = properties
                .local_name
                .clone()
                .unwrap_or_else(|| properties.address.to_string());

            let duck = properties.services.contains(&SERVICE_UUID);
            let address = Address::read(&properties, duck);
            if duck {
                addresses.push((address, name.clone()));
            }
            seen.push(Seen {
                peripheral: peripheral.clone(),
                identity: identity(&peripheral, properties.address),
                local_name: properties.local_name.clone(),
                services: properties.services.len(),
                duck,
                address,
            });

            if list_only {
                // A listing connects to nothing, so the tiers — which exist to choose what to
                // connect to — have no work to do, and the `is_connected` call below would cost one
                // round trip per device per poll for an answer nothing reads.
                continue;
            }

            if duck {
                advertised.push((peripheral, name));
            } else if target.wanted().is_some_and(|w| answers_to(&name, w)) {
                named.push((peripheral, name));
            } else if target.wanted().is_none() && peripheral.is_connected().await? {
                // Last resort, and only without a name: an unfiltered scan sees every connected
                // peripheral on the Mac, so this tier is full of keyboards and earbuds. Each one
                // costs a connect and a service discovery before it can be ruled out, which is why
                // an explicit name suppresses the tier entirely rather than being merged into it.
                connected.push((peripheral, name));
            }
        }

        // A listing is the exception, and runs the deadline out: stopping at the first robot would
        // report one and hide the second, which is the only question worth asking in a room with
        // three of them.
        if (!list_only && worth_connecting(&advertised, &named, &connected, &target))
            || Instant::now() >= deadline
        {
            break;
        }
        tokio::time::sleep(SCAN_POLL).await;
    }
    let _ = adapter.stop_scan().await;

    if list_only {
        // Nothing at all is a fault on this machine rather than a report about robots, and
        // `nothing_found` is where that diagnosis lives. An error, so the exit status says so too.
        if seen.is_empty() {
            return Err(nothing_found(&seen, &target).await.into());
        }
        println!("{}", listing(&seen, cli.verbose, &target).await);
        return Ok(());
    }

    // The advertisement, before anything connects. An address here costs no bond, no PIN and about a
    // second — and it is not stale: `btd` re-reads `net.status` every five seconds and re-advertises
    // when the answer moves, so this is that call with a five-second lag, well inside the window in
    // which a new lease has already broken ssh.
    if resolving {
        match choose(std::mem::take(&mut addresses), &target) {
            Ok((Address::At(address), _)) => return deliver(&cli.command, &address.to_string()),
            // The robot broadcast `0.0.0.0`, which is a robot with no network rather than a robot
            // that did not say. Asking `net.status` over a connection would return the same nothing
            // more slowly, so this answers now.
            Ok((Address::Unassigned, name)) => return Err(no_address(&name).into()),
            // A release from before `btd` advertised an address. The fallback answers anyway;
            // updating makes it fast.
            Ok((Address::Unsaid, name)) => eprintln!(
                "{name} advertises no address — a release from before robots broadcast one. Asking \
                 it over Bluetooth instead, which takes longer; `duckctl update apply` makes this \
                 fast."
            ),
            // Nothing advertised the service, or nothing answering to the name did. Both are the
            // fallback's case, and the tiers below have their own answer for each: this is the
            // bonded-Mac situation the fallback exists for, and its failure message is better than
            // anything that could be said here.
            Err(_) => {
                if cli.verbose {
                    eprintln!(
                        "no advertisement carried an address; connecting to ask net.status instead"
                    );
                }
            }
        }
    }

    let mut found = advertised;
    if found.is_empty() && !named.is_empty() {
        if cli.verbose {
            eprintln!(
                "nothing advertised the service; trying {} peripheral(s) matching the name — a \
                 bonded robot often stops advertising it to a Mac that has already paired",
                named.len()
            );
        }
        found = named;
    } else if found.is_empty() && !connected.is_empty() {
        if cli.verbose {
            eprintln!(
                "nothing advertised the service; trying {} already-connected peripheral(s), which \
                 may well be earbuds. `--name <robot name>`, or `DUCK_ROBOT`, skips this guesswork",
                connected.len()
            );
        }
        found = connected;
    }

    if found.is_empty() {
        return Err(nothing_found(&seen, &target).await.into());
    }

    // Whether the name matched nothing, as against matching too much: only the first wants the list
    // of everything the radio saw under it. A collision is about two robots that are both right
    // there, and fifty lines of earbuds under it would bury the two names that matter.
    let missed = target
        .wanted()
        .is_some_and(|wanted| !any_answers(&found, wanted));

    let (peripheral, name) = match choose(found, &target) {
        Ok(chosen) => chosen,
        Err(why) if missed => {
            return Err(
                format!("{why}\n\n{}", missed_the_named_robot(&seen, &target).await).into(),
            );
        }
        Err(why) => return Err(why.into()),
    };
    eprintln!("connecting to {name}…");

    step(
        "connecting",
        "The robot advertised but would not accept a connection. If macOS shows it as paired, \
         forget it there and retry; `sudo pkill bluetoothd` also clears a half-finished bond.",
        CONNECT_TIMEOUT,
        peripheral.connect(),
    )
    .await?;
    if cli.verbose {
        eprintln!("connected; discovering services…");
    }

    step(
        "service discovery",
        "Connected, but the robot never described its services. Check `journalctl -u btd -b` on \
         the robot for whether the GATT application is registered.",
        DISCOVER_TIMEOUT,
        peripheral.discover_services(),
    )
    .await?;

    let (request, response) = characteristics(&peripheral)?;

    // Read first, and this is load-bearing rather than a courtesy.
    //
    // The robot requires an authenticated encrypted link to *write*, but a subscribe needs no
    // encryption — so without this a central subscribes happily, has its first write refused, and
    // on macOS sees neither a prompt nor an error. A read is acknowledged, so an unpaired link
    // fails here instead, which is what makes CoreBluetooth start pairing.
    //
    // The value is the robot's API version, and it is reported rather than enforced. See the
    // mismatch warning below for why this tool refuses nothing on it.
    let read = step(
        "reading the API version",
        "This read requires an encrypted link, so it is what triggers pairing. A hang here usually \
         means the bond did not complete: forget the robot in macOS Bluetooth settings, or run \
         `sudo pkill bluetoothd`, and retry.",
        READ_TIMEOUT,
        peripheral.read(&response),
    )
    .await;

    match read {
        Ok(value) => {
            let theirs = value.first().copied().unwrap_or(0);
            if cli.verbose {
                eprintln!("robot speaks API v{theirs}");
            }
            if u32::from(theirs) != duck_ipc_proto::API_VERSION {
                warn_about_skew(theirs);
            }
        }
        Err(e) => return Err(e),
    }

    // Subscribe *before* writing, or a reply can arrive before there is anywhere to put it.
    // btd's session begins on the first write, so the order here is not merely defensive: the
    // notify half has to exist for the session to have somewhere to answer.
    peripheral.subscribe(&response).await?;
    let mut notifications = peripheral.notifications().await?;

    // Prove the PIN before anything else. The bond is just-works, so it encrypts the link and
    // authenticates nobody; the robot serves nothing until this succeeds. See
    // `btd/src/pairing.rs` for why the check is here rather than in the pairing.
    let auth = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "system.authenticate",
        "params": { "pin": pin },
    });
    let auth = serde_json::to_string(&auth)?;
    if cli.verbose {
        // The PIN is deliberately not printed, even here: a terminal is a log too.
        eprintln!("→ system.authenticate (pin redacted)");
    }
    write_line(&peripheral, &request, &auth).await?;

    let reply = read_line(&mut notifications, REPLY_TIMEOUT).await?;
    if cli.verbose {
        eprintln!("← {reply}");
    }
    let parsed: serde_json::Value = serde_json::from_str(&reply)?;
    if parsed["result"]["authenticated"] != serde_json::json!(true) {
        let left = parsed["result"]["attempts_remaining"].as_u64();
        return Err(match left {
            Some(0) => "wrong PIN, and no attempts left — the robot closed the session. \
                        Check it with `robotctl system pin` on the robot."
                .to_owned()
                .into(),
            Some(n) => format!(
                "wrong PIN ({n} attempt(s) left). Check it with `robotctl system pin` on the robot."
            )
            .into(),
            None => format!("authentication failed: {reply}").into(),
        });
    }

    let (line, timeout) = request_line(&cli.command)?;
    if cli.verbose {
        eprintln!("→ {line}");
    }

    // Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so
    // 20 bytes — the floor every BLE link guarantees — is the safe assumption. Slower than
    // necessary on a good link, and correct on every link.
    for chunk in framing::chunks(&line, 20) {
        peripheral
            .write(&request, &chunk, WriteType::WithoutResponse)
            .await?;
    }

    let mut reassembler = Reassembler::new();
    // The deadline is **idle**, not total: it is pushed back by every notification that arrives,
    // because a robot sending progress is a robot that is working. See `REPLY_TIMEOUT`.
    let mut deadline = tokio::time::Instant::now() + timeout;

    loop {
        let notification = match next_chunk(&peripheral, &mut notifications, deadline).await {
            Waited::Chunk(notification) => notification,
            Waited::Dropped => return Err(dropped(&cli.command).into()),
            Waited::Silent => return Err(silence(timeout).into()),
        };
        deadline = tokio::time::Instant::now() + timeout;

        for line in reassembler.push(&notification.value)? {
            if cli.verbose {
                eprintln!("← {line}");
            }
            // Notifications with no `id` are a progress stream, not an answer; report them and
            // keep waiting for the response that closes the call.
            let value: serde_json::Value = serde_json::from_str(&line)?;
            let is_answer = value.get("id").is_some_and(|id| !id.is_null());

            if !is_answer {
                if value["method"] == "update.progress" {
                    eprintln!("· {}", progress_line(&value["params"]));
                } else {
                    // Nothing else streams today. Printed whole rather than summarised, because a
                    // notification this tool does not know about is worth seeing in full.
                    eprintln!("· {}", serde_json::to_string(&value)?);
                }
                continue;
            }

            // `ip` and `open` asked `net.status` for one field, so the reply is not the answer:
            // stdout carries an address or a URL, or nothing at all. Before the JSON is printed,
            // because printing it would be the bug — `$(duckctl ip)` would carry the whole object.
            if resolving {
                let _ = peripheral.disconnect().await;
                if let Some(error) = value.get("error") {
                    return Err(format!(
                        "the robot answered and refused net.status: {error}\nA wrong PIN is the \
                         usual cause — `robotctl system pin` on the robot says what it is."
                    )
                    .into());
                }
                return match value["result"]["ip4"].as_str().filter(|ip| !ip.is_empty()) {
                    Some(address) => deliver(&cli.command, address),
                    None => Err(no_address(&name).into()),
                };
            }

            println!("{}", serde_json::to_string_pretty(&value)?);
            let _ = peripheral.disconnect().await;
            // A JSON-RPC error is the robot answering, not this tool failing — so it is
            // printed above and reported through the exit status rather than as a panic.
            return if value.get("error").is_some() {
                Err("the robot returned an error".into())
            } else {
                // After the reply, and only for one that succeeded: a rename that the robot
                // refused leaves nothing stale.
                if let Some(note) = target.stale_after_rename(&cli.command) {
                    eprintln!("{note}");
                }
                if let Some(note) = restart_note(&cli.command, &value) {
                    eprintln!("{note}");
                }
                if let Some(note) = account_note(&cli.command, &value) {
                    eprintln!("{note}");
                }
                Ok(())
            };
        }
    }
}

/// How a wait for the next notification ended.
///
/// Three outcomes rather than two, because "nothing arrived" hides the one that is diagnosable:
/// a robot that is connected and not talking wants a different next move from a robot this Mac is
/// no longer connected to.
enum Waited {
    /// Bytes arrived. Whether they complete a line is the reassembler's business.
    Chunk(btleplug::api::ValueNotification),
    /// The link is gone, so no budget is worth waiting out.
    Dropped,
    /// The budget expired with the link still up.
    Silent,
}

/// Wait for the next notification, giving up when the deadline passes *or* the link goes.
///
/// The stream alone cannot report the second: on macOS a peripheral that disconnects mid-call
/// leaves `notifications()` pending rather than ending it, so the only way to learn the link is
/// gone is to ask. Hence the poll — [`LINK_POLL`] at a time, until one of the three answers.
async fn next_chunk(
    peripheral: &Peripheral,
    notifications: &mut (impl futures::Stream<Item = btleplug::api::ValueNotification> + Unpin),
    deadline: tokio::time::Instant,
) -> Waited {
    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Waited::Silent;
        }

        match tokio::time::timeout(remaining.min(LINK_POLL), notifications.next()).await {
            Ok(Some(notification)) => return Waited::Chunk(notification),
            // The stream ended: btleplug has given up on the peripheral, which is the same news
            // the poll below goes looking for.
            Ok(None) => return Waited::Dropped,
            // Nothing yet, so ask whether there ever will be. Only a definite "no" ends the
            // wait: an adapter that errors on the question has not said the link is down, and
            // treating that as a drop would end a working call early — whereas believing a link
            // that is gone costs at most the silence this already tolerated.
            Err(_) => {
                if matches!(peripheral.is_connected().await, Ok(false)) {
                    return Waited::Dropped;
                }
            }
        }
    }
}

/// What to say when the link goes before the reply does.
///
/// A different diagnosis from [`silence`], and the difference matters most for an update: `btd` is
/// restarted about five seconds after an apply answers (`docs/design/restart-order.md` §1), so a
/// drop is one of the shapes a *successful* update has. Calling that "the robot has stopped
/// answering" describes a robot that is working perfectly as a robot that is dead.
fn dropped(command: &Command) -> String {
    // The transitions that restart daemons, and only for the release that ships `btd` — the same
    // two conditions as `restart_note`, because they describe the same event: that one predicts
    // the drop before it happens, and this one explains it afterwards.
    let restarting = matches!(
        command,
        Command::Update(
            Update::Apply { component, .. }
            | Update::Rollback { component }
            | Update::Select { component, .. },
        ) if component == "daemon"
    );

    let next = if restarting {
        "An update restarts the robot's daemons, `btd` among them, so this is as likely to be the \
         update finishing as failing. Reconnect and run `duckctl update status`: \
         `last_attempt` carries the outcome of what ran."
    } else {
        "Reconnect and try again. Anything the robot had already started — an update in \
         particular — carries on without this connection."
    };
    format!("the link to the robot dropped before it answered. {next}")
}

/// What to say when the robot stops talking, which depends on what was expected of it.
///
/// The budget is a silence rather than a total, so "no reply within 180s" would be a lie about an
/// update that had been running for ten minutes and then stalled. Reached only with the link still
/// up: a drop is [`dropped`], and answered as soon as [`LINK_POLL`] notices it.
fn silence(idle: Duration) -> String {
    format!(
        "nothing from the robot for {idle:?}, so it has stopped answering. Anything it had \
             already started — an update in particular — carries on without this connection: \
             reconnect and run `duckctl update status`."
    )
}

/// Write one NDJSON line, chunked.
///
/// Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so 20
/// bytes — the floor every BLE link guarantees — is the safe assumption: slower than necessary on a
/// good link, correct on every link.
///
/// **Acknowledged writes**, and that is not a detail. An ATT Write *Command* (`WithoutResponse`)
/// carries no reply, so a refusal — for insufficient encryption, say — is invisible: the request
/// silently never arrives and the client waits out its timeout with no idea why. That is exactly
/// how this first behaved, against a robot that was working perfectly.
async fn write_line(
    peripheral: &Peripheral,
    characteristic: &Characteristic,
    line: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    for chunk in framing::chunks(line, 20) {
        peripheral
            .write(characteristic, &chunk, WriteType::WithResponse)
            .await?;
    }
    Ok(())
}

/// Read one complete NDJSON line from the notification stream.
async fn read_line(
    notifications: &mut (impl futures::Stream<Item = btleplug::api::ValueNotification> + Unpin),
    timeout: Duration,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut reassembler = Reassembler::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(format!("no reply within {timeout:?}").into());
        }
        let Ok(Some(notification)) = tokio::time::timeout(remaining, notifications.next()).await
        else {
            return Err(format!("no reply within {timeout:?}").into());
        };
        if let Some(line) = reassembler.push(&notification.value)?.into_iter().next() {
            return Ok(line);
        }
    }
}

/// Find the two characteristics, and check they can do what we need.
///
/// Checking the properties rather than assuming them turns a confusing silence — a write that
/// lands nowhere — into a clear message naming which half is wrong.
fn characteristics(
    peripheral: &Peripheral,
) -> Result<(Characteristic, Characteristic), Box<dyn std::error::Error>> {
    let all = peripheral.characteristics();
    let find = |uuid| all.iter().find(|c| c.uuid == uuid).cloned();

    let rpc =
        find(RPC_UUID).ok_or("the robot has no RPC characteristic; is this the right service?")?;

    // One characteristic carries both directions, so it must be able to do both. Checking rather
    // than assuming turns a confusing silence — a write that lands nowhere — into a message
    // naming which half is missing.
    if !rpc
        .properties
        .intersects(CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE)
    {
        return Err("the RPC characteristic is not writable".into());
    }
    if !rpc.properties.contains(CharPropFlags::NOTIFY) {
        return Err("the RPC characteristic cannot notify".into());
    }
    // Cloned rather than borrowed twice: btleplug takes a &Characteristic for each operation.
    Ok((rpc.clone(), rpc))
}

/// One command becomes one JSON-RPC line, plus how long to wait for it.
/// Say that the two ends were not built together, and carry on.
///
/// **This used to be a refusal, and the refusal was wrong twice over.**
///
/// It was wrong about what it was reading. `API_VERSION` is an agreement between the binaries on
/// one board — `robotctl` and `updaterd` come from one release, and `updaterd`'s exact `!=` on
/// `Hello` is what enforces it. A laptop is not a binary on the board, and will routinely be a
/// release ahead of a robot it is talking to precisely because it is the machine that builds
/// releases. Nothing on the far side of this link agrees with the refusal either: this tool never
/// sends `Hello`, `configd` checks no version on `net.*` or `system.*`, and `updaterd` requires no
/// handshake before `update.status`. So every call the refusal blocked would have been answered.
///
/// And it was wrong about when to be strict. BLE is the transport for a robot that has no network,
/// and `wifi connect` is how that robot gets one — so refusing on version skew took away the
/// command that fixes the skew, at the one moment it was needed. A robot with a stale release and
/// no wifi could not be given wifi by the tool whose reason for existing is that case.
///
/// What a genuine mismatch costs without the gate is a method whose params changed shape, which
/// comes back as a JSON-RPC error naming the method — printed, and reported through the exit
/// status. That is a worse message than this one and a much better outcome than a locked door.
fn warn_about_skew(theirs: u8) {
    eprintln!(
        "warning: the robot speaks API v{theirs} and this client speaks v{}, so they were not \
         built together. Carrying on: most calls do not care, and a call that does will say so. \
         Install matching versions before believing anything surprising.",
        duck_ipc_proto::API_VERSION
    );
}

/// `vx,vy,vyaw` as three numbers.
///
/// Parsed here rather than sent as a string, so a typo is a message from this tool naming the
/// shape rather than a `PARSE_ERROR` from the robot with nothing in it to act on.
fn triple(spec: &str) -> Result<[f64; 3], String> {
    let parts: Vec<&str> = spec.split(',').collect();
    if parts.len() != 3 {
        return Err(format!(
            "expected three numbers as vx,vy,vyaw — got {spec:?}"
        ));
    }
    let mut out = [0.0; 3];
    for (slot, part) in out.iter_mut().zip(parts) {
        *slot = part
            .trim()
            .parse()
            .map_err(|_| format!("{part:?} is not a number, in {spec:?}"))?;
    }
    Ok(out)
}

fn request_line(command: &Command) -> Result<(String, Duration), Box<dyn std::error::Error>> {
    use duck_ipc_proto as proto;

    let (method, params, timeout) = match command {
        // `scan` returns from `run` as soon as the discovery loop ends, so it never reaches a
        // request: there is no method to send, and connecting is the thing it exists not to do.
        Command::Scan => unreachable!("scan returns before anything connects"),
        Command::Status => ("update.status", serde_json::json!({}), REPLY_TIMEOUT),
        // The fallback, reached only when no advertisement carried an address. `net.status` is what
        // the advertisement is made of — `btd` re-reads it every five seconds — so this asks the
        // same question over a connection that costs a bond and a PIN.
        Command::Ip | Command::Open { .. } => ("net.status", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Version => (
            "hello",
            serde_json::json!({ "api_version": duck_ipc_proto::API_VERSION }),
            REPLY_TIMEOUT,
        ),
        Command::Update(update) => return update_request_line(update),
        Command::Info => ("system.info", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Health => ("robot.health", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Name { name } => (
            "system.setName",
            serde_json::json!({ "name": name }),
            REPLY_TIMEOUT,
        ),
        Command::Reboot => ("system.reboot", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Do { skill } => (
            proto::method::ROBOT_DO,
            serde_json::json!({ "skill": skill }),
            REPLY_TIMEOUT,
        ),
        // The robot asks Hugging Face for a device code before it answers, so this is one
        // outbound HTTP round trip rather than a local read — the same budget `policy load` gets
        // for the same reason. What it does *not* wait for is the approval: that is the whole
        // shape of the flow, and it is why this tool can disconnect immediately afterwards.
        Command::Account(Account::Login { force, .. }) => (
            proto::method::ACCOUNT_LOGIN,
            serde_json::json!({ "force": force }),
            SLOW_REPLY_TIMEOUT,
        ),
        Command::Account(Account::Status) => (
            proto::method::ACCOUNT_STATUS,
            serde_json::json!({}),
            REPLY_TIMEOUT,
        ),
        Command::Account(Account::Logout) => (
            proto::method::ACCOUNT_LOGOUT,
            serde_json::json!({}),
            REPLY_TIMEOUT,
        ),
        Command::Policy(Policy::List) => (
            proto::method::ROBOT_POLICIES,
            serde_json::json!({}),
            REPLY_TIMEOUT,
        ),
        // Both of these home the robot and rebuild the controller before answering, which is
        // seconds rather than milliseconds — the same budget `wifi scan` needed for the same
        // reason, that the robot is genuinely working rather than ignoring us.
        Command::Policy(Policy::Load { slot, path }) => (
            proto::method::ROBOT_LOAD_POLICY,
            serde_json::json!({ "slot": slot, "path": path }),
            SLOW_REPLY_TIMEOUT,
        ),
        // No `path` is the reset — the same call, saying "run whatever this robot ships with".
        Command::Policy(Policy::Reset { slot }) => (
            proto::method::ROBOT_LOAD_POLICY,
            serde_json::json!({ "slot": slot }),
            SLOW_REPLY_TIMEOUT,
        ),
        Command::Pad(Pad::Bindings) => (
            proto::method::PAD_BINDINGS,
            serde_json::json!({}),
            REPLY_TIMEOUT,
        ),
        Command::Pad(Pad::Bind { button, skill }) => (
            proto::method::PAD_BIND,
            serde_json::json!({ "button": button, "skill": skill }),
            REPLY_TIMEOUT,
        ),
        // No `skill` at all is the reset, the same shape `policy reset` uses against
        // `robot.loadPolicy`: an empty string would mean "switched off", which is a different
        // wish and has to stay spellable.
        Command::Pad(Pad::Reset { button }) => (
            proto::method::PAD_BIND,
            serde_json::json!({ "button": button }),
            REPLY_TIMEOUT,
        ),
        // All four reach the Hub, so they get the budget a slow call gets: a mirror having a
        // bad day is not a robot that has stopped talking.
        Command::Policy(Policy::Search { query }) => (
            proto::method::POLICY_SEARCH,
            serde_json::json!({ "query": query }),
            SLOW_REPLY_TIMEOUT,
        ),
        Command::Policy(Policy::Check) => (
            proto::method::POLICY_CHECK,
            serde_json::json!({}),
            SLOW_REPLY_TIMEOUT,
        ),
        // Downloads about 7 MB and swaps the set, then homes the robot and reloads. The update
        // budget rather than the slow-call one, for the same reason `update apply` takes it.
        Command::Policy(Policy::Update { version }) => {
            let mut params = serde_json::json!({});
            if let Some(version) = version {
                params["version"] = serde_json::Value::String(version.clone());
            }
            (proto::method::POLICY_INSTALL, params, UPDATE_IDLE_TIMEOUT)
        }
        Command::Policy(Policy::Fetch {
            repo,
            revision,
            file,
        }) => {
            let mut params = serde_json::json!({ "repo": repo });
            if let Some(revision) = revision {
                params["revision"] = serde_json::Value::String(revision.clone());
            }
            if let Some(file) = file {
                params["file"] = serde_json::Value::String(file.clone());
            }
            (proto::method::POLICY_FETCH, params, UPDATE_IDLE_TIMEOUT)
        }
        Command::Policy(Policy::Skills) => (
            proto::method::ROBOT_SKILLS,
            serde_json::json!({}),
            REPLY_TIMEOUT,
        ),
        // Writes config and reloads seven networks before answering, like `policy load`.
        Command::Policy(Policy::Skill {
            name,
            duration,
            path,
            command,
            unwind,
            unwind_s,
        }) => {
            let mut params = serde_json::json!({ "name": name });
            if let Some(duration) = duration {
                params["duration"] = serde_json::json!(duration);
            }
            if let Some(path) = path {
                params["path"] = serde_json::Value::String(path.clone());
            }
            if let Some(command) = command {
                params["command"] = serde_json::json!(triple(command)?);
            }
            if let Some(unwind) = unwind {
                params["unwind"] = serde_json::json!(triple(unwind)?);
            }
            if let Some(unwind_s) = unwind_s {
                params["unwind_s"] = serde_json::json!(unwind_s);
            }
            (proto::method::ROBOT_SET_SKILL, params, SLOW_REPLY_TIMEOUT)
        }
        Command::Policy(Policy::Unskill { name }) => (
            proto::method::ROBOT_REMOVE_SKILL,
            serde_json::json!({ "name": name }),
            SLOW_REPLY_TIMEOUT,
        ),
        Command::Policy(Policy::Reload) => (
            proto::method::ROBOT_RELOAD_POLICIES,
            serde_json::json!({}),
            SLOW_REPLY_TIMEOUT,
        ),
        Command::Wifi(Wifi::Status) => ("net.status", serde_json::json!({}), REPLY_TIMEOUT),
        // A scan asks NetworkManager to re-scan, which takes seconds on a quiet radio.
        Command::Wifi(Wifi::Scan) => ("net.scan", serde_json::json!({}), SLOW_REPLY_TIMEOUT),
        Command::Wifi(Wifi::Connect { ssid, psk }) => {
            let mut params = serde_json::json!({ "ssid": ssid });
            if let Some(psk) = psk {
                params["psk"] = serde_json::Value::String(psk.clone());
            }
            // configd polls NM for up to 45s before calling a join timed out, so this must wait
            // longer than that or the tool gives up before the robot has decided.
            ("net.connect", params, SLOW_REPLY_TIMEOUT)
        }
        Command::Wifi(Wifi::Forget { ssid }) => (
            "net.forget",
            serde_json::json!({ "ssid": ssid }),
            REPLY_TIMEOUT,
        ),
        Command::Call { method, params } => {
            let params = match params {
                Some(text) => {
                    serde_json::from_str(text).map_err(|e| format!("params must be JSON: {e}"))?
                }
                None => serde_json::json!({}),
            };
            (method.as_str(), params, SLOW_REPLY_TIMEOUT)
        }
    };

    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": method,
        "params": params,
    });
    Ok((serde_json::to_string(&request)?, timeout))
}

/// The update commands, built from `duck_ipc_proto`'s own types rather than as hand-written JSON.
///
/// `update.apply`'s target is an externally tagged enum — `"latest"`, `{"exact":"0.5.1"}`,
/// `{"ref":"my-branch"}` — and getting that shape wrong by hand is a `PARSE_ERROR` from the robot
/// with no clue in it. Serialising the type the daemon deserialises cannot be wrong.
fn update_request_line(update: &Update) -> Result<(String, Duration), Box<dyn std::error::Error>> {
    use duck_ipc_proto as proto;

    let component = |name: &str| proto::ComponentId::new(name.to_owned());
    let (method, params, timeout) = match update {
        Update::Check { component: c } => (
            proto::method::CHECK,
            serde_json::to_value(proto::ComponentParams {
                component: component(c),
            })?,
            // Reaches the network. It answers BUSY at once during an update rather than waiting,
            // so the budget is for a slow mirror, not for a busy robot.
            SLOW_REPLY_TIMEOUT,
        ),
        Update::Apply {
            component: c,
            version,
            git_ref,
            staging,
            dry_run,
        } => {
            let target = match (version.clone(), git_ref, staging) {
                (Some(version), _, true) => proto::Target::StagingExact(version),
                (Some(version), _, false) => proto::Target::Exact(version),
                (None, Some(git_ref), _) => proto::Target::Ref(git_ref.clone()),
                (None, None, true) => proto::Target::Staging,
                (None, None, false) => proto::Target::Latest,
            };
            (
                proto::method::APPLY,
                serde_json::to_value(proto::ApplyParams {
                    component: component(c),
                    target,
                    options: proto::ApplyOptions {
                        dry_run: *dry_run,
                        ..Default::default()
                    },
                })?,
                UPDATE_IDLE_TIMEOUT,
            )
        }
        Update::Status => (proto::method::STATUS, serde_json::json!({}), REPLY_TIMEOUT),
        Update::Versions { component: c } => (
            proto::method::LIST_INSTALLED,
            serde_json::to_value(proto::ComponentParams {
                component: component(c),
            })?,
            REPLY_TIMEOUT,
        ),
        Update::Log { limit } => (
            proto::method::LOG,
            serde_json::to_value(proto::LogParams {
                limit: *limit as usize,
            })?,
            REPLY_TIMEOUT,
        ),
        Update::Rollback { component: c } => (
            proto::method::ROLLBACK,
            serde_json::to_value(proto::ComponentParams {
                component: component(c),
            })?,
            UPDATE_IDLE_TIMEOUT,
        ),
        Update::Select {
            version,
            component: c,
        } => (
            proto::method::SELECT,
            serde_json::to_value(proto::SelectParams {
                component: component(c),
                version: version.clone(),
            })?,
            UPDATE_IDLE_TIMEOUT,
        ),
        Update::Watch => (
            proto::method::SUBSCRIBE,
            serde_json::json!({}),
            FOLLOW_TIMEOUT,
        ),
    };

    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": method,
        "params": params,
    });
    Ok((serde_json::to_string(&request)?, timeout))
}

/// One progress notification, as a line for a person.
///
/// Progress goes to stderr like everything that is not an answer, so `duckctl … > reply.json`
/// keeps the two apart — and printing it as pretty JSON, which is what this used to do, put a
/// dozen lines of punctuation on stdout for every percent of a download.
fn progress_line(params: &serde_json::Value) -> String {
    let phase = params["phase"].as_str().unwrap_or("?");
    let component = params["component"].as_str().unwrap_or("?");
    let percent = params["percent"]
        .as_u64()
        .map(|p| format!(" {p}%"))
        .unwrap_or_default();
    let detail = params["detail"]
        .as_str()
        .map(|d| format!(" — {d}"))
        .unwrap_or_default();
    format!("{component}: {phase}{percent}{detail}")
}

/// What to say after an update replies, because the connection is about to drop.
///
/// Not a failure, and it should not read as one: `updaterd` and `btd` are the two units an update
/// never restarts mid-flight — `btd` may be the transport it arrived over — so both are restarted
/// about five seconds *after* the reply goes out (`docs/design/restart-order.md` §1). Every client
/// has to expect that, and a phone app should show it as a step rather than an error.
/// Show the code `account.login` just answered with, and open the page to approve it on.
///
/// The reply is JSON like every other, and this is the one call where the *human* next step is
/// not obvious from it: a code and a URL mean nothing until somebody is told to go and type them.
/// Printed as a note rather than instead of the JSON, so `--verbose` and piping still behave.
///
/// **The code is printed before the browser opens, and that order is the whole of the caution
/// here.** `remote-access-design.md` §2.1 inherits an invariant from the mini's phone app — never
/// open a browser by yourself — and it is about a phone: the browser replaces the only screen and
/// backgrounds the app, so a code shown a moment earlier is gone before it is read. A terminal
/// does not have that problem, the code stays in the scrollback, and this tool already launches a
/// browser for `duckctl open`. So it opens — after printing, so a failure to open leaves the code
/// on screen rather than an error where the instructions should have been.
///
/// It opens `verification_uri_complete`, which for Hugging Face is the plain device page: they
/// send no complete URI and their page ignores a `?user_code=` query, so what opening buys is the
/// navigation and not the typing. The code above it is still the part that matters.
fn account_note(command: &Command, reply: &serde_json::Value) -> Option<String> {
    use std::io::IsTerminal;

    let no_open = match command {
        Command::Account(Account::Login { no_open, .. }) => *no_open,
        _ => return None,
    };
    let result = reply.get("result")?;
    let code = result["user_code"].as_str()?;
    let uri = result["verification_uri"].as_str()?;
    let minutes = result["expires_in"].as_u64().unwrap_or(0) / 60;

    let mut note = format!(
        "\nOpen {uri} and enter this code:\n\n    {code}\n\n\
         You have about {minutes} minutes. The robot is doing the waiting, so this tool can \
         disconnect now — `duckctl account status` says whether it worked."
    );

    // Not a terminal means a script, and a script that opens a browser window on whoever runs it
    // is a surprise rather than a convenience.
    if no_open || !std::io::stderr().is_terminal() {
        return Some(note);
    }

    let complete = result["verification_uri_complete"]
        .as_str()
        .unwrap_or(uri)
        .to_string();
    match webbrowser::open(&complete) {
        Ok(()) => note.push_str(&format!("\n\nOpened {complete}")),
        // A warning, never a failure: the login has started, the code is valid for minutes, and
        // the terminal above says everything needed to finish it by hand. This is the opposite of
        // `duckctl open`, where opening the browser *is* the command and failing it is the result.
        Err(e) => note.push_str(&format!(
            "\n\nCould not open a browser ({e}) — the URL and code above still work. \
             `--no-open` skips this."
        )),
    }
    Some(note)
}

fn restart_note(command: &Command, reply: &serde_json::Value) -> Option<&'static str> {
    let component = match command {
        Command::Update(
            Update::Apply { component, .. }
            | Update::Rollback { component }
            | Update::Select { component, .. },
        ) => component,
        _ => return None,
    };
    // `btd` ships in the daemon release and in nothing else, so only that component's transition
    // takes the connection down. A model bundle restarts `robotd` and leaves this link alone —
    // there are no model components configured today, and a note that is wrong the first time one
    // appears is worse than no note.
    if component != "daemon" {
        return None;
    }
    // Only when something actually moved. `already_current` and `dry_run_passed` restart nothing.
    match reply["result"]["outcome"].as_str()? {
        "applied" | "rolled_back" => Some(
            "note: the robot restarts its daemons now, and `btd` about five seconds after this \
             reply — so this connection drops. That is the update working. Reconnect and run \
             `duckctl update status`: `last_attempt` carries the outcome of what just ran.",
        ),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ordinary case, and the only one on Linux: one name, reported as it was advertised.
    #[test]
    fn a_single_name_answers_to_itself() {
        assert!(answers_to("duck-c51b", "duck-c51b"));
        assert!(!answers_to("duck-c51b", "duck-ffff"));
    }

    /// The case that made `--name` unusable: the exact string a person types is *neither* of the
    /// names macOS reported, so `wifi status` failed against a robot the same message listed.
    #[test]
    fn either_half_of_a_macos_composite_answers() {
        let reported = "radxa-zero3 [duck-c51b]";
        assert!(answers_to(reported, "duck-c51b"), "the advertised name");
        assert!(answers_to(reported, "radxa-zero3"), "the cached GAP name");
        assert!(answers_to(reported, reported), "copied from the failure");
        assert!(!answers_to(reported, "duck-ffff"));
    }

    /// `scan` is read to learn which robots are reachable, and a dozen lines of earbuds above the
    /// answer bury it. The clause worth pinning is the second one: with nothing advertising the
    /// service, the other devices *are* the answer — the robot is plausibly among them — so they are
    /// listed whether or not `--verbose` was given, and gating them purely on the flag would leave
    /// that failure with nothing to act on.
    #[test]
    fn other_devices_are_listed_when_they_are_the_diagnosis() {
        assert!(!lists_others(false, 1), "the robot is the answer");
        assert!(lists_others(true, 1), "--verbose asks what the radio saw");
        assert!(lists_others(false, 0), "no robot: the list is all there is");
        assert!(lists_others(true, 0));
    }

    /// Named candidates, as `choose` takes them.
    fn candidates(names: &[&str]) -> Vec<(usize, String)> {
        names
            .iter()
            .enumerate()
            .map(|(i, name)| (i, (*name).to_owned()))
            .collect()
    }

    /// A name typed on the command line, which is the ordinary way to reach `choose`.
    fn asked_for(name: &str) -> Target {
        Target::new(Some(name.to_owned()), None)
    }

    /// The ordinary case: one name, one robot, and the composite spelling still resolves.
    #[test]
    fn a_name_selects_the_one_robot_that_answers_to_it() {
        let (which, _) = choose(
            candidates(&["duck-aaaa", "duck-c51b"]),
            &asked_for("duck-c51b"),
        )
        .expect("the named robot");
        assert_eq!(which, 1);

        let (which, _) = choose(
            candidates(&["duck-aaaa", "radxa-zero3 [duck-c51b]"]),
            &asked_for("duck-c51b"),
        )
        .expect("either half of a composite");
        assert_eq!(which, 1);
    }

    /// **The safety rule.** Two robots answering to one name is not rare or hypothetical: a board
    /// whose bootloader leaves `serial-number` empty is named from its hostname, so a bench flashed
    /// from one image is full of `radxa-zero3`. Whichever the scan reported first is not a choice,
    /// and the command that lands on it may be `net.connect` with someone's wifi password.
    #[test]
    fn a_name_matching_two_robots_is_refused_rather_than_guessed() {
        let error = choose(
            candidates(&["radxa-zero3", "radxa-zero3", "duck-c51b"]),
            &asked_for("radxa-zero3"),
        )
        .expect_err("a collision is an error");

        assert!(error.contains("2 robots"), "{error}");
        // Both are named, so the reader can tell which two collided.
        assert!(error.contains("radxa-zero3, radxa-zero3"), "{error}");
        assert!(error.contains("set-name"), "the way out: {error}");
    }

    /// A collision on a name from the environment says so. This is the failure where provenance
    /// matters most: nothing on the command line named the robot, and the message is about which of
    /// two the command would otherwise have written to.
    #[test]
    fn a_collision_on_a_default_says_where_the_name_came_from() {
        let from_env = Target::new(None, Some("radxa-zero3".to_owned()));
        let error = choose(candidates(&["radxa-zero3", "radxa-zero3"]), &from_env)
            .expect_err("a collision is an error whoever named it");

        assert!(error.contains("2 robots"), "{error}");
        assert!(error.contains("DUCK_ROBOT"), "{error}");
    }

    /// Omitting `--name` is a request to pick one, and it stays one. Making the ambiguity an error
    /// on this path would break the shorthand on every bench with two boards on it.
    #[test]
    fn without_a_name_the_first_candidate_still_wins() {
        let (which, _) = choose(
            candidates(&["duck-aaaa", "duck-c51b"]),
            &Target::new(None, None),
        )
        .expect("the first one");
        assert_eq!(which, 0);
    }

    /// A name nobody answers to lists what was there, because the usual cause is a robot that has
    /// been renamed since whoever is typing last looked.
    #[test]
    fn a_name_nobody_answers_to_lists_the_robots_that_were_there() {
        let error = choose(
            candidates(&["duck-aaaa", "duck-bbbb"]),
            &asked_for("duck-c51b"),
        )
        .expect_err("not in range");

        assert!(error.contains("no robot named"), "{error}");
        assert!(error.contains("duck-aaaa, duck-bbbb"), "{error}");
    }

    /// **The bug this replaced.** Two robots on the bench, `--name` picking one of them: the scan
    /// used to stop at the first non-empty tier, which is whichever robot the radio reported first.
    /// With the other one in `advertised`, discovery ended before the named robot had said anything,
    /// and the failure listed the robot that won the race as proof the named one was absent — while
    /// `scan`, which runs the deadline out, listed both.
    #[test]
    fn a_named_robot_is_waited_for_rather_than_the_first_one_reported() {
        let none = &candidates(&[]);
        let other = candidates(&["graphite"]);
        let both = candidates(&["graphite", "olducky"]);
        let target = asked_for("olducky");

        assert!(
            !worth_connecting(&other, none, none, &target),
            "another robot is not this one: keep listening"
        );
        assert!(
            worth_connecting(&both, none, none, &target),
            "the named robot answered: stop"
        );
        // And through the tier a bonded robot actually lands in, which is the case `--name` exists
        // for: no service UUID, so the name is the only evidence there is.
        assert!(worth_connecting(
            none,
            &candidates(&["olducky"]),
            none,
            &target
        ));
        // Either half of a macOS composite, on the same rule the search itself uses.
        assert!(worth_connecting(
            &candidates(&["radxa-zero3 [olducky]"]),
            none,
            none,
            &target,
        ));
    }

    /// The two ways a named search ends with no robot, told apart by the rule the search itself
    /// uses. Nothing answered: the failure is a claim about eight seconds, and everything the radio
    /// saw belongs under it as evidence. Two answered: the two names *are* the evidence, and fifty
    /// lines of earbuds beneath them would bury the only thing worth reading.
    #[test]
    fn a_miss_and_a_collision_are_not_the_same_failure() {
        assert!(
            !any_answers(&candidates(&["graphite"]), "olducky"),
            "a miss"
        );
        assert!(
            any_answers(&candidates(&["radxa-zero3", "radxa-zero3"]), "radxa-zero3"),
            "a collision"
        );
    }

    /// Without a name the fast path is unchanged, and that matters as much as the fix: a bonded
    /// robot may never re-advertise the service, so a shorthand that waited for a better candidate
    /// would wait out all eight seconds on every single command.
    #[test]
    fn without_a_name_the_first_candidate_still_stops_the_scan() {
        let none = &candidates(&[]);
        let one = candidates(&["duck-c51b"]);
        let anybody = Target::new(None, None);

        assert!(!worth_connecting(none, none, none, &anybody), "nothing yet");
        assert!(worth_connecting(&one, none, none, &anybody));
        assert!(worth_connecting(none, &one, none, &anybody));
        assert!(
            worth_connecting(none, none, &one, &anybody),
            "an already-connected peripheral is a candidate when nobody named one"
        );
    }

    /// `--name` says which robot to talk to and the `name` subcommand's positional says what to
    /// call it, and only an explicit id keeps the two apart. Parsing is pinned rather than left to
    /// review because the failure did not look like a CLI bug: the tool scanned for the new name,
    /// found nothing, and listed the robot it was talking to seconds earlier as merely in range.
    #[test]
    fn a_rename_still_selects_the_robot_by_the_name_it_has_now() {
        let cli = Cli::try_parse_from(["duckctl", "--name", "duck-c51b", "name", "leduckpierre"])
            .expect("the rename form parses");

        assert_eq!(cli.name.as_deref(), Some("duck-c51b"), "which robot");
        let Command::Name { name } = &cli.command else {
            panic!("the name subcommand");
        };
        assert_eq!(name, "leduckpierre", "what to call it");

        // And the new name is what reaches the robot, not the one it was found by.
        let (line, _) = request_line(&cli.command).expect("a request");
        assert!(line.contains(r#""method":"system.setName""#), "{line}");
        assert!(line.contains(r#""name":"leduckpierre""#), "{line}");
    }

    /// `account login` sends `force` as the robot's route table expects it.
    #[test]
    fn a_forced_login_says_so_on_the_wire() {
        let cli = Cli::try_parse_from(["duckctl", "account", "login", "--force"])
            .expect("the login form parses");
        let (line, _) = request_line(&cli.command).expect("a request");
        assert!(line.contains(r#""method":"account.login""#), "{line}");
        assert!(line.contains(r#""force":true"#), "{line}");

        let cli = Cli::try_parse_from(["duckctl", "account", "login"]).expect("parses");
        let (line, _) = request_line(&cli.command).expect("a request");
        assert!(line.contains(r#""force":false"#), "{line}");
    }

    /// The code and the URL reach the terminal, and `--no-open` opens nothing.
    ///
    /// The assertion that matters is the *order*: the code is in the note whether or not a
    /// browser could be launched, because a login whose instructions were replaced by an error
    /// message is a login nobody can finish by hand. `remote-access-design.md` §2.1.
    #[test]
    fn a_login_note_carries_the_code_without_opening_anything() {
        let cli =
            Cli::try_parse_from(["duckctl", "account", "login", "--no-open"]).expect("parses");
        let reply = serde_json::json!({
            "jsonrpc": "2.0",
            "id": 1,
            "result": {
                "user_code": "A6MY-0314",
                "verification_uri": "https://hf.co/oauth/device",
                // The plain URI, which is what the robot sends for Hugging Face: no complete
                // one comes back and their page prefills nothing from a query.
                "verification_uri_complete": "https://hf.co/oauth/device",
                "expires_in": 300,
                "interval": 5
            }
        });

        let note = account_note(&cli.command, &reply).expect("a note for a login");
        assert!(note.contains("A6MY-0314"), "the code: {note}");
        assert!(
            note.contains("https://hf.co/oauth/device"),
            "where to type it: {note}"
        );
        assert!(
            note.contains("about 5 minutes"),
            "how long it is good for: {note}"
        );
        assert!(
            !note.contains("Opened"),
            "--no-open must not launch a browser: {note}"
        );
        assert!(
            note.contains("duckctl account status"),
            "the note has to say how to find out whether it worked: {note}"
        );
    }

    /// Every other command gets no login note, including the two next to it in the namespace.
    #[test]
    fn only_a_login_gets_the_code_note() {
        let reply = serde_json::json!({ "result": { "account": null } });
        for args in [
            vec!["duckctl", "account", "status"],
            vec!["duckctl", "account", "logout"],
            vec!["duckctl", "info"],
        ] {
            let cli = Cli::try_parse_from(&args).expect("parses");
            assert!(
                account_note(&cli.command, &reply).is_none(),
                "{args:?} must get no login note"
            );
        }
    }

    /// The point of the whole thing: a robot named by nobody who is typing. The environment has to
    /// reach the same search `--name` does, and say so, since a default that silently redirects
    /// every command is worse than no default.
    #[test]
    fn the_environment_names_the_robot_when_the_flag_does_not() {
        let target = Target::new(None, Some("duck-c51b".to_owned()));

        assert_eq!(target.wanted(), Some("duck-c51b"));
        assert!(target.from_env);
        assert_eq!(target.source(), "DUCK_ROBOT");
        let provenance = target.provenance();
        assert!(
            provenance.contains("DUCK_ROBOT"),
            "names the variable: {provenance}"
        );
        assert!(
            provenance.contains("DUCK_ROBOT= "),
            "and the way out: {provenance}"
        );
    }

    /// A command line beats a shell profile, and whoever typed `--name` does not need telling where
    /// the name came from.
    #[test]
    fn the_flag_beats_the_environment() {
        let target = Target::new(Some("duck-ffff".to_owned()), Some("duck-c51b".to_owned()));

        assert_eq!(target.wanted(), Some("duck-ffff"));
        assert!(!target.from_env);
        assert_eq!(target.source(), "--name");
        assert!(target.provenance().is_empty(), "nothing to disclose");
    }

    /// Empty is unset, which is why clap's own `env` support is not used: it treats `DUCK_ROBOT=` as
    /// a value, and a variable exported in a shell profile could then only be escaped by unsetting
    /// it — for a command being typed on a bench that has somebody else's robot on it.
    #[test]
    fn an_empty_value_is_no_default_at_all() {
        let escaped = Target::new(None, Some(String::new()));
        assert_eq!(escaped.wanted(), None, "`DUCK_ROBOT= duckctl …`");
        assert!(
            escaped.provenance().is_empty(),
            "no name, nothing to explain"
        );

        let overridden = Target::new(Some(String::new()), Some("duck-c51b".to_owned()));
        assert_eq!(
            overridden.wanted(),
            None,
            "`--name ''` drops the default too"
        );
    }

    /// The rename works, and then every later command searches for a name nothing answers to — which
    /// reads as a robot that went away rather than a variable that went stale.
    #[test]
    fn a_rename_says_when_it_leaves_the_default_stale() {
        let rename = Command::Name {
            name: "leduckpierre".to_owned(),
        };
        let from_env = Target::new(None, Some("duck-c51b".to_owned()));

        let note = from_env
            .stale_after_rename(&rename)
            .expect("the environment still says the old name");
        assert!(note.contains("duck-c51b"), "what to change: {note}");
        assert!(note.contains("leduckpierre"), "what it is now: {note}");

        let typed = Target::new(Some("duck-c51b".to_owned()), None);
        assert!(
            typed.stale_after_rename(&rename).is_none(),
            "a `--name` typed once is not still in force"
        );
        assert!(
            from_env
                .stale_after_rename(&Command::Name {
                    name: "duck-c51b".to_owned()
                })
                .is_none(),
            "renamed to the name it already answers to"
        );
        assert!(
            from_env.stale_after_rename(&Command::Info).is_none(),
            "nothing was renamed"
        );
    }

    /// `scan` with a default set is read to answer one question — is my robot here — and otherwise
    /// leaves it as a string comparison done by eye against a column of hex.
    #[test]
    fn a_listing_marks_the_robot_the_default_names() {
        let target = Target::new(None, Some("duck-c51b".to_owned()));

        assert!(target.marks(Some("duck-c51b")));
        assert!(
            target.marks(Some("radxa-zero3 [duck-c51b]")),
            "the macOS pair"
        );
        assert!(!target.marks(Some("duck-ffff")));
        assert!(
            !target.marks(None),
            "an unnamed device is not the named one"
        );
        assert!(
            !Target::new(None, None).marks(Some("duck-c51b")),
            "with no default there is nothing to mark"
        );
    }

    /// One advertisement, as `btleplug` would report it.
    fn advertised(
        duck: bool,
        manufacturer_data: &[(u16, Vec<u8>)],
    ) -> (PeripheralProperties, bool) {
        (
            PeripheralProperties {
                manufacturer_data: manufacturer_data.iter().cloned().collect(),
                ..Default::default()
            },
            duck,
        )
    }

    /// The whole point of the change: a listing says where to reach the robot, with no connection.
    /// The one thing `open` adds over `ip`, and the reason it is a command rather than a documented
    /// `open "http://$(duckctl ip):8080"`: the port has one home.
    #[test]
    fn the_console_url_is_the_address_and_the_port() {
        assert_eq!(
            console_url("192.168.1.42", 8080),
            "http://192.168.1.42:8080/"
        );
        assert_eq!(
            console_url("192.168.1.42", 9000),
            "http://192.168.1.42:9000/"
        );
    }

    /// `ip` picks a robot the same way every other command does. Two robots on one bench, one of
    /// them named — the address that comes back has to be that robot's, not whichever advertisement
    /// arrived first.
    #[test]
    fn an_advertised_address_is_chosen_by_name() {
        let found = vec![
            (
                Address::At("192.168.1.7".parse().unwrap()),
                "duck-aaaa".to_owned(),
            ),
            (
                Address::At("192.168.1.42".parse().unwrap()),
                "duck-c51b".to_owned(),
            ),
        ];
        let (address, name) = choose(found, &asked_for("duck-c51b")).expect("one robot answers");
        assert_eq!(name, "duck-c51b");
        assert_eq!(address, Address::At("192.168.1.42".parse().unwrap()));
    }

    /// A robot that broadcast `0.0.0.0` is a robot with no network, and the fix is over the radio —
    /// so the message names the command that does it rather than the field that was empty.
    #[test]
    fn a_robot_with_no_network_is_told_how_to_get_one() {
        let message = no_address("duck-c51b");
        assert!(message.contains("duck-c51b"));
        assert!(message.contains("wifi connect"));
    }

    /// `--print` and `--port` are the two things `open` takes, and neither is positional.
    #[test]
    fn open_takes_a_port_and_can_print_instead() {
        let cli = Cli::try_parse_from(["duckctl", "open", "--print", "--port", "9000"])
            .expect("open --print --port parses");
        assert!(matches!(
            cli.command,
            Command::Open {
                print: true,
                port: 9000
            }
        ));

        let cli = Cli::try_parse_from(["duckctl", "open"]).expect("open parses on its own");
        assert!(matches!(
            cli.command,
            Command::Open {
                print: false,
                port: 8080
            }
        ));
    }

    #[test]
    fn a_robot_broadcasts_where_it_is() {
        let (properties, duck) = advertised(
            true,
            &[(
                adv::COMPANY_ID,
                adv::address_data(Some(Ipv4Addr::new(192, 168, 1, 42))),
            )],
        );
        let address = Address::read(&properties, duck);
        assert_eq!(address, Address::At(Ipv4Addr::new(192, 168, 1, 42)));
        assert_eq!(address.note().as_deref(), Some("192.168.1.42"));
    }

    /// The two blanks are not one blank. A robot with no wifi is a wifi problem; a robot that said
    /// nothing is an update — and the listing sends the reader somewhere different for each.
    #[test]
    fn no_wifi_and_no_field_read_differently() {
        let (properties, duck) = advertised(true, &[(adv::COMPANY_ID, adv::address_data(None))]);
        assert_eq!(Address::read(&properties, duck), Address::Unassigned);
        assert_eq!(
            Address::read(&properties, duck).note().as_deref(),
            Some("no address")
        );

        let (properties, duck) = advertised(true, &[]);
        assert_eq!(Address::read(&properties, duck), Address::Unsaid);
        assert_eq!(
            Address::read(&properties, duck).note(),
            None,
            "nothing on the line; the note under the list covers it"
        );
    }

    /// `0xFFFF` is the company id the SIG leaves open to anyone, so four bytes of it on a device that
    /// never advertised the duck service are somebody else's four bytes. Listing an earbud with an
    /// invented address would be worse than listing it with none.
    #[test]
    fn only_a_robot_is_read_for_an_address() {
        let (properties, duck) = advertised(
            false,
            &[(
                adv::COMPANY_ID,
                adv::address_data(Some(Ipv4Addr::new(10, 0, 0, 1))),
            )],
        );
        assert_eq!(Address::read(&properties, duck), Address::Unsaid);
    }

    /// The PIN matters more than the name does — a robot with a real one needs it on every
    /// command — and an empty `DUCK_PIN` left over from a script must not become the PIN, or the
    /// robot answers "wrong PIN" for a PIN nobody chose.
    #[test]
    fn the_pin_falls_back_through_the_environment_to_the_factory_default() {
        let six = |pin: &str| Some(pin.to_owned());

        assert_eq!(resolve_pin(six("111111"), six("222222")), "111111");
        assert_eq!(resolve_pin(None, six("222222")), "222222");
        assert_eq!(resolve_pin(None, None), DEFAULT_PIN);
        assert_eq!(resolve_pin(None, Some(String::new())), DEFAULT_PIN);
        assert_eq!(resolve_pin(Some(String::new()), six("222222")), "222222");
    }

    /// The split is a guess and it can be wrong: a robot whose own name ends in a bracket group is
    /// indistinguishable from the composite, so its halves are accepted as well. Tolerated rather
    /// than fixed — btleplug joins the two names before we see them, and the pair is gone — because
    /// the cost is only that an explicit `--name` matches more, on names nobody gives a robot. What
    /// has to hold is that the shape must actually be there.
    #[test]
    fn the_split_needs_the_shape_it_looks_for() {
        assert!(answers_to("duck [1]", "duck [1]"));
        assert!(!answers_to("[duck-c51b]", "duck-c51b"));
        assert!(!answers_to("duck-c51b [", "duck-c51b"));
    }

    /// **The Hub commands take the budget their work needs**, because the failure otherwise
    /// looks like a robot that stopped talking rather than a mirror having a bad day.
    #[test]
    fn the_hub_commands_wait_as_long_as_they_need() {
        let budget = |args: &[&str]| {
            let cli = Cli::try_parse_from([&["duckctl"], args].concat()).expect("parses");
            request_line(&cli.command).expect("a request").1
        };

        // Reaches the network and answers.
        assert_eq!(
            budget(&["policy", "search", "microduck"]),
            SLOW_REPLY_TIMEOUT
        );
        assert_eq!(budget(&["policy", "check"]), SLOW_REPLY_TIMEOUT);
        // Downloads megabytes, then homes the robot and reloads seven networks.
        assert_eq!(budget(&["policy", "update"]), UPDATE_IDLE_TIMEOUT);
        assert_eq!(
            budget(&["policy", "fetch", "org/repo"]),
            UPDATE_IDLE_TIMEOUT
        );
    }

    /// An omitted flag is an omitted field, not a null: `version`, `revision` and `file` all
    /// mean "the usual thing" by being absent, and a `null` would be a different request.
    #[test]
    fn the_hub_commands_omit_what_was_not_asked_for() {
        let wire = |args: &[&str]| {
            let cli = Cli::try_parse_from([&["duckctl"], args].concat()).expect("parses");
            request_line(&cli.command).expect("a request").0
        };

        let bare = wire(&["policy", "update"]);
        assert!(
            bare.contains(duck_ipc_proto::method::POLICY_INSTALL),
            "{bare}"
        );
        assert!(!bare.contains("version"), "{bare}");
        assert!(wire(&["policy", "update", "--version", "v1"]).contains(r#""version":"v1""#));

        let fetch = wire(&["policy", "fetch", "org/repo"]);
        assert!(fetch.contains(r#""repo":"org/repo""#), "{fetch}");
        assert!(
            !fetch.contains("revision") && !fetch.contains("file"),
            "{fetch}"
        );
    }

    /// **`policy reset` is `loadPolicy` with no path**, and that is the whole of the difference.
    ///
    /// Two subcommands over one method, because the wire call says "run this in that slot" and
    /// omitting the file means "run whatever this robot ships with". A `reset` that sent
    /// `"path":null` would be the same request, but the omission is what the daemon's
    /// `Option<String>` is written against, and pinning it here is cheaper than finding out on
    /// a robot.
    #[test]
    fn resetting_a_slot_is_loading_it_with_no_path() {
        let wire = |args: &[&str]| {
            let mut argv = vec!["duckctl", "policy"];
            argv.extend_from_slice(args);
            let cli = Cli::try_parse_from(argv).expect("parses");
            request_line(&cli.command).expect("a request").0
        };

        let load = wire(&[
            "load",
            "walk",
            "/opt/robot/policies/current/alpha_walking.onnx",
        ]);
        assert!(load.contains(r#""method":"robot.loadPolicy""#), "{load}");
        assert!(load.contains(r#""slot":"walk""#), "{load}");
        assert!(
            load.contains(r#""path":"/opt/robot/policies/current/alpha_walking.onnx""#),
            "{load}"
        );

        let reset = wire(&["reset", "walk"]);
        assert!(reset.contains(r#""method":"robot.loadPolicy""#), "{reset}");
        assert!(reset.contains(r#""slot":"walk""#), "{reset}");
        assert!(!reset.contains("path"), "no path at all: {reset}");
    }

    /// The method names, against the daemon's own constants rather than strings typed twice.
    #[test]
    fn the_policy_commands_name_the_methods_the_daemon_serves() {
        let wire = |args: &[&str]| {
            let cli = Cli::try_parse_from([&["duckctl"], args].concat()).expect("parses");
            request_line(&cli.command).expect("a request").0
        };

        assert!(wire(&["policy", "list"]).contains(duck_ipc_proto::method::ROBOT_POLICIES));
        assert!(
            wire(&["policy", "reload"]).contains(duck_ipc_proto::method::ROBOT_RELOAD_POLICIES)
        );

        let run = wire(&["do", "polite-bow"]);
        assert!(run.contains(duck_ipc_proto::method::ROBOT_DO), "{run}");
        assert!(run.contains(r#""skill":"polite-bow""#), "{run}");
    }

    /// **A load homes the robot before it answers**, so it gets the budget a slow call gets.
    /// On the reply timeout it would look like a robot that had stopped talking, mid-swap.
    #[test]
    fn swapping_a_policy_waits_as_long_as_a_slow_call() {
        let budget = |args: &[&str]| {
            let cli = Cli::try_parse_from([&["duckctl"], args].concat()).expect("parses");
            request_line(&cli.command).expect("a request").1
        };

        assert_eq!(
            budget(&["policy", "load", "walk", "/tmp/x.onnx"]),
            SLOW_REPLY_TIMEOUT
        );
        assert_eq!(budget(&["policy", "reset", "walk"]), SLOW_REPLY_TIMEOUT);
        assert_eq!(budget(&["policy", "reload"]), SLOW_REPLY_TIMEOUT);
        // Reading changes nothing and answers at once.
        assert_eq!(budget(&["policy", "list"]), REPLY_TIMEOUT);
    }

    /// Every shape `update apply` can ask for, on the wire.
    ///
    /// `Target` is an externally tagged enum, so each of these is a *different JSON shape* rather
    /// than a different value in one field, and a mistake is a parse error from the robot with
    /// nothing in it to act on. Built from the daemon's own type for that reason, and pinned here
    /// because the flags are what a person types.
    #[test]
    fn apply_asks_for_the_target_the_flags_named() {
        let wire = |args: &[&str]| {
            let mut argv = vec!["duckctl", "update", "apply"];
            argv.extend_from_slice(args);
            let cli = Cli::try_parse_from(argv).expect("parses");
            request_line(&cli.command).expect("a request").0
        };

        assert!(wire(&[]).contains(r#""target":"latest""#));
        assert!(wire(&["--version", "0.5.1"]).contains(r#""target":{"exact":"0.5.1"}"#));
        assert!(wire(&["--ref", "my-branch"]).contains(r#""target":{"ref":"my-branch"}"#));
        assert!(wire(&["--staging"]).contains(r#""target":"staging""#));
        assert!(
            wire(&["--staging", "--version", "0.6.0"])
                .contains(r#""target":{"staging_exact":"0.6.0"}"#),
            "a named candidate, which is neither of the two flags on its own"
        );

        let plain = wire(&[]);
        assert!(plain.contains(r#""method":"update.apply""#), "{plain}");
        assert!(
            plain.contains(r#""component":"daemon""#),
            "the default component"
        );
        assert!(
            plain.contains(r#""dry_run":false"#),
            "the options travel spelled out, which is what the daemon parses: {plain}"
        );
        assert!(wire(&["--dry-run"]).contains(r#""dry_run":true"#));
    }

    /// A ref and a version are alternatives, not a precedence to resolve by guessing — the same
    /// refusal `robotctl` makes, so a command copied between them fails the same way.
    #[test]
    fn a_ref_and_a_version_cannot_both_be_named() {
        assert!(
            Cli::try_parse_from([
                "duckctl",
                "update",
                "apply",
                "--ref",
                "my-branch",
                "--version",
                "0.5.1",
            ])
            .is_err()
        );
    }

    /// The read-only commands and going back, each reaching the method it is named after. A table,
    /// because the value of these commands over `call` is that the method is not typed by hand.
    #[test]
    fn every_update_command_asks_for_its_own_method() {
        for (args, method) in [
            (vec!["check"], "update.check"),
            (vec!["status"], "update.status"),
            (vec!["versions"], "update.listInstalled"),
            (vec!["log"], "update.log"),
            (vec!["watch"], "update.subscribe"),
            (vec!["rollback"], "update.rollback"),
            (vec!["select", "0.5.1"], "update.select"),
        ] {
            let mut argv = vec!["duckctl", "update"];
            argv.extend_from_slice(&args);
            let cli = Cli::try_parse_from(argv).expect("parses");
            let (line, _) = request_line(&cli.command).expect("a request");
            assert!(line.contains(&format!(r#""method":"{method}""#)), "{line}");
        }
    }

    /// `select` sends the version as a version, and defaults the component like the rest.
    #[test]
    fn select_names_a_version_and_defaults_the_component() {
        let cli = Cli::try_parse_from(["duckctl", "update", "select", "0.5.1"]).expect("parses");
        let (line, _) = request_line(&cli.command).expect("a request");
        assert!(line.contains(r#""version":"0.5.1""#), "{line}");
        assert!(line.contains(r#""component":"daemon""#), "{line}");
    }

    /// An update waits on silence, not on a total, and it waits longer than anything else does.
    /// A total budget would cut off a working update; this is the longest gap one can legitimately
    /// have (a post-install hook) rather than the longest one can take.
    #[test]
    fn an_update_is_given_the_longest_silence() {
        let budget = |args: &[&str]| {
            let mut argv = vec!["duckctl"];
            argv.extend_from_slice(args);
            let cli = Cli::try_parse_from(argv).expect("parses");
            request_line(&cli.command).expect("a request").1
        };

        assert_eq!(budget(&["update", "apply"]), UPDATE_IDLE_TIMEOUT);
        assert_eq!(budget(&["update", "rollback"]), UPDATE_IDLE_TIMEOUT);
        assert_eq!(budget(&["update", "select", "0.5.1"]), UPDATE_IDLE_TIMEOUT);
        assert!(budget(&["update", "apply"]) > budget(&["update", "status"]));
        assert_eq!(
            budget(&["update", "watch"]),
            FOLLOW_TIMEOUT,
            "watch ends with Ctrl-C, not with a deadline"
        );
    }

    /// Progress reads as a line rather than as JSON, and survives a field it does not have.
    ///
    /// The phase is always there; the percent only during a download, and the detail rarely. A
    /// `None` percent must not print as `null%`, which is what a naive format would do.
    #[test]
    fn progress_prints_as_a_line() {
        let full = serde_json::json!({
            "component": "daemon",
            "phase": "downloading",
            "percent": 42,
            "detail": null,
        });
        assert_eq!(progress_line(&full), "daemon: downloading 42%");

        let bare = serde_json::json!({ "component": "daemon", "phase": "health_gate" });
        assert_eq!(progress_line(&bare), "daemon: health_gate");

        let detailed = serde_json::json!({
            "component": "daemon",
            "phase": "verifying",
            "detail": "0.6.0",
        });
        assert_eq!(progress_line(&detailed), "daemon: verifying — 0.6.0");
    }

    /// The disconnect that follows an update is announced, and only when something moved.
    ///
    /// `already_current` restarts nothing, so telling someone their connection is about to drop
    /// would be a false alarm — and the note is what stops a *real* drop from reading as a failed
    /// update, so it has to be trustworthy.
    #[test]
    fn a_restart_is_announced_only_when_the_release_changed() {
        let apply = Cli::try_parse_from(["duckctl", "update", "apply"])
            .expect("parses")
            .command;

        let applied = serde_json::json!({
            "id": 1,
            "result": { "outcome": "applied", "from": "0.5.1", "to": "0.6.0" },
        });
        assert!(restart_note(&apply, &applied).is_some());

        let unchanged = serde_json::json!({
            "id": 1,
            "result": { "outcome": "already_current", "version": "0.6.0" },
        });
        assert!(restart_note(&apply, &unchanged).is_none());

        let dry_run = serde_json::json!({
            "id": 1,
            "result": { "outcome": "dry_run_passed", "candidate": "0.6.0" },
        });
        assert!(restart_note(&apply, &dry_run).is_none());

        // And nothing else announces one, however it answered.
        let status = Cli::try_parse_from(["duckctl", "update", "status"])
            .expect("parses")
            .command;
        assert!(restart_note(&status, &applied).is_none());

        // Nor a component whose release does not ship `btd`.
        let model = Cli::try_parse_from(["duckctl", "update", "apply", "--component", "model"])
            .expect("parses")
            .command;
        assert!(restart_note(&model, &applied).is_none());
    }

    /// A drop during an apply is sent to the record, because the apply may well have worked.
    ///
    /// This is the message that replaces `silence` for the case it used to describe wrongly, and
    /// it is worth the same care as the note that predicts the drop: it must send someone to
    /// `last_attempt` when the update was restarting daemons, and not invent a restart when
    /// nothing was being installed.
    #[test]
    fn a_drop_during_an_apply_points_at_the_record() {
        let note = |argv: &[&str]| {
            let mut full = vec!["duckctl"];
            full.extend_from_slice(argv);
            dropped(&Cli::try_parse_from(full).expect("parses").command)
        };

        for argv in [
            vec!["update", "apply"],
            vec!["update", "rollback"],
            vec!["update", "select", "0.5.1"],
        ] {
            let said = note(&argv);
            assert!(said.contains("update status"), "{argv:?}: {said}");
        }

        // A poll is not a transition, so nothing was restarting.
        let said = note(&["update", "status"]);
        assert!(!said.contains("restarts"), "{said}");

        // Nor is anything else, and a component whose release does not ship `btd` restarts
        // `robotd` and leaves this link alone.
        for argv in [
            vec!["wifi", "status"],
            vec!["update", "apply", "--component", "model"],
        ] {
            let said = note(&argv);
            assert!(!said.contains("restarts"), "{argv:?}: {said}");
        }
    }

    /// The link is checked long before the budget it interrupts expires.
    ///
    /// The relationship is the whole point: a poll as long as the budget notices a dropped link
    /// exactly when giving up would have, which is the three-minute silence this exists to end.
    #[test]
    fn the_link_is_checked_long_before_a_wait_gives_up() {
        assert!(LINK_POLL < REPLY_TIMEOUT);
        assert!(LINK_POLL * 10 < UPDATE_IDLE_TIMEOUT);
    }
}
