//! A radio for simulated ducks: what one advertises, the ones near it hear.
//!
//! ```text
//! duck-ether --duck duck-a=/run/duck-a/robotd.sock@7801 \
//!            --duck duck-b=/run/duck-b/robotd.sock@7802
//! ```
//!
//! **This replaces `btd`'s radio and nothing above it.** Presence is already an IPC contract on
//! `robotd`'s own socket — `chorale.subscribe` to be told what to put on the air, `chorale.beacon`
//! carrying it, `chorale.heard` carrying what came back — and `btd` is a *client* of `robotd`
//! rather than a server. So this impersonates nothing and steals no socket path: it holds one
//! connection per duck, exactly as `btd` does, and every duck's election, roster, beat and
//! conductor deference runs unmodified and cannot tell.
//!
//! ## What decides who hears whom
//!
//! Distance, because [`ChoraleHeard`] has no signal strength in it — a real scanner either sees an
//! advertisement or does not, and a beacon out of range simply never arrives. So the ether asks each
//! simulator where its duck is standing and delivers a beacon only to ducks within [`RANGE`] of it.
//! That is a cruder radio than a real one and a far more controllable one: a range that is a number
//! makes "these two can hear each other and those two cannot" a thing you can set up in a second,
//! which on real hardware means carrying robots into other rooms.
//!
//! ## Two things it gets right that are easy to get wrong
//!
//! **`age_us` is an age, not a timestamp.** The field exists because two daemons share a machine and
//! not an epoch, and `robotd` subtracts it from its own clock on arrival — so filling it with a real
//! elapsed time is what keeps the beat synchronisation being exercised rather than short-circuited.
//!
//! **The address rotates.** `from` is documented as an identity for de-duplication only, and a real
//! duck's BLE address changes underneath it — a fact that cost this project a day when something
//! keyed on it. `--rotate` makes that happen on a timer, which turns the bug into a test.
//!
//! ## Being a bad radio on purpose
//!
//! **A perfect ether hides the bugs a real one causes**, and that is not a hypothetical. Four ducks
//! in the twin converge on one piece every time, staggered starts included — because every duck is
//! visible to every other from the moment it boots, instantly and losslessly. On hardware, BLE
//! discovery is slow and lossy, so two ducks can be singing before the other two have seen them,
//! which is exactly the split-brain the chorale's election has to survive.
//!
//! So `--discovery` makes a duck take a while to be *noticed*, per pair rather than globally,
//! because it is the asymmetry that splits a flock: A and B finding each other quickly while C and D
//! are still deaf is the scenario, and one global delay cannot produce it. `--loss` drops a fraction
//! of deliveries. Both are driven by a seeded PRNG, so a split that happens once can be made to
//! happen again — a flaky radio is only useful for debugging if its flakiness repeats.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::Parser;
use duck_ipc_proto as proto;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpStream, UnixStream};
use tokio::sync::Mutex;

/// How far a beacon carries, in metres.
///
/// Generous on purpose: a real advertisement crosses a room and then some, and the interesting
/// failures are about who is *out* of range, which is what `--range` is for.
const RANGE: f64 = 8.0;

/// How often each duck's beacon is delivered to the ducks near it.
///
/// A real scanner sees an advertisement repeatedly, not once — `robotd` ages every sighting and
/// drops a peer that stops arriving, so a beacon sent once would be a duck that vanishes.
const DELIVERY: Duration = Duration::from_millis(200);

#[derive(Parser, Debug)]
#[command(about = "A radio for simulated ducks", version)]
struct Args {
    /// A duck, as `name=/path/to/robotd.sock@body-port`. Repeat for each.
    #[arg(long = "duck", value_name = "NAME=SOCKET@PORT", required = true)]
    ducks: Vec<String>,

    /// How far a beacon carries, in metres.
    #[arg(long, default_value_t = RANGE)]
    range: f64,

    /// Rotate each duck's address every N seconds, as a real one does. 0 leaves it alone.
    #[arg(long, default_value_t = 0)]
    rotate: u64,

    /// Take up to this many seconds to notice each duck, per pair. 0 is an instant, perfect radio.
    #[arg(long, default_value_t = 0)]
    discovery: u64,

    /// Drop this fraction of deliveries, 0.0 to 1.0. A real advertisement is missed often.
    #[arg(long, default_value_t = 0.0)]
    loss: f64,

    /// Seed for the discovery delays and the losses, so a split can be reproduced.
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

/// A radio's imperfections, as numbers.
#[derive(Debug, Clone, Copy)]
struct Weather {
    discovery: Duration,
    loss: f64,
    seed: u64,
}

/// Deterministic noise from a name and a counter — a splitmix step, which is short, well distributed
/// and needs no dependency. Deterministic is the point: a radio that is flaky differently every run
/// cannot be used to chase a bug.
fn noise(seed: u64, key: &str, salt: u64) -> u64 {
    let mut x = seed ^ salt;
    for byte in key.as_bytes() {
        x = x.wrapping_mul(0x100_0000_01b3) ^ u64::from(*byte);
    }
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

#[derive(Debug, Clone)]
struct Duck {
    name: String,
    socket: PathBuf,
    body: u16,
}

fn parse_duck(text: &str) -> Result<Duck, String> {
    let (name, rest) = text
        .split_once('=')
        .ok_or_else(|| format!("{text:?} is not name=socket@port"))?;
    let (socket, port) = rest
        .rsplit_once('@')
        .ok_or_else(|| format!("{text:?} has no @body-port"))?;
    Ok(Duck {
        name: name.to_owned(),
        socket: PathBuf::from(socket),
        body: port
            .parse()
            .map_err(|_| format!("{port:?} is not a port number"))?,
    })
}

/// What one duck is putting on the air, and whether it is listening.
#[derive(Default)]
struct OnAir {
    beacon: Option<proto::ChoraleBeacon>,
    listening: bool,
    /// Where this duck is standing, from its simulator.
    at: [f64; 3],
    /// When this duck first had something to advertise — discovery is timed from there, because a
    /// scanner cannot notice a duck that is not yet on the air.
    since: Option<Instant>,
    /// What its radio calls itself today.
    address: String,
}

type Air = Arc<Mutex<HashMap<String, OnAir>>>;

#[tokio::main(flavor = "current_thread")]
async fn main() -> std::process::ExitCode {
    let args = Args::parse();
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let ducks: Vec<Duck> = match args.ducks.iter().map(|d| parse_duck(d)).collect() {
        Ok(ducks) => ducks,
        Err(why) => {
            tracing::error!("{why}");
            return std::process::ExitCode::FAILURE;
        }
    };

    let air: Air = Arc::new(Mutex::new(HashMap::new()));
    {
        let mut on_air = air.lock().await;
        for (index, duck) in ducks.iter().enumerate() {
            on_air.insert(
                duck.name.clone(),
                OnAir {
                    address: address_for(index, 0),
                    ..Default::default()
                },
            );
        }
    }

    let weather = Weather {
        discovery: Duration::from_secs(args.discovery),
        loss: args.loss.clamp(0.0, 1.0),
        seed: args.seed,
    };
    tracing::info!(
        ducks = ducks.len(),
        range = args.range,
        discovery_s = args.discovery,
        loss = weather.loss,
        seed = args.seed,
        "the ether is open"
    );

    let mut tasks = Vec::new();
    for duck in &ducks {
        let (duck, air, range, weather) = (duck.clone(), air.clone(), args.range, weather);
        tasks.push(tokio::spawn(async move {
            loop {
                if let Err(e) = serve(&duck, &air, range, weather).await {
                    tracing::warn!(duck = %duck.name, error = %e, "lost the duck; retrying");
                }
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }));
    }

    tasks.push(tokio::spawn(positions(ducks.clone(), air.clone())));
    if args.rotate > 0 {
        tasks.push(tokio::spawn(rotate(
            ducks.clone(),
            air.clone(),
            args.rotate,
        )));
    }

    for task in tasks {
        let _ = task.await;
    }
    std::process::ExitCode::SUCCESS
}

/// A believable BLE address, and a different one after every rotation.
fn address_for(index: usize, generation: u64) -> String {
    let n = (index as u64 + 1) * 0x1_0000 + generation;
    format!(
        "E{:1X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        index & 0xF,
        (n >> 32) & 0xFF,
        (n >> 24) & 0xFF,
        (n >> 16) & 0xFF,
        (n >> 8) & 0xFF,
        n & 0xFF
    )
}

/// One duck's connection: `btd`'s half of the conversation, without a radio underneath it.
async fn serve(duck: &Duck, air: &Air, range: f64, weather: Weather) -> std::io::Result<()> {
    let stream = UnixStream::connect(&duck.socket).await?;
    let (read_half, mut write_half) = stream.into_split();
    let mut lines = BufReader::new(read_half).lines();

    let request = proto::Request::call(proto::Id::Number(1), &proto::Call::ChoraleSubscribe);
    let mut line = serde_json::to_string(&request).map_err(std::io::Error::other)?;
    line.push('\n');
    write_half.write_all(line.as_bytes()).await?;
    tracing::info!(duck = %duck.name, "on the air");

    let mut deliveries = tokio::time::interval(DELIVERY);
    let mut delivered = usize::MAX;
    let mut tick = 0u64;
    loop {
        tokio::select! {
            line = lines.next_line() => {
                let Some(line) = line? else { return Ok(()) };
                let Ok(request) = serde_json::from_str::<proto::Request>(&line) else { continue };
                let Ok(proto::Call::ChoraleBeaconSet(want)) = request.as_call() else { continue };
                let mut on_air = air.lock().await;
                if let Some(entry) = on_air.get_mut(&duck.name) {
                    let changed = entry.listening != want.listening
                        || entry.beacon.is_some() != want.beacon.is_some();
                    if entry.beacon.is_none() && want.beacon.is_some() {
                        entry.since = Some(Instant::now());
                    }
                    if want.beacon.is_none() {
                        entry.since = None;
                    }
                    entry.beacon = want.beacon.clone();
                    entry.listening = want.listening;
                    if changed {
                        tracing::info!(
                            duck = %duck.name,
                            advertising = entry.beacon.is_some(),
                            listening = entry.listening,
                            "on the air"
                        );
                    }
                }
            }
            _ = deliveries.tick() => {
                tick += 1;
                let heard = nearby(duck, air, range, weather, tick).await;
                if heard.len() != delivered {
                    delivered = heard.len();
                    tracing::info!(duck = %duck.name, hears = delivered, "who is in range");
                }
                for heard in heard {
                    let notify = proto::Request::notify(&proto::Call::ChoraleHeard(heard));
                    let mut line = serde_json::to_string(&notify).map_err(std::io::Error::other)?;
                    line.push('\n');
                    if write_half.write_all(line.as_bytes()).await.is_err() {
                        return Ok(());
                    }
                }
            }
        }
    }
}

/// Every beacon this duck is close enough to hear.
async fn nearby(
    duck: &Duck,
    air: &Air,
    range: f64,
    weather: Weather,
    tick: u64,
) -> Vec<proto::ChoraleHeard> {
    let on_air = air.lock().await;
    let Some(me) = on_air.get(&duck.name) else {
        return Vec::new();
    };
    if !me.listening {
        return Vec::new();
    }
    let here = me.at;

    let mut heard = Vec::new();
    for (name, other) in on_air.iter() {
        if name == &duck.name {
            continue;
        }
        let Some(beacon) = &other.beacon else {
            continue;
        };
        let distance = ((other.at[0] - here[0]).powi(2) + (other.at[1] - here[1]).powi(2)).sqrt();
        if distance > range {
            continue;
        }

        // Not noticed yet. Per pair, and timed from when the other duck went on the air: it is the
        // asymmetry that splits a flock, so one delay shared by everybody would not produce it.
        if !weather.discovery.is_zero() {
            let pair = format!("{}<-{}", duck.name, name);
            let wait = Duration::from_millis(
                noise(weather.seed, &pair, 0) % (weather.discovery.as_millis() as u64).max(1),
            );
            match other.since {
                Some(since) if since.elapsed() >= wait => {}
                _ => continue,
            }
        }

        // And an advertisement is missed often. Seeded by the pair and the tick, so the same run
        // drops the same frames.
        if weather.loss > 0.0 {
            let pair = format!("{}<-{}", duck.name, name);
            let roll = (noise(weather.seed, &pair, tick) % 10_000) as f64 / 10_000.0;
            if roll < weather.loss {
                continue;
            }
        }
        heard.push(proto::ChoraleHeard {
            beacon: beacon.clone(),
            from: other.address.clone(),
            // An age rather than a timestamp — see the module comment. Zero would claim the
            // advertisement arrived at the instant it is being handed over, which is the one
            // reading a real scanner never produces.
            age_us: 1_000,
        });
    }
    heard
}

/// Where every duck is standing, asked of its simulator.
async fn positions(ducks: Vec<Duck>, air: Air) {
    let mut ticker = tokio::time::interval(Duration::from_millis(250));
    loop {
        ticker.tick().await;
        for duck in &ducks {
            if let Some(at) = ask_where(duck.body).await
                && let Some(entry) = air.lock().await.get_mut(&duck.name)
            {
                entry.at = at;
            }
        }
    }
}

async fn ask_where(port: u16) -> Option<[f64; 3]> {
    let stream = TcpStream::connect(("127.0.0.1", port)).await.ok()?;
    let (read_half, mut write_half) = stream.into_split();
    let mut lines = BufReader::new(read_half).lines();
    write_half
        .write_all(b"{\"op\":\"hello\",\"protocol\":1,\"joints\":15}\n")
        .await
        .ok()?;
    lines.next_line().await.ok()??;
    write_half.write_all(b"{\"op\":\"read\"}\n").await.ok()?;
    let answer = lines.next_line().await.ok()??;
    let value: serde_json::Value = serde_json::from_str(&answer).ok()?;
    let trunk = value.get("trunk")?.as_array()?;
    Some([
        trunk.first()?.as_f64()?,
        trunk.get(1)?.as_f64()?,
        trunk.get(2)?.as_f64()?,
    ])
}

/// Give every duck a new address, as a real radio does, so that nothing may key on the old one.
async fn rotate(ducks: Vec<Duck>, air: Air, seconds: u64) {
    let mut ticker = tokio::time::interval(Duration::from_secs(seconds));
    let started = Instant::now();
    ticker.tick().await;
    loop {
        ticker.tick().await;
        let generation = started.elapsed().as_secs() / seconds.max(1);
        let mut on_air = air.lock().await;
        for (index, duck) in ducks.iter().enumerate() {
            if let Some(entry) = on_air.get_mut(&duck.name) {
                entry.address = address_for(index, generation);
            }
        }
        tracing::info!(generation, "every duck has a new address");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_duck_is_a_name_a_socket_and_a_body() {
        let duck = parse_duck("duck-a=/run/duck-a/robotd.sock@7801").expect("parses");
        assert_eq!(duck.name, "duck-a");
        assert_eq!(duck.socket, PathBuf::from("/run/duck-a/robotd.sock"));
        assert_eq!(duck.body, 7801);
    }

    #[test]
    fn a_socket_path_may_contain_an_at_sign() {
        // Split from the right: the port is the last `@`, and a path is allowed to be strange.
        let duck = parse_duck("d=/tmp/o@d/robotd.sock@7999").expect("parses");
        assert_eq!(duck.socket, PathBuf::from("/tmp/o@d/robotd.sock"));
        assert_eq!(duck.body, 7999);
    }

    #[test]
    fn what_is_not_a_duck_says_so() {
        assert!(parse_duck("duck-a").is_err());
        assert!(parse_duck("duck-a=/path/without/a/port").is_err());
        assert!(parse_duck("duck-a=/path@not-a-number").is_err());
    }

    #[test]
    fn every_duck_has_its_own_address_and_a_new_one_each_generation() {
        let first: Vec<String> = (0..4).map(|i| address_for(i, 0)).collect();
        let later: Vec<String> = (0..4).map(|i| address_for(i, 7)).collect();
        for window in first.windows(2) {
            assert_ne!(window[0], window[1], "two ducks shared an address");
        }
        for (a, b) in first.iter().zip(&later) {
            assert_ne!(a, b, "an address survived a rotation: {a}");
        }
        assert_eq!(first[0].len(), "E0:00:01:00:00:00".len());
    }
}
