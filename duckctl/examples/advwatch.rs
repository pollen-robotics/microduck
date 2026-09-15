//! `advwatch` — how often does a robot's advertisement actually arrive?
//!
//! `duckctl` scans for eight seconds and either finds a robot or does not, which makes a slow
//! advertiser look like a broken one. This watches continuously instead and prints the arrival
//! pattern, so "found it on the second try" can be read as a number.
//!
//! It exists because that number is the only way to tell three failures apart: the robot is not
//! advertising, the robot is advertising too rarely to be caught in a scan window, or the client is
//! at fault. **Every other device in range is measured alongside it**, which is what makes the
//! answer conclusive — a robot heard ten times less often than a beacon 55 dB weaker than it is not
//! suffering from range or interference.
//!
//! ```text
//! cargo run -p duckctl --example advwatch -- <robot-name>
//! ```
//!
//! Reads as: arrivals per device with signal strength, then a one-character-per-second timeline for
//! the robot, then the gaps. A gap as long as `duckctl`'s scan window is a run that reports no robot.

use std::time::{Duration, Instant};

use btleplug::api::{Central, CentralEvent, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::Manager;
use duck_ble::gatt::SERVICE_UUID;
use futures::StreamExt;

/// How long to watch.
///
/// Two minutes because the thing being measured is *silences*, and a window has to be long enough to
/// contain several of the worst ones to say anything about how often they happen.
const WATCH: Duration = Duration::from_secs(120);

struct Device {
    arrivals: usize,
    last: f64,
    name: Option<String>,
    rssi: Option<i16>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let wanted = std::env::args().nth(1).unwrap_or("radxa-zero3".to_owned());

    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no Bluetooth adapter")?;

    let mut events = adapter.events().await?;
    adapter.start_scan(ScanFilter::default()).await?;
    eprintln!("watching for {wanted:?} for {WATCH:?}…");

    let start = Instant::now();
    let mut robot: Option<String> = None;
    let mut hits: Vec<f64> = Vec::new();
    let mut total = 0usize;
    // Per-device counts, so the robot's rate is read against the other radios in the same room
    // rather than against a guess at what "normal" is. This is the control, and it is the reason the
    // measurement can rule out range and interference rather than merely suspect them.
    let mut per_device: std::collections::HashMap<String, Device> =
        std::collections::HashMap::new();

    loop {
        let left = WATCH.saturating_sub(start.elapsed());
        if left.is_zero() {
            break;
        }
        let Ok(Some(event)) = tokio::time::timeout(left, events.next()).await else {
            break;
        };
        total += 1;

        let id = match &event {
            CentralEvent::DeviceDiscovered(id)
            | CentralEvent::DeviceUpdated(id)
            | CentralEvent::DeviceConnected(id)
            | CentralEvent::DeviceDisconnected(id) => id.clone(),
            CentralEvent::ServicesAdvertisement { id, .. }
            | CentralEvent::ManufacturerDataAdvertisement { id, .. }
            | CentralEvent::ServiceDataAdvertisement { id, .. } => id.clone(),
            _ => continue,
        };

        let Ok(peripheral) = adapter.peripheral(&id).await else {
            continue;
        };
        let Some(properties) = peripheral.properties().await? else {
            continue;
        };

        // Arrivals, not events: one advertisement reception fires several btleplug events
        // (DeviceUpdated, ServicesAdvertisement, ManufacturerDataAdvertisement…) at the same
        // instant, so counting events overstates how often a device is actually heard by 3-4x.
        let now = start.elapsed().as_secs_f64();
        let entry = per_device.entry(id.to_string()).or_insert(Device {
            arrivals: 0,
            last: -1.0,
            name: properties.local_name.clone(),
            rssi: properties.rssi,
        });
        if now - entry.last > 0.2 {
            entry.arrivals += 1;
            entry.last = now;
        }
        if entry.name.is_none() {
            entry.name = properties.local_name.clone();
        }
        if properties.rssi.is_some() {
            entry.rssi = properties.rssi;
        }

        // Properties accumulate in btleplug, so a name or a UUID seen once keeps identifying this
        // peripheral — which is what makes the id enough after the first sighting.
        let is_robot = properties.local_name.as_deref() == Some(wanted.as_str())
            || properties.services.contains(&SERVICE_UUID)
            || robot.as_deref() == Some(id.to_string().as_str());
        if !is_robot {
            continue;
        }
        if robot.is_none() {
            robot = Some(id.to_string());
            eprintln!(
                "  first sighting at {:.1}s: id={} name={:?} services={}",
                start.elapsed().as_secs_f64(),
                id,
                properties.local_name,
                properties.services.len()
            );
        }
        hits.push(start.elapsed().as_secs_f64());
    }
    let _ = adapter.stop_scan().await;

    println!(
        "\n{total} events from all devices, {} from the robot",
        hits.len()
    );

    let mut ranked: Vec<(&String, &Device)> = per_device.iter().collect();
    ranked.sort_by_key(|device| std::cmp::Reverse(device.1.arrivals));
    println!("heard most often in the same room, over {WATCH:?}:");
    for (id, device) in ranked.iter().take(8) {
        let mine = if robot.as_deref() == Some(id.as_str()) {
            "  <-- the robot"
        } else {
            ""
        };
        println!(
            "  {:4} arrivals  {:>5}  {}{mine}",
            device.arrivals,
            device
                .rssi
                .map(|r| format!("{r}dBm"))
                .unwrap_or_else(|| "?".to_owned()),
            device.name.as_deref().unwrap_or("(no name)")
        );
    }
    if let Some(id) = &robot {
        let device = &per_device[id];
        println!(
            "  the robot: {} arrivals, {} — one every {:.1}s",
            device.arrivals,
            device
                .rssi
                .map(|r| format!("{r}dBm"))
                .unwrap_or_else(|| "?".to_owned()),
            WATCH.as_secs_f64() / device.arrivals.max(1) as f64
        );
    }
    if let Some(robot) = &robot {
        let rank = ranked.iter().position(|(id, _)| *id == robot);
        println!(
            "the robot ranks {} of {} devices heard",
            rank.map(|r| r + 1).unwrap_or(0),
            ranked.len()
        );
    }
    if hits.is_empty() {
        println!("the robot was never heard from in {WATCH:?}");
        return Ok(());
    }

    // One character per second: '#' is a second with at least one report, '.' is silence. The gap
    // structure is the whole question — 8s of '.' is a failed `duckctl` run.
    let seconds = WATCH.as_secs() as usize;
    let mut timeline = vec![b'.'; seconds];
    for hit in &hits {
        let slot = (*hit as usize).min(seconds - 1);
        timeline[slot] = b'#';
    }
    println!("{}", String::from_utf8(timeline).unwrap());

    let mut gaps: Vec<f64> = Vec::new();
    let mut previous = 0.0;
    for hit in &hits {
        gaps.push(hit - previous);
        previous = *hit;
    }
    gaps.push(WATCH.as_secs_f64() - previous);
    gaps.sort_by(|a, b| b.partial_cmp(a).unwrap());
    println!(
        "gaps between reports: worst {:.1}s, then {:?}",
        gaps[0],
        gaps.iter()
            .take(6)
            .map(|g| format!("{g:.1}s"))
            .collect::<Vec<_>>()
    );
    // The *smallest* gaps are the advertising interval: consecutive reports that both arrived cannot
    // be closer together than the robot advertises. Reading it off the arrivals costs nothing, where
    // asking the controller needs root on the board.
    let mut closest = gaps.clone();
    closest.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!(
        "closest reports: {:?}",
        closest
            .iter()
            .take(8)
            .map(|g| format!("{:.0}ms", g * 1000.0))
            .collect::<Vec<_>>()
    );
    let buckets = [0.2, 0.5, 1.0, 2.0, 4.0, 8.0, f64::MAX];
    let mut counts = vec![0usize; buckets.len()];
    for gap in &gaps {
        let slot = buckets.iter().position(|b| gap < b).unwrap();
        counts[slot] += 1;
    }
    println!(
        "gap histogram  <200ms:{} <500ms:{} <1s:{} <2s:{} <4s:{} <8s:{} 8s+:{}",
        counts[0], counts[1], counts[2], counts[3], counts[4], counts[5], counts[6]
    );

    let over_scan = gaps.iter().filter(|g| **g >= 8.0).count();
    println!(
        "{over_scan} gap(s) of 8s or more — each one is a `duckctl` run that would report no robot"
    );
    Ok(())
}
