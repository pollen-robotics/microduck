//! The duck chorale's radio: a beacon out, and other ducks' beacons in.
//!
//! **Still a transport adapter.** `btd` owns no chorale state and makes no chorale decisions — it
//! broadcasts bytes it was handed and reports bytes it heard, which is the same job it does for
//! JSON-RPC. Who conducts, who sings what, and when a piece starts are all `robotd`'s, because
//! they are behaviour.
//!
//! It lives here rather than in a daemon of its own because everything it needs is already here:
//! the adapter, `bluer`, the D-Bus permissions, and a connection to `robotd`. A second process
//! contending for the same adapter would be more moving parts for no separation that matters.
//!
//! ## Out: a second advertising instance
//!
//! The board reports five advertising instances with one in use, so the beacon gets its own and
//! **the existing advertisement is not touched.** That is not tidiness. `duck_ble::adv` documents a
//! 31-byte budget, and the controller here reports a 251-byte one — so BlueZ would happily accept
//! a bigger payload on the existing instance and, because it picks legacy against extended PDUs by
//! size, would switch it to extended and make the robot invisible to a legacy-only scanner. Phone
//! discovery would regress and it would look like a Bluetooth fault rather than a chorale one.
//!
//! ## Registered on demand, and this one is load-bearing
//!
//! A controller interleaves its advertising instances, so registering a second one **halves the
//! rate of the first**. `crate::bluez`'s interval was tuned against measurements — the default
//! 1.28 s left a robot absent for up to 31 s at a time, and 100–150 ms fixed it — so permanently
//! halving it would spend that hard-won margin on a feature nobody has asked for yet. The beacon
//! is therefore registered only while a chorale is wanted, and dropped when it is not.
//!
//! ## In: two ways to listen, because the good one is not always there
//!
//! The one to want is BlueZ's advertisement monitor ([`bluer::monitor`]): it filters on a byte
//! pattern *in the controller*, so the host is woken only for advertisements that are already
//! chorale beacons, and it is passive — the duck transmits nothing to listen, which matters when
//! one antenna carries this, the gamepad's link and wifi.
//!
//! **It is not on this board.** `AdvertisementMonitorManager1` is still behind `bluetoothd
//! --experimental` in BlueZ 5.82, and the daemon here runs without it, so `RegisterMonitor` comes
//! back `UnknownMethod` and two ducks sit in a room hearing nothing. Turning `--experimental` on
//! globally would enable a set of other unfinished interfaces underneath the phone's only way in,
//! which is a poor trade for a power saving.
//!
//! So [`watch`] tries the monitor and falls back to ordinary discovery, which has been in every
//! BlueZ forever. The costs of the fallback are real and worth knowing:
//!
//!  - **It scans actively**, so the duck transmits. More airtime taken from the gamepad.
//!  - **It needs `duplicate_data`.** BlueZ suppresses repeated advertisement data by default, and a
//!    beacon's *payload* is the whole signal — without this every beat after the first is invisible
//!    and a follower locks onto nothing.
//!  - Filtering happens in the host rather than the controller, so the byte pattern is applied by
//!    [`beacon_in`] after the fact instead of before the wakeup.
//!
//! Each listening session owns its scan and each scan owns its device followers. Stopping the
//! session (including a lost robot socket or adapter) cancels those subscriptions. A new session
//! gets a fresh sighting queue, so buffered events from the old one cannot cross the restart.
//!
//! Either way the discriminator is the same: manufacturer-data on company id `0xFFFF` beginning
//! with [`ChoraleBeacon::TAG`]. That tag is why the *other* advertising instance's four bytes of
//! IPv4 address are not delivered here as a beat.
//!
//! ## What the arrival time means
//!
//! A sighting is stamped when this process sees it, which is after the controller, the kernel and
//! a D-Bus property change. That path adds a few milliseconds and some jitter — and the design
//! upstream ([`sounds::chorale::beat`], in the `sounds` crate) is built for exactly that: it
//! averages the phase over many beats, so anything random averages out, and anything *constant* is
//! common to every duck on identical hardware and so inaudible. What it cannot absorb is a
//! systematic difference between the conductor's idea of when its beat went out and the
//! followers' — which is why that constant wants measuring on hardware rather than assuming.

use std::collections::HashMap;

use duck_ipc_proto::ChoraleBeacon;

/// The company id chorale beacons ride under — the same one [`duck_ble::adv`] uses, for the same
/// reason: `0xFFFF` is the id the SIG reserves for testing and is the correct choice for a project
/// that has not been assigned one.
pub const COMPANY_ID: u16 = duck_ble::adv::COMPANY_ID;

/// The advertising interval for the beacon.
///
/// Faster than [`crate::bluez`]'s 100–150 ms, and deliberately: this one carries a beat, and how
/// promptly a payload change reaches the air *is* the sync error. It is affordable because the
/// instance only exists while a chorale is running — see the module docs.
pub const BEACON_INTERVAL_MIN: std::time::Duration = std::time::Duration::from_millis(20);
pub const BEACON_INTERVAL_MAX: std::time::Duration = std::time::Duration::from_millis(40);

/// The bytes a scan filter has to match, from the start of the manufacturer-data AD field.
///
/// Little-endian company id, then the beacon's tag. Matching in the controller rather than in the
/// host is what makes the scan cost nothing while nothing is singing.
pub fn scan_pattern() -> Vec<u8> {
    let id = COMPANY_ID.to_le_bytes();
    vec![id[0], id[1], ChoraleBeacon::TAG]
}

/// A beacon as the advertisement's manufacturer-data payload.
///
/// The payload rather than the map, mirroring [`duck_ble::adv::address_data`] — and because the two
/// halves want different containers: `bluer` advertises from a `BTreeMap` and reports a scanned
/// device's data as a `HashMap`.
pub fn beacon_data(beacon: &ChoraleBeacon) -> Vec<u8> {
    beacon.to_bytes()
}

/// The beacon in a device's manufacturer data, if there is one.
///
/// `None` for anything else on the same company id — the address field the other instance
/// broadcasts, or another vendor using `0xFFFF`, which anyone may. The tag is the discriminator;
/// see [`ChoraleBeacon::from_bytes`].
pub fn beacon_in(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<ChoraleBeacon> {
    ChoraleBeacon::from_bytes(manufacturer_data.get(&COMPANY_ID)?)
}

#[cfg(target_os = "linux")]
pub use radio::{Sighting, broadcast, run, watch};

#[cfg(target_os = "linux")]
mod radio {
    use std::time::Instant;

    use bluer::adv::{Advertisement, Type};
    use bluer::monitor::{Monitor, MonitorEvent, Pattern, RssiSamplingPeriod};
    use duck_ipc_proto::ChoraleBeacon;
    use futures::StreamExt;
    use tokio::sync::mpsc;

    use super::{BEACON_INTERVAL_MAX, BEACON_INTERVAL_MIN, beacon_data, beacon_in, scan_pattern};

    /// AD type for manufacturer-specific data. What the monitor pattern is matched against.
    const AD_TYPE_MANUFACTURER_DATA: u8 = 0xFF;

    /// One beacon heard from another duck.
    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct Sighting {
        pub beacon: ChoraleBeacon,
        /// Which duck. Only an identity for de-duplicating — a beacon says nothing about who is
        /// broadcasting it beyond its register and tie-break byte.
        pub from: bluer::Address,
        /// When this process saw it. See the module docs for what that is and is not.
        pub at: Instant,
    }

    /// Put a beacon on the air, on its own advertising instance.
    ///
    /// Dropping the returned handle stops it, which is how the instance is released the moment a
    /// chorale ends — the whole reason it is registered on demand.
    ///
    /// Carries neither the service UUID nor a local name: the scan filters on the manufacturer
    /// data, so an 18-byte UUID would buy nothing and a name would only make the payload big
    /// enough to change PDU type.
    ///
    /// **Non-connectable, which `Advertisement::default()` is not.** `bluer` defaults the type to
    /// `Peripheral`, so this instance used to offer a second way in to a daemon that serves one
    /// central — and, worse, a connectable instance is one Linux will not re-enable while a
    /// peripheral-role connection is open (`is_advertising_allowed` in
    /// `net/bluetooth/hci_sync.c`; see [`crate::bluez::advertise`]). A beacon is re-registered on
    /// every change `robotd` asks for, so that rule meant a chorale went silent for as long as a
    /// phone was connected, and came back only when it left. `Broadcast` is what this always meant.
    pub async fn broadcast(
        adapter: &bluer::Adapter,
        beacon: &ChoraleBeacon,
    ) -> bluer::Result<bluer::adv::AdvertisementHandle> {
        adapter
            .advertise(Advertisement {
                manufacturer_data: [(super::COMPANY_ID, beacon_data(beacon))]
                    .into_iter()
                    .collect(),
                // A beacon, not a way in. The robot's front door is the other advertisement, and
                // it is unchanged. `Discoverable` is left unset rather than set false, because
                // `org.bluez.LEAdvertisement` says it shall not be set on a broadcast at all.
                advertisement_type: Type::Broadcast,
                min_interval: Some(BEACON_INTERVAL_MIN),
                max_interval: Some(BEACON_INTERVAL_MAX),
                ..Default::default()
            })
            .await
    }

    /// Serve the chorale for as long as the adapter lives.
    ///
    /// One connection to `robotd`, carrying both directions: a `chorale.subscribe` stream down
    /// saying what to advertise, and `chorale.heard` notifications up saying what arrived. `robotd`
    /// decides everything; this only holds the radio.
    ///
    /// Returns when the connection or the adapter goes away, so the caller restarts it the same way
    /// [`crate::bluez::serve`] restarts on losing an adapter. A `robotd` that is not there yet — the
    /// ordinary case at boot — is a retry, not a failure.
    pub async fn run(
        adapter: &bluer::Adapter,
        robot_socket: &std::path::Path,
    ) -> std::io::Result<()> {
        run_with(
            robot_socket,
            |beacon| async move { broadcast(adapter, &beacon).await },
            |tx| {
                let adapter = adapter.clone();
                async move { watch(&adapter, tx).await }
            },
        )
        .await
    }

    // Keep the socket session real in tests; only advertising and discovery need a radio.
    async fn run_with<A, B, BF, W, WF>(
        robot_socket: &std::path::Path,
        mut advertise: B,
        mut watch: W,
    ) -> std::io::Result<()>
    where
        B: FnMut(ChoraleBeacon) -> BF,
        BF: std::future::Future<Output = bluer::Result<A>>,
        W: FnMut(mpsc::Sender<Sighting>) -> WF,
        WF: std::future::Future<Output = bluer::Result<()>> + Send + 'static,
    {
        use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};

        let stream = tokio::net::UnixStream::connect(robot_socket).await?;
        let (read_half, mut write_half) = stream.into_split();
        let mut lines = BufReader::new(read_half).lines();

        let request = duck_ipc_proto::Request::call(
            duck_ipc_proto::Id::Number(1),
            &duck_ipc_proto::Call::ChoraleSubscribe,
        );
        let mut line = serde_json::to_string(&request).map_err(std::io::Error::other)?;
        line.push('\n');
        write_half.write_all(line.as_bytes()).await?;

        // Held for as long as `robotd` wants them, and dropped the moment it does not: a second
        // advertising instance halves the first one's rate, and a monitor costs the controller.
        // Held only to be dropped: an `AdvertisementHandle` stops advertising when it goes, which
        // is exactly how the instance is released the moment `robotd` says to stop.
        let mut _advertisement: Option<A> = None;
        // Dropping the session aborts the scan, including on socket errors or adapter cancellation.
        // Each scan gets its own queue so a restart cannot forward its predecessor's sightings.
        let mut listening = tokio::task::JoinSet::new();
        let mut heard_rx: Option<mpsc::Receiver<Sighting>> = None;

        loop {
            tokio::select! {
                line = lines.next_line() => {
                    let Some(line) = line? else { return Ok(()) };
                    let Ok(request) = serde_json::from_str::<duck_ipc_proto::Request>(&line) else {
                        continue;
                    };
                    let Ok(duck_ipc_proto::Call::ChoraleBeaconSet(want)) = request.as_call() else {
                        continue;
                    };
                    // Advertise what was asked for, and nothing when nothing was.
                    _advertisement = match &want.beacon {
                        Some(beacon) => match advertise(beacon.clone()).await {
                            Ok(handle) => Some(handle),
                            Err(e) => {
                                tracing::warn!(error = %e, "chorale: cannot advertise the beacon");
                                None
                            }
                        },
                        None => None,
                    };
                    // Advertising awaits BlueZ. Reap a scan that ended during that await before
                    // deciding whether this request needs to start a replacement.
                    if let Some(result) = listening.try_join_next() {
                        heard_rx = None;
                        if let Err(error) = result {
                            tracing::warn!(%error, "chorale: the scan task stopped");
                        }
                    }
                    // And scan only while asked to. Restarting the monitor on every beat would be
                    // absurd, so the task is started once and kept until listening stops.
                    match (want.listening, !listening.is_empty()) {
                        (true, false) => {
                            let (tx, rx) = mpsc::channel::<Sighting>(64);
                            heard_rx = Some(rx);
                            let scan = watch(tx);
                            listening.spawn(async move {
                                if let Err(e) = scan.await {
                                    tracing::warn!(error = %e, "chorale: the scan stopped");
                                }
                            });
                        }
                        (false, true) => {
                            heard_rx = None;
                            listening.shutdown().await;
                            tracing::warn!("chorale: no longer listening");
                        }
                        _ => {}
                    }
                }
                Some(result) = listening.join_next(), if !listening.is_empty() => {
                    heard_rx = None;
                    if let Err(error) = result {
                        tracing::warn!(%error, "chorale: the scan task stopped");
                    }
                }
                Some(sighting) = async {
                    match heard_rx.as_mut() {
                        Some(rx) => rx.recv().await,
                        None => std::future::pending().await,
                    }
                } => {
                    // An *age*, not a timestamp: the two daemons share a machine but not an epoch,
                    // and an age survives the trip down a socket in a way another process's clock
                    // reading does not.
                    let heard = duck_ipc_proto::ChoraleHeard {
                        beacon: sighting.beacon,
                        from: sighting.from.to_string(),
                        age_us: sighting.at.elapsed().as_micros() as u64,
                    };
                    let notify = duck_ipc_proto::Request::notify(
                        &duck_ipc_proto::Call::ChoraleHeard(heard),
                    );
                    let mut line = serde_json::to_string(&notify).map_err(std::io::Error::other)?;
                    line.push('\n');
                    if write_half.write_all(line.as_bytes()).await.is_err() {
                        return Ok(());
                    }
                }
            }
        }
    }

    /// Watch for other ducks' beacons, forever, sending each sighting to `tx`.
    ///
    /// Prefers the controller-offloaded monitor and falls back to ordinary discovery when BlueZ has
    /// no monitor to offer — see the module docs for why that is the common case here and what the
    /// fallback costs.
    pub async fn watch(adapter: &bluer::Adapter, tx: mpsc::Sender<Sighting>) -> bluer::Result<()> {
        match monitor(adapter, tx.clone()).await {
            Ok(()) => Ok(()),
            Err(e) => {
                // Any failure to *register* falls back; a monitor that registered and then died is a
                // different matter and is reported as itself.
                tracing::warn!(
                    error = %e,
                    "chorale: no advertisement monitor (bluetoothd without --experimental?); \
                     falling back to discovery, which scans actively"
                );
                discover(adapter, tx).await
            }
        }
    }

    /// The good path: the controller matches the pattern and only wakes us for beacons.
    async fn monitor(adapter: &bluer::Adapter, tx: mpsc::Sender<Sighting>) -> bluer::Result<()> {
        let manager = adapter.monitor().await?;
        let monitor = manager
            .register(Monitor {
                monitor_type: bluer::monitor::Type::OrPatterns,
                patterns: Some(vec![Pattern {
                    data_type: AD_TYPE_MANUFACTURER_DATA,
                    start_position: 0,
                    content: scan_pattern(),
                }]),
                // Every packet, not just the first sighting of a device: the payload is what
                // changes, and a beat we are told about once is not a beat.
                rssi_sampling_period: Some(RssiSamplingPeriod::All),
                ..Default::default()
            })
            .await?;
        tracing::warn!(pattern = ?scan_pattern(), "chorale: listening (monitor)");

        let devices = monitor.filter_map(|event| async move {
            match event {
                MonitorEvent::DeviceFound(found) => Some(found.device),
                _ => None,
            }
        });
        follow_devices(devices, |address| {
            let device = adapter.device(address);
            let tx = tx.clone();
            async move {
                if let Ok(device) = device {
                    follow(device, tx).await;
                }
            }
        })
        .await;
        Ok(())
    }

    /// The fallback: ordinary discovery, filtered in the host.
    async fn discover(adapter: &bluer::Adapter, tx: mpsc::Sender<Sighting>) -> bluer::Result<()> {
        adapter
            .set_discovery_filter(bluer::DiscoveryFilter {
                transport: bluer::DiscoveryTransport::Le,
                // **Load-bearing.** Without it BlueZ reports a device's advertisement data once and
                // suppresses the repeats — and a beacon's payload is the entire signal, so every
                // beat after the first would be invisible.
                duplicate_data: true,
                ..Default::default()
            })
            .await?;
        let events = adapter.discover_devices_with_changes().await?;
        tracing::warn!("chorale: listening (discovery)");

        let devices = events.filter_map(|event| async move {
            match event {
                bluer::AdapterEvent::DeviceAdded(address) => Some(address),
                _ => None,
            }
        });
        follow_devices(devices, |address| {
            let device = adapter.device(address);
            let tx = tx.clone();
            async move {
                if let Ok(device) = device {
                    follow(device, tx).await;
                }
            }
        })
        .await;
        Ok(())
    }

    // Both discovery paths own their followers: cancelling a scan must cancel its subscriptions,
    // and repeated DeviceFound events must not multiply delivery of the same beacon.
    async fn follow_devices<S, F, Fut>(devices: S, mut follow: F)
    where
        S: futures::Stream<Item = bluer::Address>,
        F: FnMut(bluer::Address) -> Fut,
        Fut: std::future::Future<Output = ()> + Send + 'static,
    {
        futures::pin_mut!(devices);
        let mut followers = tokio::task::JoinSet::new();
        let mut active = std::collections::HashMap::new();
        loop {
            tokio::select! {
                // Reap before rediscovery, so a completed subscription can be opened again.
                biased;
                Some(result) = followers.join_next(), if !followers.is_empty() => {
                    if let Err(error) = result {
                        tracing::warn!(%error, "chorale: a device follower failed");
                    }
                    // Keep failures local to one device, just as normal subscription completion.
                    active.retain(|_, task: &mut tokio::task::AbortHandle| !task.is_finished());
                }
                address = devices.next() => {
                    let Some(address) = address else { break };
                    if let std::collections::hash_map::Entry::Vacant(entry) = active.entry(address) {
                        entry.insert(followers.spawn(follow(address)));
                    }
                }
            }
        }
        followers.shutdown().await;
    }

    /// Watch one device's initial advertisement and subsequent manufacturer-data updates.
    async fn follow(device: bluer::Device, tx: mpsc::Sender<Sighting>) {
        let address = device.address();
        // Whatever it was already advertising when it was found — the first beat is otherwise
        // missed while waiting for a change that has already happened.
        if let Ok(Some(data)) = device.manufacturer_data().await
            && let Some(beacon) = beacon_in(&data)
            && tx
                .send(Sighting {
                    beacon,
                    from: address,
                    at: Instant::now(),
                })
                .await
                .is_err()
        {
            return;
        }
        let Ok(mut events) = device.events().await else {
            return;
        };
        while let Some(event) = events.next().await {
            let bluer::DeviceEvent::PropertyChanged(bluer::DeviceProperty::ManufacturerData(data)) =
                event
            else {
                continue;
            };
            // Stamped here, as early as this process can: everything after is jitter the phase
            // average has to absorb.
            let at = Instant::now();
            let Some(beacon) = beacon_in(&data) else {
                continue;
            };
            if tx
                .send(Sighting {
                    beacon,
                    from: address,
                    at,
                })
                .await
                .is_err()
            {
                return;
            }
        }
    }

    #[cfg(test)]
    mod lifecycle_tests {
        use super::*;
        use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
        use tokio::sync::oneshot;
        use tokio::time::{Duration, timeout};

        /// Aborting discovery must also drop the device subscriptions it created.
        #[tokio::test]
        async fn stopping_discovery_stops_device_followers() {
            let (devices_tx, devices_rx) = mpsc::channel(4);
            let devices = device_stream(devices_rx);
            let (started_tx, started_rx) = oneshot::channel();
            let (stopped_tx, stopped_rx) = oneshot::channel::<()>();
            let mut signals = Some((started_tx, stopped_tx));
            let scan = tokio::spawn(follow_devices(devices, move |_| {
                let (started, stopped) = signals.take().unwrap();
                async move {
                    let _stopped = stopped;
                    started.send(()).unwrap();
                    std::future::pending::<()>().await;
                }
            }));
            devices_tx
                .send(bluer::Address::new([1, 2, 3, 4, 5, 6]))
                .await
                .unwrap();
            started_rx.await.unwrap();
            scan.abort();
            assert!(scan.await.unwrap_err().is_cancelled());
            assert!(
                timeout(Duration::from_secs(1), stopped_rx).await.is_ok(),
                "a device follower survived the end of discovery"
            );
        }

        fn device_stream(
            rx: mpsc::Receiver<bluer::Address>,
        ) -> impl futures::Stream<Item = bluer::Address> {
            futures::stream::unfold(rx, |mut rx| async move {
                rx.recv().await.map(|address| (address, rx))
            })
        }

        /// A repeated discovery must not duplicate a subscription; a finished one can be reopened.
        #[tokio::test]
        async fn discovery_deduplicates_active_followers_and_reopens_finished_ones() {
            let (devices_tx, devices_rx) = mpsc::channel(4);
            let (started_tx, mut started_rx) = mpsc::unbounded_channel();
            let (finished_tx, mut finished_rx) = mpsc::unbounded_channel();
            let scan = tokio::spawn(follow_devices(device_stream(devices_rx), move |address| {
                let started = started_tx.clone();
                let finished = finished_tx.clone();
                async move {
                    let (finish_tx, finish_rx) = oneshot::channel();
                    started.send((address, finish_tx)).unwrap();
                    let _ = finish_rx.await;
                    finished.send(address).unwrap();
                }
            }));
            let first = bluer::Address::new([1, 2, 3, 4, 5, 6]);
            let other = bluer::Address::new([6, 5, 4, 3, 2, 1]);
            devices_tx.send(first).await.unwrap();
            let (address, finish) = started_rx.recv().await.unwrap();
            assert_eq!(address, first);
            devices_tx.send(first).await.unwrap();
            devices_tx.send(other).await.unwrap();
            let (address, _keep_other_alive) = started_rx.recv().await.unwrap();
            assert_eq!(address, other, "rediscovery started a duplicate follower");
            finish.send(()).unwrap();
            assert_eq!(finished_rx.recv().await.unwrap(), first);
            devices_tx.send(first).await.unwrap();
            let (address, _keep_first_alive) = timeout(Duration::from_secs(1), started_rx.recv())
                .await
                .unwrap()
                .unwrap();
            assert_eq!(address, first);
            drop(devices_tx);
            timeout(Duration::from_secs(1), scan)
                .await
                .unwrap()
                .unwrap();
            assert!(
                started_rx.recv().await.is_none(),
                "followers outlived discovery EOF"
            );
        }

        /// One failed device subscription must not stop discovery or cancel unrelated followers.
        #[tokio::test]
        async fn a_panicking_follower_does_not_stop_discovery() {
            let (devices_tx, devices_rx) = mpsc::channel(4);
            let (started_tx, mut started_rx) = mpsc::unbounded_channel();
            let failed = bluer::Address::new([1, 2, 3, 4, 5, 6]);
            let healthy = bluer::Address::new([6, 5, 4, 3, 2, 1]);
            let mut first_attempt = true;
            let scan = tokio::spawn(follow_devices(device_stream(devices_rx), move |address| {
                let started = started_tx.clone();
                let fail = address == failed && std::mem::take(&mut first_attempt);
                async move {
                    let (release, released) = oneshot::channel::<()>();
                    started.send((address, release)).unwrap();
                    assert!(!fail, "injected device follower failure");
                    let _ = released.await;
                }
            }));
            devices_tx.send(healthy).await.unwrap();
            let (_, healthy_release) = started_rx.recv().await.unwrap();
            devices_tx.send(failed).await.unwrap();
            let (_, mut failed_release) = started_rx.recv().await.unwrap();
            failed_release.closed().await;
            devices_tx
                .send(failed)
                .await
                .expect("one failed follower stopped discovery");
            let restarted = timeout(Duration::from_secs(1), started_rx.recv())
                .await
                .unwrap();
            assert!(
                restarted.is_some(),
                "one failed follower stopped all discovery"
            );
            assert!(
                !healthy_release.is_closed(),
                "the healthy follower was cancelled"
            );
            scan.abort();
            let _ = scan.await;
        }

        /// Off/on must discard the old queue, not turn old sightings into a new session's input.
        #[tokio::test]
        async fn stopping_and_restarting_listening_replaces_the_sighting_queue() {
            restarting_listening_replaces_queue(false).await;
        }

        /// A scan that returns must be reaped, so the next request can start discovery again.
        #[tokio::test]
        async fn a_finished_scan_can_be_restarted() {
            restarting_listening_replaces_queue(true).await;
        }

        async fn restarting_listening_replaces_queue(finish_scan: bool) {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("robot.sock");
            let server = tokio::net::UnixListener::bind(&path).unwrap();
            let (scans_tx, mut scans_rx) = mpsc::unbounded_channel();
            let session = tokio::spawn(async move {
                run_with(
                    &path,
                    |_| async { Ok(()) },
                    move |tx| {
                        let scans = scans_tx.clone();
                        async move {
                            let (finish_tx, finish_rx) = oneshot::channel();
                            scans.send((tx, finish_tx)).unwrap();
                            let _ = finish_rx.await;
                            Ok(())
                        }
                    },
                )
                .await
            });
            let (socket, _) = server.accept().await.unwrap();
            let (read, mut write) = socket.into_split();
            let mut lines = BufReader::new(read).lines();
            lines.next_line().await.unwrap().unwrap();
            set_listening(&mut write, true).await;
            let (old, finish) = timeout(Duration::from_secs(1), scans_rx.recv())
                .await
                .unwrap()
                .unwrap();
            if finish_scan {
                finish.send(()).unwrap();
            } else {
                set_listening(&mut write, false).await;
            }
            timeout(Duration::from_secs(1), old.closed()).await.unwrap();
            set_listening(&mut write, true).await;
            let (current, _keep_scan_running) = timeout(Duration::from_secs(1), scans_rx.recv())
                .await
                .unwrap()
                .unwrap();
            assert!(old.is_closed());
            current
                .send(Sighting {
                    beacon: ChoraleBeacon {
                        piece: 2,
                        beat: 91,
                        register: 56,
                        id: 0x3D,
                        roster: vec![],
                    },
                    from: bluer::Address::new([1, 2, 3, 4, 5, 6]),
                    at: Instant::now(),
                })
                .await
                .unwrap();
            let line = timeout(Duration::from_secs(1), lines.next_line())
                .await
                .unwrap()
                .unwrap()
                .unwrap();
            let request: duck_ipc_proto::Request = serde_json::from_str(&line).unwrap();
            assert!(matches!(
                request.as_call().unwrap(),
                duck_ipc_proto::Call::ChoraleHeard(_)
            ));
            drop(lines);
            drop(write);
            timeout(Duration::from_secs(1), session)
                .await
                .unwrap()
                .unwrap()
                .unwrap();
            timeout(Duration::from_secs(1), current.closed())
                .await
                .unwrap();
        }

        /// Advertising awaits BlueZ; a scan that finishes during that await must not hide a restart.
        #[tokio::test]
        async fn a_scan_finishing_during_advertising_does_not_swallow_the_start_request() {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("robot.sock");
            let server = tokio::net::UnixListener::bind(&path).unwrap();
            let (scans_tx, mut scans_rx) = mpsc::unbounded_channel();
            let (ads_tx, mut ads_rx) = mpsc::unbounded_channel();
            let session = tokio::spawn(async move {
                run_with(
                    &path,
                    move |_| {
                        let ads = ads_tx.clone();
                        async move {
                            let (resume_tx, resume_rx) = oneshot::channel();
                            ads.send(resume_tx).unwrap();
                            resume_rx.await.unwrap();
                            Ok(())
                        }
                    },
                    move |tx| {
                        let scans = scans_tx.clone();
                        async move {
                            let (finish_tx, finish_rx) = oneshot::channel();
                            let (done_tx, done_rx) = oneshot::channel();
                            scans.send((tx, finish_tx, done_rx)).unwrap();
                            let _ = finish_rx.await;
                            let _ = done_tx.send(());
                            Ok(())
                        }
                    },
                )
                .await
            });
            let (socket, _) = server.accept().await.unwrap();
            let (read, mut write) = socket.into_split();
            let mut lines = BufReader::new(read).lines();
            lines.next_line().await.unwrap().unwrap();
            set_listening(&mut write, true).await;
            let (_old, finish, done) = timeout(Duration::from_secs(1), scans_rx.recv())
                .await
                .unwrap()
                .unwrap();
            let request = duck_ipc_proto::Request::notify(&duck_ipc_proto::Call::ChoraleBeaconSet(
                duck_ipc_proto::ChoraleAdvertise {
                    beacon: Some(ChoraleBeacon {
                        piece: 2,
                        beat: 91,
                        register: 56,
                        id: 0x3D,
                        roster: vec![],
                    }),
                    listening: true,
                },
            ));
            write
                .write_all(format!("{}\n", serde_json::to_string(&request).unwrap()).as_bytes())
                .await
                .unwrap();
            let resume = timeout(Duration::from_secs(1), ads_rx.recv())
                .await
                .unwrap()
                .unwrap();
            finish.send(()).unwrap();
            done.await.unwrap();
            resume.send(()).unwrap();
            let restarted = timeout(Duration::from_secs(1), scans_rx.recv()).await;
            session.abort();
            let _ = session.await;
            assert!(
                restarted.is_ok(),
                "a finished scan swallowed the start request while advertising"
            );
        }

        async fn set_listening(write: &mut tokio::net::unix::OwnedWriteHalf, listening: bool) {
            let request = duck_ipc_proto::Request::notify(&duck_ipc_proto::Call::ChoraleBeaconSet(
                duck_ipc_proto::ChoraleAdvertise {
                    beacon: None,
                    listening,
                },
            ));
            write
                .write_all(format!("{}\n", serde_json::to_string(&request).unwrap()).as_bytes())
                .await
                .unwrap();
        }

        /// Reconnecting robotd must not leave the old radio scan running behind its new session.
        #[tokio::test]
        async fn robot_disconnect_stops_the_listener() {
            ending_robot_session_stops_listener(false).await;
        }

        /// Adapter bring-up cancels this session externally; Drop must clean up that path too.
        #[tokio::test]
        async fn cancelling_robot_session_stops_the_listener() {
            ending_robot_session_stops_listener(true).await;
        }

        async fn ending_robot_session_stops_listener(cancel: bool) {
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("robot.sock");
            let server = tokio::net::UnixListener::bind(&path).unwrap();
            let (started_tx, started_rx) = oneshot::channel();
            let (stopped_tx, stopped_rx) = oneshot::channel::<()>();
            let mut signals = Some((started_tx, stopped_tx));
            let session = tokio::spawn(async move {
                run_with(
                    &path,
                    |_| async { Ok(()) },
                    move |_| {
                        let (started, stopped) = signals.take().unwrap();
                        async move {
                            let _stopped = stopped;
                            started.send(()).unwrap();
                            std::future::pending::<bluer::Result<()>>().await
                        }
                    },
                )
                .await
            });
            let (socket, _) = server.accept().await.unwrap();
            let (read, mut write) = socket.into_split();
            let mut lines = BufReader::new(read).lines();
            lines.next_line().await.unwrap().unwrap(); // chorale.subscribe
            set_listening(&mut write, true).await;
            started_rx.await.unwrap();
            if cancel {
                session.abort();
                assert!(session.await.unwrap_err().is_cancelled());
            } else {
                drop(lines);
                drop(write);
                timeout(Duration::from_secs(1), session)
                    .await
                    .unwrap()
                    .unwrap()
                    .unwrap();
            }
            assert!(
                timeout(Duration::from_secs(1), stopped_rx).await.is_ok(),
                "the radio listener survived the robot socket disconnect"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn beacon() -> ChoraleBeacon {
        ChoraleBeacon {
            piece: 2,
            beat: 91,
            register: 56,
            id: 0x3D,
            roster: vec![(56, 0x3D), (180, 0x11)],
        }
    }

    fn advertised(beacon: &ChoraleBeacon) -> HashMap<u16, Vec<u8>> {
        HashMap::from([(COMPANY_ID, beacon_data(beacon))])
    }

    /// The round trip the broadcasting half and the scanning half both depend on — the same
    /// property `duck_ble::adv` pins for the address field, and for the same reason.
    #[test]
    fn a_beacon_survives_the_advertisement() {
        assert_eq!(beacon_in(&advertised(&beacon())), Some(beacon()));
    }

    /// The trap the tag exists to close: the *other* advertising instance broadcasts four bytes of
    /// IPv4 under the same company id, and a scanner that read those as a beacon would hear a beat
    /// in an address.
    #[test]
    fn the_address_instance_is_not_heard_as_a_beat() {
        let address = HashMap::from([(
            COMPANY_ID,
            duck_ble::adv::address_data(Some(std::net::Ipv4Addr::new(192, 168, 1, 42))),
        )]);
        assert_eq!(beacon_in(&address), None);
        // And the reverse: a beacon is not read as an address, so a scanning `duck-btctl` does not
        // report a robot at some nonsense IP.
        let as_advertised = advertised(&beacon());
        assert_eq!(duck_ble::adv::address_in(&as_advertised), None);
        assert!(
            !duck_ble::adv::has_address_field(&as_advertised),
            "a beacon is not four bytes, so it is not an address field"
        );
    }

    /// The scan filter has to match what the beacon actually broadcasts, byte for byte, or the
    /// controller drops every beat and the failure looks like a radio problem.
    #[test]
    fn the_scan_pattern_matches_what_is_broadcast() {
        let pattern = scan_pattern();
        // The AD field is the little-endian company id followed by the payload.
        let mut field = COMPANY_ID.to_le_bytes().to_vec();
        field.extend(beacon_data(&beacon()));
        assert!(
            field.starts_with(&pattern),
            "pattern {pattern:?} does not prefix the broadcast field {field:?}"
        );
        // Company id first, little-endian, then the tag — the order a controller matches in.
        assert_eq!(pattern, vec![0xFF, 0xFF, ChoraleBeacon::TAG]);
        // An address advertisement must *not* match, or the filter buys nothing.
        let mut address_field = COMPANY_ID.to_le_bytes().to_vec();
        address_field.extend(duck_ble::adv::address_data(None));
        assert!(!address_field.starts_with(&pattern));
    }

    /// Another vendor on the testing company id, or a future beacon this build does not know, is
    /// not a beat. `0xFFFF` is unassigned and anyone may use it.
    #[test]
    fn somebody_elses_payload_is_not_a_beacon() {
        for payload in [vec![], vec![0x01], vec![0xC0], vec![0xC0, 1, 2, 3, 4, 5]] {
            let data = HashMap::from([(COMPANY_ID, payload.clone())]);
            assert_eq!(beacon_in(&data), None, "{payload:?}");
        }
        // A different company id is not ours however well-formed it looks.
        let elsewhere = HashMap::from([(0x004C, beacon().to_bytes())]);
        assert_eq!(beacon_in(&elsewhere), None);
    }

    /// The beacon advertises faster than the front door, and that is only affordable because the
    /// instance is transient. If someone makes it permanent, these numbers are the argument.
    #[test]
    fn the_beacon_is_faster_than_the_front_door() {
        assert!(BEACON_INTERVAL_MIN < BEACON_INTERVAL_MAX);
        // The spec's floor is 20 ms; going under it would be refused by the controller.
        assert!(BEACON_INTERVAL_MIN >= std::time::Duration::from_millis(20));
        // And genuinely faster than the always-on advertisement, or a beat would reach the air no
        // sooner than the robot's name does.
        assert!(BEACON_INTERVAL_MAX < std::time::Duration::from_millis(100));
    }
}
