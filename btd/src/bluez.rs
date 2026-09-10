//! The radio. BlueZ via `bluetoothd`'s D-Bus API, Linux only.
//!
//! Everything here is plumbing between BlueZ and [`crate::session`]'s two channels. No decision
//! about the robot is taken in this file, which is the point: the logic that could be wrong is
//! the logic that is tested, and this is the part that needs a radio.
//!
//! It uses `bluer`'s **callback model**, and the alternative was tried on hardware and does not
//! work. `bluer`'s IO model answers BlueZ's `WriteValue` and `StartNotify` with `NotSupported` —
//! it serves only the `AcquireWrite`/`AcquireNotify` fd paths — and a CoreBluetooth central drove
//! the ordinary methods. The result was a robot that advertised, accepted a connection, accepted a
//! subscription, accepted a write, and delivered none of it to this file: no `central connected`
//! line, no pairing prompt, and a client timing out against a service that was working.
//!
//! The IO model was chosen for a benefit that turns out not to exist. It reports
//! `device_address()` on both halves, which looked necessary for pairing a subscription to the
//! session that should feed it — but `bluer` holds **one** `CharacteristicNotifyState` per
//! characteristic, so there is only ever one notification session to pair with. One central at a
//! time is a property of the stack, not a shortcut taken here.
//!
//! So: one session for the service's lifetime, one notify pump, and a write callback that pushes
//! bytes into it.
//!
//! **What the callback model costs is flow control**, and that bill came due the first time a
//! reply was more than a few kilobytes. `notify` hands a D-Bus signal to the connection and
//! returns; nothing here can ask BlueZ whether the radio has caught up, and nothing reports the
//! notification MTU either. Both gaps are worked around rather than solved: the payload is taken
//! from what BlueZ reports on inbound writes (one ATT MTU serves both directions), and the pump
//! pauses every [`NOTIFY_BURST`] chunks. The IO model has the readiness signal this wants and
//! still cannot be used, for the reason above — it serves only the `Acquire*` paths.

use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::Duration;

use bluer::adv::Advertisement;
use bluer::agent::Agent;
// Aliased: `bluer` has two error types called `ReqError`, one for the pairing agent and one for a
// characteristic. Naming this one makes a mix-up a name error rather than a puzzling type error,
// which is how it first presented.
use bluer::gatt::local::ReqError as GattError;
use bluer::gatt::local::{
    Application, Characteristic, CharacteristicNotifier, CharacteristicNotify,
    CharacteristicNotifyMethod, CharacteristicRead, CharacteristicWrite, CharacteristicWriteMethod,
    Service,
};
use futures::FutureExt;
use std::sync::Mutex as StdMutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use tokio::sync::mpsc;

use crate::gatt::{RPC_UUID, SERVICE_UUID};
use crate::link::Link;
use crate::session;
use crate::upstream::{NameChoice, Sockets};

/// Notification payload a session starts with, before any write has reported the negotiated one.
///
/// 20 bytes is what every BLE link is required to support, so it is the only safe *first* guess.
/// It used to be the guess for the whole session — the notify side has no way to ask BlueZ, which
/// remains true — and that was wrong in two ways. Tenfold more notifications than the link needed
/// was the visible half; the other half is that a reply above roughly 5 KiB tore the session down
/// (see [`NOTIFY_BURST`]). The write side does learn the real MTU, BlueZ reports it on every
/// inbound write, and both directions share one ATT MTU — so the floor now lasts until the
/// client's first write, which is always `system.authenticate`.
const FLOOR_MTU: usize = 20;

/// How many notifications to queue before pausing to let the radio drain.
///
/// **This is the only backpressure the callback model offers, and it has to exist.**
/// `CharacteristicNotifier::notify` emits a D-Bus `PropertiesChanged` signal and returns as soon
/// as the signal is queued on the connection — it does not wait for BlueZ, let alone for the
/// controller — so an unpaced pump queues a whole reply in microseconds. Measured on the board:
/// a ~5 KiB reply at the 20-byte floor (≈265 notifications) got through in 1.8 s, and a ~7 KiB one
/// (≈350) killed the notification session 150 ms in, leaving the client waiting out its idle
/// timeout against a robot that had already torn down the session. `bluer`'s IO model has a real
/// readiness signal for this and cannot be used here — it serves only the `Acquire*` fd paths,
/// which a CoreBluetooth central never drives (see this module's header).
///
/// Sixteen chunks is what a connection interval can plausibly carry, so a small reply — an
/// authentication answer is four chunks — never pauses at all.
const NOTIFY_BURST: usize = 16;

/// How long to pause between bursts. Roughly one connection interval.
const NOTIFY_PAUSE: Duration = Duration::from_millis(20);

/// How many times to re-send a chunk the D-Bus connection would not take, and how long to wait
/// between attempts.
///
/// A `notify` that fails while the session is *not* stopped is a queue that is momentarily full,
/// which is recoverable and was being treated as the central having left — the session was torn
/// down mid-reply and the client learned nothing at all. Only an exhausted budget, or a genuinely
/// stopped session, ends the session now.
const NOTIFY_RETRIES: u32 = 5;
const NOTIFY_RETRY_BACKOFF: Duration = Duration::from_millis(20);

/// How often to advertise, and this is the difference between a robot that is found and one that is
/// not.
///
/// Left unset, BlueZ takes the kernel's default of **1.28 s**, and that was measured against this
/// board from a Mac scanning continuously for two minutes: the robot arrived once every 7.5 s on
/// average with silences of 9 s, 14 s, 17 s and once 31 s. Every other radio in the room — a smart
/// plug at −66 dBm, a beacon at −91 dBm — arrived 130 to 212 times over the same window, against the
/// robot's 16, while the robot was the *strongest* signal there at −36 dBm. So it was not range,
/// not interference and not the client: it simply spoke too rarely to be caught.
///
/// A central scans at a low duty cycle, which is what turns "6× slower" into "absent for seconds at
/// a time" — an eight-second scan that lands in one of those silences finds nothing, and roughly
/// half of them did. The large gaps came out as near-integer multiples of 1.28 s, which is what
/// identified the interval from the arrivals rather than from a guess.
///
/// 100-150 ms is the range ordinary peripherals use, and it is 8-12× the default. Not the 20 ms
/// floor the spec allows: one antenna carries this, a gamepad's LE link and wifi, so airtime spent
/// shouting is taken from the things the robot is for. A *range* rather than one value because a
/// fixed interval can keep colliding with the same neighbour's, which the controller avoids by
/// jittering inside the window.
///
/// Measured again with this installed, same Mac and same two minutes: **151 arrivals, one every
/// 0.8 s, worst silence 3.8 s, and not one silence of 8 s or more.** The failure it was diagnosed
/// from cannot happen at that spacing, which is the point — the margin against an eight-second scan
/// is now a factor of two rather than a coin toss.
const ADV_INTERVAL_MIN: Duration = Duration::from_millis(100);
const ADV_INTERVAL_MAX: Duration = Duration::from_millis(150);

/// A task that does not outlive the bring-up that started it.
///
/// The advertisement, the GATT application and the agent all deregister on drop, and the chorale's
/// radio task has to follow the same rule — a task left running against an adapter that is gone
/// would reconnect to `robotd` forever and advertise on nothing.
struct AbortOnDrop(tokio::task::JoinHandle<()>);

impl Drop for AbortOnDrop {
    fn drop(&mut self) {
        self.0.abort();
    }
}

/// How long to wait between attempts to find a usable adapter.
///
/// Measured on the board: `hci0` does not exist until roughly 73 seconds after power-on —
/// `aic-bluetooth.service` attaches the AIC8800's UART late, and `bluetooth.service` itself
/// spends 26s blocked behind `dbus`. A daemon that exited on "no adapter" would be restarted by
/// systemd into the same emptiness for over a minute, so it waits. Same lesson as `robotd`
/// waiting for the motor bus rather than giving up on it.
const ADAPTER_RETRY: Duration = Duration::from_secs(5);

/// How long to wait for `configd` to say what the robot is called, or what address it has.
///
/// Nothing is blocked on the answer — unlike the PIN, where BlueZ holds a pairing exchange open —
/// so this is generous enough to survive a loaded board rather than tuned for a spinner.
///
/// It bounds [`ask_address`] as much as [`ask_name`], and that matters more there: `net.status`
/// costs `configd` a handful of D-Bus round trips to NetworkManager, and NetworkManager mid-scan is
/// slow. A late answer costs one poll's worth of a stale address, which is what the fallback in
/// those two functions is for.
const ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// How often the advertisement is reconciled with what `configd` says — the name, and the address.
///
/// **Polled rather than event-driven, deliberately.** `btd` forwards `system.setName` to `configd`
/// without reading the reply (`upstream::Pool` merges lines for the client, and interpreting them
/// here is exactly what this daemon avoids), so it does not learn a rename by watching. It could
/// re-ask the moment it forwards one, but the write it just forwarded may not have been applied
/// yet, and a second connection has no ordering guarantee against the first.
///
/// Reconciling instead is fewer moving parts and covers every rename path, including
/// `robotctl system set-name` over the unix socket, which never crosses this process at all. The
/// cost is a socket connect and one line every few seconds, forever, which is far below the noise
/// floor of a daemon that already waits 73 seconds for a radio. A `system.*` notification from
/// `configd` would be the tidier answer and is a protocol change nobody needs yet.
///
/// **The address is asked on the same tick, at the same cadence**, which is faster than a DHCP
/// lease could ever move. Two questions rather than one is a second socket connect and a `net.status`
/// that `configd` answers out of NetworkManager, and splitting the cadences would buy back some of
/// that at the price of a second timer and an address that lags a `wifi connect` by half a minute.
/// The robot has just been given a network at that moment, and the address is the thing whoever did
/// it is waiting to read.
const ADV_POLL: Duration = Duration::from_secs(5);

/// Serve BLE for as long as this process lives, across an adapter that comes and goes.
///
/// Waiting for an adapter to *appear* was never enough. Everything after that wait — powering the
/// adapter, registering the agent, advertising, publishing the GATT application — used to propagate
/// its error out of this function and exit the process, so an adapter that appeared and then
/// misbehaved took `btd` down where an adapter that never appeared did not. On a robot with no
/// network that is the difference between "wifi is unavailable" and "unreachable".
///
/// So the whole bring-up retries in place, on the same 5s cadence as the wait it already did:
///
/// - **radio faults never leave this function.** A `failed` `btd` therefore means a broken binary,
///   which is what admits it to the boot recovery net — see `docs/design/boot-recovery-net.md`;
/// - **and it self-heals.** Exiting non-zero got the same retry from `Restart=always`, but only by
///   spending a process death on it, and only until the day the unit gains a start limit.
///
/// `require_pairing` controls whether writing a request needs an authenticated, encrypted link.
/// It defaults on, because §7 requires it for anything carrying wifi credentials and
/// `net.connect` now does. The opt-out exists for bench work against a client that cannot pair.
pub async fn serve(sockets: Sockets, name: NameChoice, require_pairing: bool) -> bluer::Result<()> {
    loop {
        match serve_on_an_adapter(&sockets, &name, require_pairing).await {
            Ok(()) => tracing::warn!(
                retry_in = ?ADAPTER_RETRY,
                "the adapter is gone; waiting for it to come back"
            ),
            // Not fatal, deliberately: every failure reachable here is a property of the radio or
            // of BlueZ, and none of them is fixed by dying. See this function's own doc comment.
            Err(e) => tracing::warn!(
                error = %e,
                retry_in = ?ADAPTER_RETRY,
                "BLE bring-up failed"
            ),
        }
        tokio::time::sleep(ADAPTER_RETRY).await;
    }
}

/// One bring-up: acquire an adapter, advertise, serve, and return when the adapter goes away.
///
/// The advertisement, GATT application and agent handles all deregister on drop, so returning here
/// is what releases them before the next attempt registers its own.
async fn serve_on_an_adapter(
    sockets: &Sockets,
    name: &NameChoice,
    require_pairing: bool,
) -> bluer::Result<()> {
    let sockets = sockets.clone();
    let name = name.clone();
    let bt = bluer::Session::new().await?;

    // Kept as its own loop rather than folded into the caller's: "no adapter yet" is the ordinary
    // state of a board for its first 73 seconds and reads as progress, while a failure after this
    // point is a fault. Collapsing them would log a fault every 5s during a normal boot.
    let adapter = loop {
        match bt.default_adapter().await {
            Ok(adapter) => break adapter,
            Err(e) => {
                tracing::warn!(error = %e, retry_in = ?ADAPTER_RETRY, "no Bluetooth adapter yet");
                tokio::time::sleep(ADAPTER_RETRY).await;
            }
        }
    };
    adapter.set_powered(true).await?;

    // Pairable only matters while we advertise, and the board reports `Pairable: no` by default.
    // Left open rather than gated behind a window: the PIN carries what a window would add, as
    // long as it is per-robot. See `crate::pairing` for why that was chosen over a button.
    if require_pairing {
        adapter.set_pairable(true).await?;
    }

    // A **just-works** agent: every handler left `None`, which bluer publishes as
    // `NoInputNoOutput`. So the bond needs no interaction and is encrypted but *not*
    // authenticated.
    //
    // This is not the design that was intended. The first version answered BlueZ's passkey request
    // with the stored PIN, which cannot work on a headless robot: in LE passkey entry the roles
    // follow from the declared IO capabilities, so implementing `request_passkey` told macOS "this
    // device can input", and macOS displayed a random code for someone to type into a robot with no
    // keyboard. The reverse is no better — with `DisplayPasskey` the *spec* has BlueZ generate the
    // passkey, so a PIN printed on a sticker cannot be presented at all.
    //
    // The PIN check therefore moved above the link layer: `crate::session` serves nothing until a
    // client passes `system.authenticate`. See `crate::pairing` for the trade that involves.
    let _agent = if require_pairing {
        Some(
            bt.register_agent(Agent {
                request_default: true,
                ..Default::default()
            })
            .await?,
        )
    } else {
        tracing::warn!(
            "pairing NOT required: any device in range can reach the RPC characteristic. The PIN \
             is still enforced by the session. Bench use only."
        );
        None
    };

    // `bd_addr` rather than `address`, because the advertisement now carries an IPv4 one too and a
    // journal with both spelled `address` reads as one field contradicting itself.
    //
    // `max_adv_len` is logged because it is the budget `crate::adv` is written against: the payload
    // fits 31 bytes, and a controller that reports less is the one place that assumption fails. It
    // is the first thing to read if a robot ever advertises its name but no address.
    tracing::warn!(
        adapter = adapter.name(),
        bd_addr = %adapter.address().await?,
        service = %SERVICE_UUID,
        pairing = require_pairing,
        max_adv_len = adapter
            .supported_advertising_capabilities()
            .await
            .ok()
            .flatten()
            .map(|caps| caps.max_advertisement_length),
        "serving BLE"
    );

    // The advertised name is what someone sees in a phone's Bluetooth list, so it is the robot's
    // name rather than the service's — and `configd` owns it. Until this asked, the advertisement
    // carried `/etc/hostname` while `system.setName` wrote a name nothing ever read: every board
    // flashed from one image appeared as `radxa-zero3`, and renaming one changed nothing a phone
    // could see, not even after a restart.
    let advertised = Advertised {
        name: match &name.pinned {
            Some(pinned) => pinned.clone(),
            None => ask_name(&sockets, &name.fallback).await,
        },
        // Asked before the first advertisement rather than left to the first reconcile tick: a
        // robot that boots onto a network it already knows would otherwise broadcast `0.0.0.0` for
        // the first few seconds, and a listing cannot tell that from a robot with no wifi at all.
        address: ask_address(&sockets, None).await,
    };
    let handle = Some(advertise(&adapter, &advertised).await?);

    // The chorale's radio, on its own connection to `robotd` and its own advertising instance. A
    // task rather than part of the session loop: it is not serving a client, and it must not be
    // able to hold up the one thing this daemon exists for. Its failures are its own — a `robotd`
    // that is not up yet is the ordinary case at boot.
    let chorale = {
        let adapter = adapter.clone();
        let robot_socket = sockets.robot.clone();
        tokio::spawn(async move {
            loop {
                if let Err(e) = crate::chorale::run(&adapter, &robot_socket).await {
                    tracing::debug!(error = %e, "chorale: robotd is not answering");
                }
                tokio::time::sleep(ADAPTER_RETRY).await;
            }
        })
    };
    // Aborted when this bring-up ends, so a new adapter gets a new connection rather than one
    // pointing at a radio that has gone.
    let _chorale = AbortOnDrop(chorale);

    // **One session per subscription**, not one per daemon.
    //
    // The first version kept a single session alive for the whole service, which is simpler and
    // wrong: a client that vanishes mid-request leaves a partial line in the reassembler and
    // undelivered chunks in the outbound queue, and the *next* client is handed them. That
    // presented as a reply arriving without its beginning —
    // `":0,"result":{"authenticated":true}}` — which is the tail of a previous run's answer.
    //
    // Created when a central subscribes, torn down when it goes away. Subscribing first is the
    // order every client uses, and a write with no live subscription is refused: there would be
    // nowhere to send the answer.
    //
    // A `std::sync::Mutex` rather than tokio's, deliberately: the write callback must read this
    // without awaiting, because a yield point there lets two chunks swap places. Nothing is held
    // across an await.
    let current: Arc<StdMutex<Option<mpsc::Sender<Vec<u8>>>>> = Arc::new(StdMutex::new(None));
    let for_write = current.clone();
    let for_notify = current.clone();

    // The negotiated payload, written by the write callback and read by the session when it sizes
    // a reply. An atomic rather than a channel or the mutex above, because the write callback may
    // not await and may not block: a yield point there lets two chunks swap places. See
    // [`crate::link::Link::mtu`].
    let mtu = Arc::new(AtomicUsize::new(FLOOR_MTU));
    let write_mtu = mtu.clone();

    // The notify callback below takes ownership of `sockets` for the sessions it spawns, and the
    // reconcile loop at the end outlives it.
    let for_reconcile = sockets.clone();

    let app = Application {
        services: vec![Service {
            uuid: SERVICE_UUID,
            primary: true,
            characteristics: vec![Characteristic {
                uuid: RPC_UUID,
                // A read whose only job is to force a bond before anything is written.
                //
                // §7 requires the characteristic carrying wifi credentials to be paired and
                // encrypted. A read is acknowledged, so an unpaired central gets "insufficient
                // authentication" and starts pairing there and then, which a subscribe cannot do:
                // `CharacteristicNotify` carries no encryption flags at all.
                //
                // NOTE: this is currently the *unencrypted* path in practice — see
                // `docs/design/app-path-design.md` §5.5. Requiring encryption here hangs CoreBluetooth.
                //
                // The value matters less than the fact that reading it needs a bond; the API
                // version is the most useful byte available, and a client that finds a version it
                // does not know can say so before writing anything.
                read: Some(CharacteristicRead {
                    read: true,
                    encrypt_read: require_pairing,
                    fun: Box::new(|req| {
                        // Logged because this read is the pairing trigger, so "did the central get
                        // this far" is the first question when a client hangs.
                        tracing::debug!(peer = %req.device_address, "version read");
                        async move { Ok(vec![duck_ipc_proto::API_VERSION as u8]) }.boxed()
                    }),
                    ..Default::default()
                }),
                write: Some(CharacteristicWrite {
                    write: true,
                    // Write-without-response as well: a chunked request needs no ATT
                    // acknowledgement per chunk. A client that wants a *refusal* to be visible
                    // must use the acknowledged form, which is why `duckctl` does.
                    write_without_response: true,
                    encrypt_write: require_pairing,
                    // No `.await` between receiving a chunk and enqueueing it. BlueZ dispatches
                    // each `WriteValue` as its own task, so a yield point here lets two chunks swap
                    // places — and a reordered chunk corrupts a request silently rather than
                    // failing it. `main` also pins the runtime to one thread for the same reason.
                    method: CharacteristicWriteMethod::Fun(Box::new(move |value, req| {
                        let bytes = value.len();
                        let head =
                            String::from_utf8_lossy(&value[..value.len().min(8)]).to_string();
                        let sender = for_write.lock().expect("write slot poisoned").clone();

                        // What BlueZ says this link negotiated, minus the three bytes of ATT
                        // header a notification cannot use. Stored on every write because it is
                        // free to do so and a central may renegotiate; logged only when it moves,
                        // because the value that matters is the one a reply gets chunked for and
                        // that number had never appeared in the journal at all.
                        let payload = usize::from(req.mtu).saturating_sub(3).max(FLOOR_MTU);
                        let previous = write_mtu.swap(payload, Ordering::Relaxed);
                        let learned = (previous != payload).then_some(payload);

                        let result = match sender {
                            None => {
                                // Nowhere to send an answer, so accepting the request would be a
                                // lie. Clients subscribe first; this is a client that did not.
                                tracing::warn!(
                                    peer = %req.device_address,
                                    "write with no subscription; refusing"
                                );
                                Err(GattError::Failed)
                            }
                            Some(tx) => match tx.try_send(value) {
                                Ok(()) => Ok(()),
                                Err(mpsc::error::TrySendError::Full(_)) => {
                                    // Refusing is recoverable — the client resends. Dropping the
                                    // chunk is not: the line would reassemble into something that
                                    // parses as the wrong thing.
                                    tracing::warn!(
                                        peer = %req.device_address,
                                        "inbound queue full; refusing the write"
                                    );
                                    Err(GattError::Failed)
                                }
                                Err(mpsc::error::TrySendError::Closed(_)) => {
                                    tracing::warn!("the session has ended; refusing the write");
                                    Err(GattError::Failed)
                                }
                            },
                        };

                        async move {
                            if let Some(payload) = learned {
                                tracing::info!(
                                    payload,
                                    "negotiated notification payload; replies are sized for this"
                                );
                            }
                            // Eight bytes of the chunk, so a reordering is visible in the journal
                            // rather than inferred from a parse error three layers up. Truncated
                            // because a request may carry a wifi passphrase.
                            tracing::debug!(
                                peer = %req.device_address,
                                mtu = req.mtu,
                                bytes,
                                ok = result.is_ok(),
                                head = %head,
                                "write"
                            );
                            result
                        }
                        .boxed()
                    })),
                    ..Default::default()
                }),
                notify: Some(CharacteristicNotify {
                    notify: true,
                    method: CharacteristicNotifyMethod::Fun(Box::new(move |mut notifier| {
                        let slot = for_notify.clone();
                        let sockets = sockets.clone();
                        let mtu = mtu.clone();
                        async move {
                            tokio::spawn(async move {
                                // A fresh session, so nothing from a previous central can leak
                                // into this one.
                                // Back to the floor for a new central: the previous one's MTU is
                                // not this one's, and the first write will report the real value
                                // before any reply is chunked.
                                mtu.store(FLOOR_MTU, Ordering::Relaxed);
                                let (link, inbound, mut outbound) =
                                    Link::pair_sharing_mtu(mtu.clone(), "central");
                                let mine = inbound.clone();
                                {
                                    let mut slot = slot.lock().expect("write slot poisoned");
                                    if slot.is_some() {
                                        // bluer keeps one notify state per characteristic, so this
                                        // replaces rather than shares: two clients through one
                                        // reassembly buffer would interleave their requests.
                                        tracing::warn!(
                                            "another central was subscribed; replacing its session"
                                        );
                                    }
                                    *slot = Some(inbound);
                                }
                                let session = tokio::spawn(session::run(link, sockets));
                                tracing::info!("central subscribed");

                                // Notifications queued since the last pause. See `NOTIFY_BURST`
                                // for why a pump with no readiness signal has to pace itself.
                                let mut queued = 0usize;
                                let mut gone = false;

                                loop {
                                    tokio::select! {
                                        // Biased so a central that has gone away is noticed before
                                        // another chunk is pulled out of the queue and lost in the
                                        // notify that follows.
                                        biased;
                                        // Without this the pump only learns the central is gone
                                        // when a notify fails — which needs a reply to send, so a
                                        // client that disconnects while idle would hold the slot
                                        // until the next request arrives for nobody.
                                        () = notifier.stopped() => {
                                            gone = true;
                                            break;
                                        }
                                        chunk = outbound.recv() => match chunk {
                                            None => break,
                                            Some(chunk) => {
                                                if !notify_chunk(&mut notifier, chunk).await {
                                                    gone = true;
                                                    break;
                                                }
                                                queued += 1;
                                                if queued >= NOTIFY_BURST {
                                                    queued = 0;
                                                    tokio::time::sleep(NOTIFY_PAUSE).await;
                                                }
                                            }
                                        },
                                    }
                                }

                                // Only clear the slot if it is still *ours*. This task can outlive
                                // its subscription — a notify to a vanished central takes as long
                                // as BlueZ takes to give up — and by then a reconnecting central may
                                // have installed a newer session, which a blind `take()` would kill.
                                {
                                    let mut slot = slot.lock().expect("write slot poisoned");
                                    if slot.as_ref().is_some_and(|tx| tx.same_channel(&mine)) {
                                        // Dropping the sender ends the session task, which discards
                                        // its reassembly buffer and its upstream connections.
                                        slot.take();
                                        session.abort();
                                        // Which of the two it was, because they need different
                                        // next moves and the old line said "unsubscribed" for
                                        // both — including for a reply this pump could not
                                        // deliver, which is a robot problem wearing a client's
                                        // clothes.
                                        if gone {
                                            tracing::info!(
                                                "the central is gone; session discarded"
                                            );
                                        } else {
                                            tracing::info!(
                                                "the outbound queue closed; session discarded"
                                            );
                                        }
                                    } else {
                                        tracing::debug!(
                                            "a newer session holds the slot; leaving it alone"
                                        );
                                        session.abort();
                                    }
                                }
                            });
                        }
                        .boxed()
                    })),
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        }],
        ..Default::default()
    };
    let _app = adapter.serve_gatt_application(app).await?;

    tracing::info!("GATT application registered; waiting for a central");

    // The advertisement and application handles deregister on drop, so this must not return while
    // the adapter is usable — which used to mean `pending()`, waiting forever. Forever was wrong in
    // one direction: an adapter that disappeared left this task parked on a dead radio, holding
    // handles to nothing and advertising nothing, with no way back short of a restart nobody knew
    // to perform. Returning hands the caller a bring-up on the adapter's next appearance.
    //
    // The advertisement is reconciled *alongside* that wait rather than after it, so losing the
    // adapter ends both: whichever finishes first ends the bring-up, and the reconcile is dropped
    // with the advertisement handle it owns.
    //
    // It runs even when `--name` pins the name, because the address moves on its own and a pinned
    // name never meant a frozen advertisement — before the address was in it, the two were the same
    // thing. So the pin suppresses the *question*, not the loop.
    if name.pinned.is_some() {
        tracing::info!(
            name = %advertised.name,
            "--name pins the advertised name; only the address is reconciled"
        );
    }
    tokio::select! {
        () = watch_adapter(&adapter) => {}
        // Never completes on its own.
        () = reconcile_advertisement(
            &adapter,
            &for_reconcile,
            advertised,
            name.pinned.is_some(),
            handle,
        ) => {}
    }
    Ok(())
}

/// Send one chunk, retrying a queue that is momentarily full. `false` means give up on the session.
///
/// **The distinction this function exists to draw**: `notify` returns the same error for "the
/// central unsubscribed" and "the D-Bus connection would not take this signal", and the pump used
/// to read both as the central having left. So a reply too big for one burst tore down the session
/// mid-line, and the client — still connected, as far as CoreBluetooth was concerned — waited out
/// its idle timeout with no error to report and half an answer in its reassembler. That was
/// diagnosed from a board, not from the journal, because the only line it left was a `debug` one
/// blaming the central.
///
/// `is_stopped` tells them apart: it is closed only when the notification session really has
/// ended. Anything else is retried, and the chunk is cloned because `notify` consumes it and a
/// dropped chunk corrupts the line rather than failing it.
async fn notify_chunk(notifier: &mut CharacteristicNotifier, chunk: Vec<u8>) -> bool {
    for attempt in 1..=NOTIFY_RETRIES {
        match notifier.notify(chunk.clone()).await {
            Ok(()) => return true,
            Err(_) if notifier.is_stopped() => {
                tracing::debug!("the notification session ended mid-reply");
                return false;
            }
            Err(e) => {
                tracing::warn!(
                    attempt,
                    error = %e,
                    "the notification queue would not take a chunk; retrying"
                );
                tokio::time::sleep(NOTIFY_RETRY_BACKOFF).await;
            }
        }
    }
    // A warning rather than a silence, which is the whole point: this ends a session, and
    // whoever is holding the client deserves to find out why from the robot's own journal.
    tracing::warn!(
        retries = NOTIFY_RETRIES,
        "gave up on a notification chunk; ending the session rather than sending a corrupt line"
    );
    false
}

/// What the advertisement says about the robot: what it is called, and where it is on the network.
///
/// One struct rather than two arguments threaded through the reconcile loop, so that "has anything
/// moved" is one comparison. Adding a third field would otherwise mean finding every place that
/// compares the pair — and `crate::adv` explains why there is no room for a third field anyway.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Advertised {
    name: String,
    /// `None` is a robot with no IPv4 address, which goes out as `0.0.0.0` — see [`crate::adv`] for
    /// why the field is broadcast either way.
    address: Option<Ipv4Addr>,
}

impl std::fmt::Display for Advertised {
    /// For the journal, where the interesting line is the one that says what changed.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.address {
            Some(address) => write!(f, "{} at {address}", self.name),
            None => write!(f, "{} with no address", self.name),
        }
    }
}

/// Advertise the service under this name and address, and make the name the adapter's too.
///
/// The handle deregisters on drop, so the caller holds it for as long as the robot should be
/// visible.
///
/// **Both names, because a robot has two and only one of them used to be set.** The advertisement
/// carries a Local Name; the adapter separately serves a GAP Device Name (`0x2A00`), which BlueZ
/// takes from `Adapter.Alias` and which defaults to the hostname. So a renamed robot advertised
/// `duck-5b21` while answering `radxa-zero3` to anyone who read the characteristic — and a client
/// that read it kept the answer:
///
/// - **BlueZ caches it over the advertised name.** `Device1.Name` is what `btleplug` reports, so
///   on Linux a robot is `duck-5b21` until the first connection and `radxa-zero3` after it, and
///   `duckctl --name duck-5b21` then finds nothing. Two scans a minute apart disagreed;
/// - **CoreBluetooth keeps both**, and `btleplug` joins them as `radxa-zero3 [duck-5b21]`;
/// - **a phone's Bluetooth settings shows the GAP name**, which is the case that matters most and
///   the one nothing in this repo could see.
///
/// Setting the alias is therefore part of naming the robot rather than a nicety, and it belongs
/// here so that no path can publish a name without it: [`reconcile_advertisement`] re-advertises on
/// every rename and comes through this function to do it. The alias persists in BlueZ's own state,
/// so the write is skipped when it already says the right thing.
///
/// A failure to set it is logged and not propagated. The alias is worth less than being visible at
/// all, and returning an error here would take the advertisement down with it.
///
/// **The address field is dropped rather than allowed to fail the registration.** The arithmetic in
/// [`crate::adv`] says the payload fits, but the byte that overflows a legacy advertisement is the
/// controller's to count, not ours — and BlueZ refuses the whole registration when it does not fit.
/// On a robot whose only front door may be BLE, that trade is not close: an advertisement with no
/// address is a robot someone can still reach, and a refused one is a robot that has gone dark. Same
/// reasoning as the alias above, one step further down.
async fn advertise(
    adapter: &bluer::Adapter,
    advertised: &Advertised,
) -> bluer::Result<bluer::adv::AdvertisementHandle> {
    let name = advertised.name.as_str();
    if adapter.alias().await.ok().as_deref() != Some(name)
        && let Err(e) = adapter.set_alias(name.to_owned()).await
    {
        tracing::warn!(error = %e, name, "cannot set the adapter alias; the GAP name stays stale");
    }

    let advertisement = |address: Option<Vec<u8>>| Advertisement {
        service_uuids: [SERVICE_UUID].into_iter().collect(),
        manufacturer_data: address
            .map(|data| [(crate::adv::COMPANY_ID, data)].into_iter().collect())
            .unwrap_or_default(),
        discoverable: Some(true),
        local_name: Some(name.to_owned()),
        min_interval: Some(ADV_INTERVAL_MIN),
        max_interval: Some(ADV_INTERVAL_MAX),
        ..Default::default()
    };

    let with_address = advertisement(Some(crate::adv::address_data(advertised.address)));
    match adapter.advertise(with_address).await {
        Ok(handle) => Ok(handle),
        Err(e) => {
            tracing::warn!(
                error = %e,
                "BlueZ refused the advertisement carrying the address; retrying without it, so \
                 `duckctl scan` will show this robot with no address at all"
            );
            adapter.advertise(advertisement(None)).await
        }
    }
}

/// Keep the advertisement in step with what `configd` says — name and address. Never returns.
///
/// Owns the advertisement handle, because changing either means deregistering one advertisement and
/// registering another — nothing else may be holding it while that happens.
///
/// `pinned_name` is `--name`: the name is then this process's own and there is nobody to ask about
/// it, so only the address is reconciled. The loop still runs, because a pinned name does not pin a
/// DHCP lease.
async fn reconcile_advertisement(
    adapter: &bluer::Adapter,
    sockets: &Sockets,
    mut advertised: Advertised,
    pinned_name: bool,
    mut handle: Option<bluer::adv::AdvertisementHandle>,
) {
    loop {
        tokio::time::sleep(ADV_POLL).await;

        let current = Advertised {
            name: if pinned_name {
                advertised.name.clone()
            } else {
                ask_name(sockets, &advertised.name).await
            },
            address: ask_address(sockets, advertised.address).await,
        };
        // `handle` is `None` only after a failed re-advertise, and then the robot is invisible —
        // so retry regardless of whether anything moved.
        if current == advertised && handle.is_some() {
            continue;
        }

        // Deregistered before the replacement is registered: BlueZ is being asked to change one
        // advertisement, and holding two while swapping invites it to refuse the second. The gap is
        // brief, and a central that is already connected does not notice — a connection is not an
        // advertisement.
        drop(handle.take());
        match advertise(adapter, &current).await {
            Ok(new) => {
                if current == advertised {
                    tracing::info!(advertising = %current, "advertising again after a failure");
                } else {
                    tracing::info!(from = %advertised, to = %current, "advertisement changed");
                }
                handle = Some(new);
                advertised = current;
            }
            // Left for the next tick rather than fatal, and never propagated: this is inside a
            // bring-up whose whole point is that radio faults do not end the process.
            Err(e) => tracing::error!(error = %e, advertising = %current, "cannot advertise"),
        }
    }
}

/// What `configd` says the robot is called, or `fallback` if it will not say.
///
/// A failure is `debug` rather than `warn`: this runs every few seconds, and a `configd` that is
/// restarting would otherwise fill the journal with a condition that resolves itself. The startup
/// call is the one that matters, and it is logged by the caller through the name it ends up
/// advertising.
async fn ask_name(sockets: &Sockets, fallback: &str) -> String {
    let socket = sockets.path(crate::route::Upstream::Config);
    match crate::upstream::ask(
        "configd",
        socket,
        &duck_ipc_proto::Call::SystemInfo,
        ASK_TIMEOUT,
    )
    .await
    .and_then(|response| {
        response
            .result_as::<duck_ipc_proto::SystemInfoResult>()
            .map_err(|e| e.to_string())
    }) {
        Ok(info) => info.name,
        Err(e) => {
            tracing::debug!(error = %e, fallback, "configd would not say the robot's name");
            fallback.to_owned()
        }
    }
}

/// What `configd` says the robot's IPv4 address is, or `last` if it will not say.
///
/// **The two failures are not the same answer, and conflating them made the advertisement flap.**
/// `configd` reporting no address is a robot that is not on wifi, and that clears the field.
/// `configd` not answering — restarting, or NetworkManager taking longer than [`ASK_TIMEOUT`] —
/// says nothing about the robot's network, and clearing the field on it would deregister and
/// re-register the advertisement on every tick for as long as the outage lasted, with a client
/// watching the address blink. So an outage keeps the last known address, exactly as [`ask_name`]
/// keeps the last known name.
///
/// Only IPv4, because only IPv4 fits — see [`crate::adv`].
///
/// `debug` rather than `warn` for the same reason as [`ask_name`]: this runs every few seconds.
async fn ask_address(sockets: &Sockets, last: Option<Ipv4Addr>) -> Option<Ipv4Addr> {
    let socket = sockets.path(crate::route::Upstream::Config);
    match crate::upstream::ask(
        "configd",
        socket,
        &duck_ipc_proto::Call::NetStatus,
        ASK_TIMEOUT,
    )
    .await
    .and_then(|response| {
        response
            .result_as::<duck_ipc_proto::NetStatusResult>()
            .map_err(|e| e.to_string())
    }) {
        // Parsed rather than trusted: `ip4` is whatever NetworkManager put in `address-data`, and a
        // string this cannot parse is not something to broadcast four bytes of.
        Ok(status) => status.ip4.and_then(|address| match address.parse() {
            Ok(address) => Some(address),
            Err(e) => {
                tracing::warn!(error = %e, address, "configd reported an unparseable IPv4 address");
                None
            }
        }),
        Err(e) => {
            tracing::debug!(error = %e, "configd would not say the robot's address");
            last
        }
    }
}

/// Return once the adapter stops being usable.
///
/// A poll, not an event stream. `bluer` can report adapter removal, but the failure this has to
/// catch is broader than removal — an adapter still on the bus that answers nothing is the case
/// that used to kill the process — and reading one property covers both without depending on which
/// events BlueZ emits for a radio that is wedged rather than absent.
///
/// The interval is [`ADAPTER_RETRY`] because the cost of noticing late is exactly the cost of
/// retrying late: BLE stays dark a few more seconds, on a daemon that is otherwise idle.
async fn watch_adapter(adapter: &bluer::Adapter) {
    loop {
        tokio::time::sleep(ADAPTER_RETRY).await;
        match adapter.is_powered().await {
            Ok(true) => {}
            // Powered off underneath us — by `bluetoothctl power off`, by a driver reset, or by a
            // suspend. The next bring-up powers it again: on a robot whose only front door may be
            // BLE, an unpowered adapter is not a state to preserve out of politeness.
            Ok(false) => {
                tracing::warn!("the adapter is no longer powered");
                return;
            }
            Err(e) => {
                tracing::warn!(error = %e, "the adapter stopped answering");
                return;
            }
        }
    }
}
