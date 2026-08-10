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
//! **Untested against hardware.** It type-checks for aarch64 and has never met a real central.
//! Treat what follows as intent until someone connects a phone.

use std::sync::Arc;
use std::time::Duration;

use bluer::adv::Advertisement;
use bluer::agent::Agent;
// Aliased: `bluer` has two error types called `ReqError`, one for the pairing agent and one for a
// characteristic. Naming this one makes a mix-up a name error rather than a puzzling type error,
// which is how it first presented.
use bluer::gatt::local::ReqError as GattError;
use bluer::gatt::local::{
    Application, Characteristic, CharacteristicNotify, CharacteristicNotifyMethod,
    CharacteristicRead, CharacteristicWrite, CharacteristicWriteMethod, Service,
};
use futures::FutureExt;
use std::sync::Mutex as StdMutex;

use tokio::sync::mpsc;

use crate::gatt::{RPC_UUID, SERVICE_UUID};
use crate::link::Link;
use crate::session;
use crate::upstream::Sockets;

/// Notification payload assumed for outbound chunks.
///
/// The write side learns the negotiated MTU (BlueZ reports it per request); the notify side has no
/// way to ask. So chunks are sized for 20 bytes — the payload every BLE link is required to
/// support — which is slower than necessary on a good link and correct on every link.
const FLOOR_MTU: usize = 20;

/// How long to wait between attempts to find a usable adapter.
///
/// Measured on the board: `hci0` does not exist until roughly 73 seconds after power-on —
/// `aic-bluetooth.service` attaches the AIC8800's UART late, and `bluetooth.service` itself
/// spends 26s blocked behind `dbus`. A daemon that exited on "no adapter" would be restarted by
/// systemd into the same emptiness for over a minute, so it waits. Same lesson as `robotd`
/// waiting for the motor bus rather than giving up on it.
const ADAPTER_RETRY: Duration = Duration::from_secs(5);

/// Wait for an adapter, then advertise and serve until cancelled.
///
/// `require_pairing` controls whether writing a request needs an authenticated, encrypted link.
/// It defaults on, because §7 requires it for anything carrying wifi credentials and
/// `net.connect` now does. The opt-out exists for bench work against a client that cannot pair.
pub async fn serve(sockets: Sockets, name: String, require_pairing: bool) -> bluer::Result<()> {
    let bt = bluer::Session::new().await?;

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

    tracing::warn!(
        adapter = adapter.name(),
        address = %adapter.address().await?,
        service = %SERVICE_UUID,
        pairing = require_pairing,
        "serving BLE"
    );

    // The advertised name is what someone sees in a phone's Bluetooth list, so it is the robot's
    // name rather than the service's. `system.setName` will rewrite it once `configd` exists;
    // until then it is the hostname, which is at least unique per board.
    let advertisement = Advertisement {
        service_uuids: [SERVICE_UUID].into_iter().collect(),
        discoverable: Some(true),
        local_name: Some(name),
        ..Default::default()
    };
    let _adv = adapter.advertise(advertisement).await?;

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
                    // must use the acknowledged form, which is why `btctl` does.
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
                        async move {
                            tokio::spawn(async move {
                                // A fresh session, so nothing from a previous central can leak
                                // into this one.
                                let (link, inbound, mut outbound) =
                                    Link::pair(FLOOR_MTU, "central");
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
                                        () = notifier.stopped() => break,
                                        chunk = outbound.recv() => match chunk {
                                            None => break,
                                            Some(chunk) => {
                                                if let Err(e) = notifier.notify(chunk).await {
                                                    tracing::debug!(
                                                        error = %e, "notify failed; central gone"
                                                    );
                                                    break;
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
                                        tracing::info!("central unsubscribed; session discarded");
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

    // The advertisement and application handles deregister on drop, so this task must outlive
    // the service.
    std::future::pending::<()>().await;
    Ok(())
}
