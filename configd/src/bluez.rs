//! Gamepad pairing, over BlueZ's D-Bus API. Linux only.
//!
//! `zbus` rather than `bluer`, which `btd` uses: `bluer` links libdbus (vendored, built with `cc`)
//! and this crate already has a pure-Rust D-Bus stack for NetworkManager. Adding `bluer` here would
//! put a second C dependency in `configd` to make four method calls.
//!
//! ## The order, and why the state decides rather than the return values
//!
//! `connect` **before** `pair`, and `trust` after both. Leading with `Pair()` on an Xbox controller
//! returns `AuthenticationCanceled`; that ordering comes from `microduck_runtime`'s notes and is the
//! one that works on this board. It used to live in a provisioning script's comments and in whoever
//! had done it before; now it is here, once, with the reason attached.
//!
//! But the order is **tried, not enforced**, because BlueZ's replies do not describe what happened:
//!
//!  - `Connect()` on a device BlueZ has never bonded with can answer
//!    `br-connection-profile-unavailable` — there is no profile to connect to *yet*. Refusing there
//!    would reject a pad that `Pair()` would have bonded a moment later, so it is soft-failed.
//!  - `Connect()` on a pad that *does* bond **returns before the bond has completed.** A HID profile
//!    requires an encrypted link, so connecting triggers bonding, and it lands a moment afterwards.
//!  - `Pair()` on a bond already in flight **never answers.** Not `AlreadyExists` — outstanding,
//!    until the timeout.
//!
//! Those last two compose into the failure this was shipped with: read `Paired` straight after
//! `Connect()`, see `false` about a bond in flight, call `Pair()`, wait 30 seconds for a reply that
//! is never coming, and return a timeout about a pad that is by then paired — having never reached
//! `set_trusted`. It presented as "the first pair times out, the second works instantly", the second
//! being fast because the bond was already there.
//!
//! So `Paired` turning true is the ground truth for "this worked". `Connect()` gets
//! [`BOND_SETTLE`] to produce it on its own, and the `Pair()` that follows is raced against the same
//! property rather than believed.
//!
//! Discovery is stopped before connecting, deliberately: BlueZ will accept a `Connect()` during an
//! active scan and it fails intermittently, which presents as a pad that pairs on the second
//! attempt and looks like flaky hardware.
//!
//! ## The agent, and why it is not the default one
//!
//! Pairing needs an agent — something for bluetoothd to ask "is this allowed" — and `btd` already
//! registers one as the **default** agent for the phone path. This registers a second, *non-default*
//! agent, which works because bluetoothd picks the agent belonging to the D-Bus connection that
//! called `Pair()` and only falls back to the default. So `configd` answers for the pairings it
//! starts and `btd` keeps answering for everything else; neither has to know about the other.
//!
//! It is scoped to one device path and rejects anything else, so a pairing request arriving from an
//! unrelated device while the window is open is refused rather than auto-accepted. A pad is
//! just-works — there is no passkey to check — so "accept this one device, for these few seconds,
//! because a human asked" is the entire authorisation, and narrowing it to the device is the only
//! part of that this code controls.
//!
//! ## The board setting that made all of this impossible
//!
//! `Privacy = device` in `/etc/bluetooth/main.conf` stops a pad bonding at all, and nothing in this
//! file can work around it. The trace, with no key on either side:
//!
//! ```text
//! SMP: Pairing Public Key ×2 · Confirm · Random ×2 · DHKey Check
//! > ACL Data RX: SMP: Pairing Failed — Reason: DHKey check failed (0x0b)
//! ```
//!
//! The DHKey check is computed over both devices' addresses, and privacy makes the adapter pair from
//! a resolvable private address rather than its public one, so the two sides compute different
//! values and the pad refuses. `bluetoothctl pair` fails identically, which is what places this
//! below anything here. `Privacy = off` — now what `scripts/setup-board.sh` sets — pairs first time.
//!
//! Worth knowing because the symptom is indistinguishable from the ones this file *can* cause, and
//! it cost most of a day: retrying does not help, `JustWorksRepairing` does not help, and neither
//! does clearing the bond on either side.
//!
//! ## Where this has and has not run
//!
//! **Run against a real BlueZ on a Radxa Zero 3W with an Xbox Wireless Controller**, which is where
//! everything above about asynchronous bonding comes from. What has been seen work: discovery finds
//! the pad and the heuristic identifies it, the bond completes, `Trusted` sticks, the pad reconnects
//! by itself across a reboot, `padd` drives from it, and `pad forget` drops it.
//!
//! What has **not** been exercised on hardware: a DualSense, two pads in pairing mode at once, and
//! pairing by explicit address.
//!
//! And one case that cannot be fixed from here: `pad forget` removes only the robot's half of the
//! bond. A pad that still holds its half will not pair again until it is put back into pairing mode
//! or bonded to something else, and it reports the same `AuthenticationFailed` as everything above.

use std::collections::HashMap;
use std::time::Duration;

use async_trait::async_trait;
use duck_ipc_proto as proto;
use zbus::names::OwnedInterfaceName;
use zbus::zvariant::{ObjectPath, OwnedObjectPath, OwnedValue};

use crate::pad::{PadResult, Pads, looks_like_a_gamepad};

/// Where our pairing agent lives on the bus. Any path we own will do; this one says whose it is.
const AGENT_PATH: &str = "/com/pollenrobotics/configd/pad_agent";

/// `NoInputNoOutput` — the robot has no keypad and no display, which is a fact about the hardware
/// rather than a choice. It is also what makes a pad's pairing just-works.
const AGENT_CAPABILITY: &str = "NoInputNoOutput";

/// How long BlueZ gets to finish bonding once a pad has been found.
///
/// Separate from the caller's discovery window, because they measure different things: the window
/// is how long to wait for a human to hold the sync button, this is how long the radio gets after
/// the device is already in hand. BlueZ's own pairing timeout is 60s; this stays inside it so the
/// answer comes from here rather than from a dropped D-Bus call.
const BOND_TIMEOUT: Duration = Duration::from_secs(30);

/// How long to let a bond triggered by `Connect()` finish on its own before asking for one.
///
/// Bonding is asynchronous and lands a moment after `Connect()` returns, so this is the window in
/// which "it is already happening" is distinguished from "it is not going to". Measured against a
/// real Xbox controller, it completes inside a second; five is margin, and the cost of it being too
/// short is a `Pair()` call on a bond in flight, which never answers.
const BOND_SETTLE: Duration = Duration::from_secs(5);

/// How often to re-read `Paired` while waiting for a bond.
const BOND_POLL: Duration = Duration::from_millis(200);

/// How often to re-read the object tree while looking for a pad.
///
/// Polling rather than `InterfacesAdded`, which sounds like the right signal and is not: BlueZ emits
/// it only for devices it has never seen, so a pad that was paired and forgotten — the exact case
/// someone is retrying — stays in the cache and never announces itself again.
const DISCOVERY_POLL: Duration = Duration::from_millis(500);

#[zbus::proxy(interface = "org.bluez.Adapter1", default_service = "org.bluez")]
trait Adapter {
    fn start_discovery(&self) -> zbus::Result<()>;
    fn stop_discovery(&self) -> zbus::Result<()>;
    fn remove_device(&self, device: &ObjectPath<'_>) -> zbus::Result<()>;

    #[zbus(property)]
    fn powered(&self) -> zbus::Result<bool>;
    #[zbus(property)]
    fn set_powered(&self, on: bool) -> zbus::Result<()>;
}

#[zbus::proxy(interface = "org.bluez.Device1", default_service = "org.bluez")]
trait Device {
    fn connect(&self) -> zbus::Result<()>;
    fn pair(&self) -> zbus::Result<()>;

    #[zbus(property)]
    fn set_trusted(&self, on: bool) -> zbus::Result<()>;
}

#[zbus::proxy(
    interface = "org.bluez.AgentManager1",
    default_service = "org.bluez",
    default_path = "/org/bluez"
)]
trait AgentManager {
    fn register_agent(&self, agent: &ObjectPath<'_>, capability: &str) -> zbus::Result<()>;
    fn unregister_agent(&self, agent: &ObjectPath<'_>) -> zbus::Result<()>;
}

/// An agent that says yes to exactly one device.
///
/// Every handler that could authorise something checks the path it was called about. The ones that
/// would need a keypad refuse: this robot cannot enter a passkey, and answering `0000` on its behalf
/// would be inventing a credential.
struct PairingAgent {
    device: OwnedObjectPath,
}

impl PairingAgent {
    fn permit(&self, device: &ObjectPath<'_>, what: &str) -> zbus::fdo::Result<()> {
        if device.as_str() == self.device.as_str() {
            tracing::info!(device = device.as_str(), what, "authorising, as asked");
            return Ok(());
        }
        // Not the pad someone is pairing. Refusing is the point of scoping the agent: an open
        // pairing window on a robot in a room full of Bluetooth devices should not accept them all.
        tracing::warn!(
            device = device.as_str(),
            expected = self.device.as_str(),
            what,
            "refusing: not the device being paired"
        );
        Err(zbus::fdo::Error::AccessDenied(
            "this robot is not pairing with that device".into(),
        ))
    }
}

#[zbus::interface(name = "org.bluez.Agent1")]
impl PairingAgent {
    fn release(&self) {
        tracing::debug!("pairing agent released");
    }

    /// The one BlueZ actually calls for a just-works bond.
    fn request_authorization(&self, device: ObjectPath<'_>) -> zbus::fdo::Result<()> {
        self.permit(&device, "bond")
    }

    /// Asked per profile once bonded — HID, in a pad's case.
    fn authorize_service(&self, device: ObjectPath<'_>, uuid: String) -> zbus::fdo::Result<()> {
        tracing::debug!(uuid, "service authorisation requested");
        self.permit(&device, "service")
    }

    /// Numeric comparison, when the remote end has a display. Nothing here can compare anything, so
    /// accepting is the only answer that lets a pad bond — and the passkey is logged so it is at
    /// least on the record.
    fn request_confirmation(&self, device: ObjectPath<'_>, passkey: u32) -> zbus::fdo::Result<()> {
        tracing::info!(passkey, "confirmation requested with no way to compare it");
        self.permit(&device, "confirmation")
    }

    /// Refused rather than answered with a guess. With `NoInputNoOutput` declared, BlueZ should
    /// never ask — and if it does, the device wants a credential this robot does not have. Sending
    /// `0000` would be inventing one, and it would fail anyway on anything made this decade.
    fn request_pin_code(&self, device: ObjectPath<'_>) -> zbus::fdo::Result<String> {
        tracing::warn!(
            device = device.as_str(),
            "a PIN was requested; this robot has no keypad"
        );
        Err(zbus::fdo::Error::NotSupported(
            "this robot cannot enter a PIN".into(),
        ))
    }

    /// As above, for LE passkey entry. See `btd::pairing` for the long version of why a headless
    /// robot cannot take part in it.
    fn request_passkey(&self, device: ObjectPath<'_>) -> zbus::fdo::Result<u32> {
        tracing::warn!(
            device = device.as_str(),
            "a passkey was requested; this robot has no keypad"
        );
        Err(zbus::fdo::Error::NotSupported(
            "this robot cannot enter a passkey".into(),
        ))
    }

    /// Display handlers: nothing to display on, so they only reach the journal. Implemented rather
    /// than omitted, because a missing method makes BlueZ fail the bond with a D-Bus error that
    /// says nothing about the cause.
    fn display_passkey(&self, device: ObjectPath<'_>, passkey: u32, entered: u16) {
        tracing::info!(
            device = device.as_str(),
            passkey,
            entered,
            "passkey to display, on a robot with no display"
        );
    }

    fn display_pin_code(&self, device: ObjectPath<'_>, pincode: String) {
        tracing::info!(
            device = device.as_str(),
            pincode,
            "PIN to display, on a robot with no display"
        );
    }

    fn cancel(&self) {
        tracing::warn!("the remote end cancelled pairing");
    }
}

/// One object's interfaces, as `GetManagedObjects` reports them: interface name to properties.
type Interfaces = HashMap<OwnedInterfaceName, HashMap<String, OwnedValue>>;

/// One interface's properties, by name.
///
/// A scan rather than a lookup: the keys are `OwnedInterfaceName`, which cannot be borrowed as a
/// `&str` for `HashMap::get`, and an object carries three or four interfaces.
fn interface<'a>(
    interfaces: &'a Interfaces,
    name: &str,
) -> Option<&'a HashMap<String, OwnedValue>> {
    interfaces
        .iter()
        .find(|(iface, _)| iface.as_str() == name)
        .map(|(_, props)| props)
}

/// What BlueZ currently knows about one device.
///
/// A snapshot from `GetManagedObjects` rather than a live proxy: every field is read together, in
/// one round trip, and there is no cache to be stale. The properties are all optional because BlueZ
/// omits what it does not know — a device seen in discovery but never queried has no `Name`.
#[derive(Debug, Clone)]
struct Snapshot {
    path: OwnedObjectPath,
    mac: String,
    name: String,
    icon: Option<String>,
    class: Option<u32>,
    appearance: Option<u16>,
    paired: bool,
    trusted: bool,
    connected: bool,
}

impl Snapshot {
    fn read(path: &OwnedObjectPath, props: &HashMap<String, OwnedValue>) -> Option<Self> {
        let get = |key: &str| props.get(key).cloned();
        let text = |key: &str| get(key).and_then(|v| String::try_from(v).ok());
        let flag = |key: &str| {
            get(key)
                .and_then(|v| bool::try_from(v).ok())
                .unwrap_or(false)
        };

        Some(Self {
            path: path.clone(),
            // No address, no device: everything here is keyed on it, and a client cannot act on a
            // pad it cannot name.
            mac: text("Address")?,
            // `Alias` is what BlueZ shows and falls back to `Name`, so it is the better of the two
            // — but it is also what a rename would have changed, and either is better than empty.
            name: text("Alias").or_else(|| text("Name")).unwrap_or_default(),
            icon: text("Icon"),
            class: get("Class").and_then(|v| u32::try_from(v).ok()),
            appearance: get("Appearance").and_then(|v| u16::try_from(v).ok()),
            paired: flag("Paired"),
            trusted: flag("Trusted"),
            connected: flag("Connected"),
        })
    }

    fn is_gamepad(&self) -> bool {
        looks_like_a_gamepad(
            &self.name,
            self.icon.as_deref(),
            self.class,
            self.appearance,
        )
    }

    fn as_pad(&self) -> proto::Pad {
        proto::Pad {
            mac: self.mac.clone(),
            name: self.name.clone(),
            paired: self.paired,
            trusted: self.trusted,
            connected: self.connected,
        }
    }
}

/// The result of one search: what looked like a pad, and everything the radio saw.
struct Found {
    matches: Vec<Snapshot>,
    seen: Vec<Snapshot>,
}

/// Pads, through bluetoothd.
pub struct BlueZ {
    bus: zbus::Connection,
    /// One pairing at a time. Two concurrent ones would fight over discovery and over the agent
    /// path, and there is only one adapter and one human holding one pad.
    pairing: tokio::sync::Mutex<()>,
}

impl BlueZ {
    pub async fn new() -> Result<Self, String> {
        let bus = zbus::Connection::system()
            .await
            .map_err(|e| e.to_string())?;
        Ok(Self {
            bus,
            pairing: tokio::sync::Mutex::new(()),
        })
    }

    /// Everything bluetoothd is managing, by object path and interface.
    async fn objects(&self) -> PadResult<HashMap<OwnedObjectPath, Interfaces>> {
        let manager = zbus::fdo::ObjectManagerProxy::new(&self.bus, "org.bluez", "/")
            .await
            .map_err(|e| format!("cannot reach bluetoothd on the system bus: {e}"))?;
        manager
            .get_managed_objects()
            .await
            .map_err(|e| format!("bluetoothd would not list its objects: {e}"))
    }

    /// The first adapter, powered on.
    ///
    /// "First" rather than "hci0 by name": the board has one adapter and naming it would be a
    /// guess that happens to be right. An absent adapter is a normal answer early in a boot — on
    /// this board `hci0` does not exist until roughly 73 seconds after power-on.
    async fn adapter(&self) -> PadResult<Option<AdapterProxy<'static>>> {
        let mut paths: Vec<OwnedObjectPath> = self
            .objects()
            .await?
            .into_iter()
            .filter(|(_, interfaces)| interface(interfaces, "org.bluez.Adapter1").is_some())
            .map(|(path, _)| path)
            .collect();
        // Sorted so "the first adapter" means the same one on every call rather than whatever the
        // hash map yielded.
        paths.sort_by(|a, b| a.as_str().cmp(b.as_str()));

        let Some(path) = paths.into_iter().next() else {
            return Ok(None);
        };

        let adapter = AdapterProxy::builder(&self.bus)
            .path(path)
            .map_err(|e| e.to_string())?
            .build()
            .await
            .map_err(|e| e.to_string())?;

        // An adapter that is present but off finds nothing, and reports it as "no pad" — which
        // sends someone looking at the pad instead of at the radio.
        if !adapter.powered().await.unwrap_or(false) {
            adapter
                .set_powered(true)
                .await
                .map_err(|e| format!("cannot power on the Bluetooth adapter: {e}"))?;
        }
        Ok(Some(adapter))
    }

    /// Devices bluetoothd knows about, newest state each call.
    async fn devices(&self) -> PadResult<Vec<Snapshot>> {
        let objects = self.objects().await?;
        Ok(objects
            .iter()
            .filter_map(|(path, interfaces)| {
                Snapshot::read(path, interface(interfaces, "org.bluez.Device1")?)
            })
            .collect())
    }

    /// Look for a gamepad until `deadline`, then give up.
    ///
    /// **An unbonded pad wins, and the search waits for one.** A robot that already has a pad bonded
    /// still sees it in every sweep — it is in BlueZ's cache whether or not anyone touched it — so
    /// returning on the first match would answer "you already have a pad" to someone standing there
    /// with a second one in pairing mode. That made adding a pad impossible without forgetting the
    /// first, which is the wrong shape: a robot may have several pads bonded, and `padd` drives
    /// whichever connects.
    ///
    /// So with no address given, the sweep only ends early on a candidate that is **not yet paired**;
    /// otherwise it runs to the deadline and reports what it has, which may be the pad already
    /// bonded. The cost is that re-running `pad pair` with nothing new in pairing mode takes the whole
    /// window before saying "already paired" — `--timeout` shortens it.
    ///
    /// An explicit address ends the sweep as soon as it appears, paired or not: the caller has named
    /// what they want and there is nothing to prefer.
    ///
    /// Also returns **everything else it saw**, which matters more than it looks: BlueZ reports a
    /// freshly-discovered device with an address and often nothing else — no `Name`, no `Class`, no
    /// `Icon` — because those need a further exchange. A pad that never resolves any of them is
    /// invisible to [`Snapshot::is_gamepad`], and a bare "no gamepad found" would leave someone with
    /// no way to learn the address that `--mac` needs. So the refusal carries the list.
    async fn find(&self, mac: Option<&str>, timeout: Duration) -> PadResult<Found> {
        let deadline = tokio::time::Instant::now() + timeout;
        loop {
            let seen = self.devices().await?;
            let matches: Vec<Snapshot> = seen
                .iter()
                .filter(|device| match mac {
                    // An explicit address bypasses the heuristic entirely. That is the escape hatch
                    // for hardware this does not recognise, and it must not be second-guessed.
                    Some(wanted) => wanted.eq_ignore_ascii_case(&device.mac),
                    None => device.is_gamepad(),
                })
                .cloned()
                .collect();

            let worth_stopping_for = match mac {
                Some(_) => !matches.is_empty(),
                None => matches.iter().any(|device| !device.paired),
            };
            if worth_stopping_for || tokio::time::Instant::now() >= deadline {
                return Ok(Found { matches, seen });
            }
            tokio::time::sleep(DISCOVERY_POLL.min(deadline - tokio::time::Instant::now())).await;
        }
    }

    /// Watch `Paired` until it turns true, or `within` elapses.
    ///
    /// The property, not a method's return value, because bonding is asynchronous: `Connect()` comes
    /// back before the bond it triggered has completed, and `Pair()` on a bond already in flight
    /// simply never answers. `Paired` is the one thing that says whether this worked.
    async fn wait_until_paired(&self, mac: &str, within: Duration) -> bool {
        let deadline = tokio::time::Instant::now() + within;
        loop {
            let state = self
                .devices()
                .await
                .ok()
                .and_then(|all| all.into_iter().find(|d| d.mac.eq_ignore_ascii_case(mac)));
            if let Some(state) = &state
                && state.paired
            {
                tracing::info!(connected = state.connected, "bonded");
                return true;
            }
            if tokio::time::Instant::now() >= deadline {
                tracing::info!(
                    paired = false,
                    connected = state.is_some_and(|s| s.connected),
                    "still not bonded"
                );
                return false;
            }
            tokio::time::sleep(BOND_POLL.min(deadline - tokio::time::Instant::now())).await;
        }
    }

    /// Connect, pair, trust — in that order, for the reasons in this module's docs.
    async fn bond(&self, device: &Snapshot) -> Result<(), (proto::PadPairFailure, String)> {
        let proxy = DeviceProxy::builder(&self.bus)
            .path(device.path.as_ref())
            .map_err(|e| (proto::PadPairFailure::Other, e.to_string()))?
            .build()
            .await
            .map_err(|e| (proto::PadPairFailure::Other, e.to_string()))?;

        if !device.paired {
            // `Connect()` first, which is the order that works on this board — leading with `Pair()`
            // on an Xbox controller returns `AuthenticationCanceled`.
            //
            // But **soft-failed**, deliberately. A device BlueZ has never bonded with has no known
            // profile to connect to, so `Connect()` can answer
            // `br-connection-profile-unavailable` before pairing has happened — and treating that
            // as the end would refuse a pad that `Pair()` would have bonded a moment later. So the
            // preferred order is tried first and the fallback is still available, rather than the
            // order being enforced against the radio.
            let connected = match tokio::time::timeout(BOND_TIMEOUT, proxy.connect()).await {
                Ok(Ok(())) => {
                    tracing::info!("connected");
                    true
                }
                Ok(Err(e)) => {
                    tracing::info!(error = %e, "connect first failed; asking to pair");
                    false
                }
                Err(_) => {
                    tracing::info!("connect did not answer in time; asking to pair");
                    false
                }
            };

            // Did that bond it on its own? It usually does — a HID profile requires an encrypted
            // link, so connecting triggers bonding — but **it finishes a moment after `Connect()`
            // returns**, not before. Reading `Paired` immediately therefore says `false` about a
            // bond that is already in flight, and the `Pair()` that follows never answers: BlueZ
            // leaves it outstanding rather than reporting `AlreadyExists`.
            //
            // That is the whole of the "first pair times out, second one works instantly" report:
            // the first call bonded the pad, waited 30s for a reply that was never coming, and
            // returned a timeout before it could set `Trusted`.
            // Wait for a bond to land **only if a connect succeeded**, because that is the only case
            // where one is in flight. After a failed connect the wait is dead time, and dead time
            // here does damage: a pad holds pairing mode for a limited window, and spending five
            // seconds of it waiting for something that was never started is how a fresh pairing ends
            // in `AuthenticationFailed`.
            let settle = if connected {
                BOND_SETTLE
            } else {
                Duration::ZERO
            };
            if !self.wait_until_paired(&device.mac, settle).await {
                // Genuinely not bonding on its own, so ask. **Raced against the state**, not
                // trusted to answer: `Paired` turning true is the ground truth for "this worked",
                // and `Pair()`'s reply is only one of the two ways to learn it.
                let paired = tokio::select! {
                    outcome = tokio::time::timeout(BOND_TIMEOUT, proxy.pair()) => match outcome {
                        Ok(Ok(())) => { tracing::info!("bonded"); true }
                        // Something bonded it between the two calls — success, not a failure.
                        Ok(Err(e)) if is_already_paired(&e) => { tracing::info!("already bonded"); true }
                        Ok(Err(e)) => return Err((proto::PadPairFailure::Rejected, e.to_string())),
                        Err(_) => false,
                    },
                    bonded = self.wait_until_paired(&device.mac, BOND_TIMEOUT) => bonded,
                };
                if !paired {
                    return Err((
                        proto::PadPairFailure::Timeout,
                        "the pad did not finish pairing".to_owned(),
                    ));
                }

                // Connect again now that a bond exists, for the case the first attempt failed for
                // want of one. Soft too: a bonded pad reconnects by itself, so failing here is not
                // worth refusing a pairing that succeeded.
                if let Ok(Err(e)) = tokio::time::timeout(BOND_TIMEOUT, proxy.connect()).await {
                    tracing::info!(error = %e, "bonded but not connected yet");
                }
            }
        }

        // Trust is what makes the pad work after a reboot with nobody logged in: an untrusted
        // device's reconnection needs an agent to approve it, and at boot there is none. This is the
        // line whose absence looks like "it paired fine yesterday and does nothing today".
        proxy
            .set_trusted(true)
            .await
            .map_err(|e| (proto::PadPairFailure::Other, e.to_string()))?;
        Ok(())
    }
}

/// Did BlueZ refuse this because the bond already exists?
fn is_already_paired(error: &zbus::Error) -> bool {
    matches!(error, zbus::Error::MethodError(name, _, _)
        if name.as_str() == "org.bluez.Error.AlreadyExists")
}

#[async_trait]
impl Pads for BlueZ {
    async fn status(&self) -> PadResult<Vec<proto::Pad>> {
        let mut pads: Vec<proto::Pad> = self
            .devices()
            .await?
            .into_iter()
            // Bonded pads only. Everything else BlueZ has ever seen in a scan is noise here: the
            // question this answers is "what can drive this robot", not "what is in range".
            .filter(|device| device.paired && device.is_gamepad())
            .map(|device| device.as_pad())
            .collect();
        // Connected first, then by name, so the pad someone is holding is the first line.
        pads.sort_by(|a, b| b.connected.cmp(&a.connected).then(a.name.cmp(&b.name)));
        Ok(pads)
    }

    async fn pair(&self, mac: Option<&str>, timeout: Duration) -> PadResult<proto::PadPairResult> {
        let _one_at_a_time = self.pairing.lock().await;

        let Some(adapter) = self.adapter().await? else {
            return Ok(proto::PadPairResult::Failed {
                reason: proto::PadPairFailure::NoAdapter,
                detail: Some(
                    "no Bluetooth adapter. On this board hci0 appears about 73s after power-on."
                        .to_owned(),
                ),
            });
        };

        // No short-circuit on "a pad is already bonded", deliberately. That reads as an obvious
        // optimisation and it made **adding a second pad impossible**: the bonded one is in BlueZ's
        // cache every sweep, so it always won, and the only way to pair a new pad was to forget the
        // working one first. A robot may have several pads bonded — `padd` drives whichever connects
        // — so the search runs, and `find` prefers a pad that is not yet paired.
        //
        // Idempotence is kept where it belongs instead: if the pad that turns up is already bonded,
        // `bond` skips connecting and pairing and only re-asserts `Trusted`.

        // Discovery has to be running for a first-time bond to resolve an address. A failure here
        // is worth reporting rather than working around: without it the search below can only ever
        // find devices already in BlueZ's cache.
        adapter
            .start_discovery()
            .await
            .map_err(|e| format!("cannot start Bluetooth discovery: {e}"))?;
        tracing::info!(?timeout, "looking for a gamepad in pairing mode");

        let found = self.find(mac, timeout).await;

        // Stop discovery before connecting, always — including on the error path, so a failed
        // search does not leave the adapter scanning.
        if let Err(e) = adapter.stop_discovery().await {
            tracing::warn!(error = %e, "could not stop discovery");
        }

        let found = found?;
        let device = match found.matches.as_slice() {
            [] => {
                // Name what the radio *did* see, unpaired devices first. Without this the answer is
                // "no gamepad found" and the only escape — naming an address — needs an address
                // nobody has. A pad that advertises no name and no class is invisible to the
                // heuristic and perfectly pairable by address, so this list is the difference
                // between a dead end and one more command.
                let mut others: Vec<&Snapshot> =
                    found.seen.iter().filter(|d| !d.is_gamepad()).collect();
                others.sort_by(|a, b| a.paired.cmp(&b.paired).then(a.mac.cmp(&b.mac)));
                let detail = if others.is_empty() {
                    "nothing at all turned up, not even a device this does not recognise. The pad \
                     is probably not in pairing mode: its light has to be flashing quickly."
                        .to_owned()
                } else {
                    let listed: Vec<String> = others
                        .iter()
                        .take(8)
                        .map(|d| {
                            let name = if d.name.is_empty() {
                                "(no name yet)"
                            } else {
                                &d.name
                            };
                            format!("{} {name}", d.mac)
                        })
                        .collect();
                    format!(
                        "nothing that looks like a gamepad turned up. These were in range — if one \
                         of them is the pad, pair it by address:\n  {}",
                        listed.join("\n  ")
                    )
                };
                tracing::warn!(seen = found.seen.len(), "no gamepad found");
                return Ok(proto::PadPairResult::Failed {
                    reason: proto::PadPairFailure::NotFound,
                    detail: Some(detail),
                });
            }
            [only] => only.clone(),
            several => {
                // Prefer the pads that are **not** already bonded: those are the ones someone just
                // put into pairing mode, and a pad this robot already has is not a competing answer.
                // Without this, adding a second pad in a room where the first is in range would be
                // refused as ambiguous forever.
                let fresh: Vec<&Snapshot> = several.iter().filter(|d| !d.paired).collect();
                match fresh.as_slice() {
                    [only] => (*only).clone(),
                    // Nothing new: report the bonded one, and let `bond` re-assert `Trusted`. This is
                    // the idempotent re-run, and the one case where several bonded pads are in range
                    // — either is a correct answer, so take the first by address for determinism.
                    [] => several
                        .iter()
                        .min_by(|a, b| a.mac.cmp(&b.mac))
                        .expect("several is non-empty")
                        .clone(),
                    _ => {
                        let names: Vec<String> = fresh
                            .iter()
                            .map(|d| format!("{} ({})", d.name, d.mac))
                            .collect();
                        return Ok(proto::PadPairResult::Failed {
                            reason: proto::PadPairFailure::Ambiguous,
                            detail: Some(format!(
                                "more than one pad is in pairing mode: {}",
                                names.join(", ")
                            )),
                        });
                    }
                }
            }
        };

        // The agent, alive only for this bond and scoped to this device. Registered *after* the
        // device is known, which is what makes scoping possible at all.
        let agent_path = ObjectPath::try_from(AGENT_PATH).map_err(|e| e.to_string())?;
        self.bus
            .object_server()
            .at(
                &agent_path,
                PairingAgent {
                    device: device.path.clone(),
                },
            )
            .await
            .map_err(|e| format!("cannot serve a pairing agent: {e}"))?;

        let manager = AgentManagerProxy::new(&self.bus)
            .await
            .map_err(|e| e.to_string())?;
        // Not `RequestDefaultAgent`: `btd` holds the default agent for the phone path, and
        // bluetoothd prefers the agent belonging to whoever called `Pair()` anyway.
        let registered = manager.register_agent(&agent_path, AGENT_CAPABILITY).await;
        if let Err(e) = &registered {
            // Not fatal. A pad is just-works, so bluetoothd may never need to ask anyone — and if
            // it does, `btd`'s default agent is still there to answer.
            tracing::warn!(error = %e, "could not register a pairing agent; relying on the default");
        }

        let outcome = self.bond(&device).await;

        if registered.is_ok()
            && let Err(e) = manager.unregister_agent(&agent_path).await
        {
            tracing::warn!(error = %e, "could not unregister the pairing agent");
        }
        if let Err(e) = self
            .bus
            .object_server()
            .remove::<PairingAgent, _>(&agent_path)
            .await
        {
            tracing::warn!(error = %e, "could not withdraw the pairing agent");
        }

        if let Err((reason, detail)) = outcome {
            tracing::warn!(mac = %device.mac, ?reason, %detail, "pairing failed");
            return Ok(proto::PadPairResult::Failed {
                reason,
                detail: Some(detail),
            });
        }

        // Re-read rather than assume: what BlueZ ended up with is what the caller should be told,
        // including a pad that bonded but has not connected yet.
        let pad = self
            .devices()
            .await?
            .into_iter()
            .find(|d| d.mac.eq_ignore_ascii_case(&device.mac))
            .map(|d| d.as_pad())
            .unwrap_or_else(|| proto::Pad {
                paired: true,
                trusted: true,
                ..device.as_pad()
            });
        tracing::warn!(mac = %pad.mac, name = %pad.name, "gamepad paired and trusted");
        Ok(proto::PadPairResult::Paired { pad })
    }

    async fn forget(&self, mac: &str) -> PadResult<proto::PadForgetResult> {
        let Some(adapter) = self.adapter().await? else {
            // No adapter, so nothing is bonded to it as far as anyone can tell. `removed: false` is
            // the honest answer and matches what forgetting an unknown pad returns.
            return Ok(proto::PadForgetResult { removed: false });
        };

        let Some(device) = self
            .devices()
            .await?
            .into_iter()
            .find(|d| d.mac.eq_ignore_ascii_case(mac))
        else {
            return Ok(proto::PadForgetResult { removed: false });
        };

        adapter
            .remove_device(&device.path.as_ref())
            .await
            .map_err(|e| format!("bluetoothd would not remove {mac}: {e}"))?;
        tracing::info!(mac, "pad forgotten");
        Ok(proto::PadForgetResult { removed: true })
    }
}
