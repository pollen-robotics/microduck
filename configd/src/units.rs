//! Which of the robot's daemons are running, and which release each is running.
//!
//! Started as "is `padd` running?", reported alongside the pads because a connected pad and a dead
//! `padd` is the failure that looks like working hardware: the light on the controller is on, the
//! robot ignores it, and nothing in either place says why. That argument was never specific to
//! `padd` — a dead `btd` is a robot no phone can see, with the same silence — so it answers for
//! every unit a release manages.
//!
//! Asked of systemd rather than tracked: these are started, stopped and restarted by systemd, so
//! systemd is the only thing that knows. `configd` deliberately holds no opinion — it does not start
//! them, does not restart them, and reporting is the whole of its involvement.
//!
//! ## Which release is running
//!
//! Not asked of systemd and not inferred from `/proc`: **each daemon publishes its own identity at
//! startup**, to `/run/<service>/identity.json`, and this reads it. See
//! [`duck_ipc_proto::Identity`] for why that beats inspecting a process from outside — briefly, a
//! process knows its version, its git revision and its own exe, and needs no privilege to say so.
//!
//! What is left for systemd is the question only systemd can answer: whether the unit is running.
//! The two are read together because they mean different things apart. A published identity with a
//! stopped unit cannot happen — systemd deletes the runtime directory with the unit — but a *stopped
//! unit with no identity* and a *running daemon too old to publish one* both report nothing, and the
//! unit state is what distinguishes them.

use duck_ipc_proto as proto;

/// The unit that turns a pad into intents.
pub const PADD: &str = "padd.service";

/// Every unit a daemon release manages, in the order a reader wants them: the update engine, then
/// the robot, then the ones that depend on both.
///
/// Hardcoded rather than discovered, and that is a real limitation worth naming: a unit added to a
/// release and not to this list is invisible here. The alternative — asking systemd for everything
/// and filtering — reports units this project does not own, which is worse for a status line.
/// `scripts/install.sh` knows exactly these.
///
/// It has already cost once: `mediad` and `tofd` shipped units two releases before they were named
/// here, so the block a person reads after an update — the one that exists to say which daemon is
/// still on the old release — could not report either of them at all.
pub const MANAGED: [&str; 7] = [
    "updaterd.service",
    "robotd.service",
    "configd.service",
    "btd.service",
    "padd.service",
    "mediad.service",
    "tofd.service",
];

/// What systemd says about one unit. The narrow question, kept for `pad.status`.
pub async fn state(unit: &str) -> proto::UnitState {
    describe(unit).await.state
}

pub async fn all() -> Vec<proto::ServiceUnit> {
    let mut units = Vec::with_capacity(MANAGED.len());
    for unit in MANAGED {
        units.push(describe(unit).await);
    }
    units
}

#[cfg(target_os = "linux")]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    let state = match query(unit).await {
        Ok(state) => state,
        Err(e) => {
            // A warning, not an error: this is one line of a status report, and failing to read it
            // must not fail the report.
            tracing::warn!(error = %e, unit, "could not ask systemd about a unit");
            proto::UnitState::Unknown
        }
    };

    proto::ServiceUnit {
        identity: proto::read_identity(service_of(unit)),
        unit: unit.to_owned(),
        state,
    }
}

/// Off the board there is no systemd to ask, and inventing an answer would make a laptop look like a
/// robot with a broken daemon. The identity file is still read: it is an ordinary file, and a daemon
/// run by hand on a laptop publishes one.
#[cfg(not(target_os = "linux"))]
pub async fn describe(unit: &str) -> proto::ServiceUnit {
    proto::ServiceUnit {
        identity: proto::read_identity(service_of(unit)),
        unit: unit.to_owned(),
        state: proto::UnitState::Unknown,
    }
}

/// `btd.service` names the service `btd`, which is what it publishes under.
fn service_of(unit: &str) -> &str {
    unit.strip_suffix(".service").unwrap_or(unit)
}

#[cfg(target_os = "linux")]
async fn query(unit: &str) -> Result<proto::UnitState, String> {
    let bus = zbus::Connection::system()
        .await
        .map_err(|e| e.to_string())?;

    // `LoadUnit` rather than `GetUnit`: `GetUnit` fails for a unit systemd has not loaded, which is
    // indistinguishable from a unit that does not exist — and those are different answers here.
    // `LoadUnit` loads it if the file is there and fails only when it genuinely is not.
    let path: zbus::zvariant::OwnedObjectPath = match bus
        .call_method(
            Some("org.freedesktop.systemd1"),
            "/org/freedesktop/systemd1",
            Some("org.freedesktop.systemd1.Manager"),
            "LoadUnit",
            &(unit),
        )
        .await
    {
        Ok(reply) => reply.body().deserialize().map_err(|e| e.to_string())?,
        Err(e) => {
            // No such unit: a board on a release older than the one that added it. That is a fact
            // about the install, not a failure to report as one.
            tracing::debug!(error = %e, unit, "no such unit");
            return Ok(proto::UnitState::Absent);
        }
    };

    let active: String = property(&bus, &path, "org.freedesktop.systemd1.Unit", "ActiveState")
        .await?
        .try_into()
        .map_err(|e: zbus::zvariant::Error| e.to_string())?;

    // A second property on the connection and object path already in hand, so it costs one more
    // round trip on the same bus. It buys the only distinction systemd makes between a daemon
    // coming up and a daemon that keeps dying: both are `ActiveState=activating`.
    let sub: String = property(&bus, &path, "org.freedesktop.systemd1.Unit", "SubState")
        .await?
        .try_into()
        .map_err(|e: zbus::zvariant::Error| e.to_string())?;

    Ok(state_of(&active, &sub, unit))
}

/// What one `ActiveState`/`SubState` pair means.
///
/// Pure, and separated from the bus call for the reason `reconcile::verdict_for` is: the mapping is
/// the part that can be wrong, and the states worth checking are the ones a test cannot arrange —
/// a crash loop needs a daemon that will not start, and asking systemd for one on a board means
/// breaking the robot.
///
/// **`SubState` is read first**, because `auto-restart` is the answer `ActiveState` cannot give.
/// systemd reports a unit waiting out its `RestartSec=` as `activating`, exactly as it reports one
/// starting for the first time; taking `activating` for `Active` is what let `mediad` crash-loop on
/// an unplugged camera flex while `robotctl health` called it active. `auto-restart-queued` is the
/// same state one systemd version later, hence the prefix rather than an equality.
pub fn state_of(active: &str, sub: &str, unit: &str) -> proto::UnitState {
    if sub.starts_with("auto-restart") {
        return proto::UnitState::Restarting;
    }

    match active {
        // `activating` still counts as active: `padd` spends its first moments connecting to
        // `robotd`, and reporting that as "not running" would make a robot mid-boot look broken.
        // With `auto-restart` taken out above, that is all this arm now catches.
        "active" | "activating" | "reloading" => proto::UnitState::Active,
        // Reported apart from `inactive`, because a daemon that could not start and a daemon
        // somebody stopped are different news even though neither is running.
        "failed" => proto::UnitState::Failed,
        "inactive" | "deactivating" => proto::UnitState::Inactive,
        other => {
            tracing::warn!(state = other, unit, "unfamiliar unit state");
            proto::UnitState::Unknown
        }
    }
}

#[cfg(target_os = "linux")]
async fn property(
    bus: &zbus::Connection,
    path: &zbus::zvariant::OwnedObjectPath,
    interface: &str,
    name: &str,
) -> Result<zbus::zvariant::Value<'static>, String> {
    bus.call_method(
        Some("org.freedesktop.systemd1"),
        path,
        Some("org.freedesktop.DBus.Properties"),
        "Get",
        &(interface, name),
    )
    .await
    .map_err(|e| e.to_string())?
    .body()
    .deserialize::<zbus::zvariant::Value>()
    .map(|value| value.try_to_owned().map(Into::into))
    .map_err(|e| e.to_string())?
    .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pair a crash loop presents as, and the whole reason [`state_of`] reads `SubState`.
    #[test]
    fn auto_restart_is_not_active() {
        assert_eq!(
            state_of("activating", "auto-restart", "mediad.service"),
            proto::UnitState::Restarting
        );
        // The same state on a systemd new enough to queue the job separately.
        assert_eq!(
            state_of("activating", "auto-restart-queued", "mediad.service"),
            proto::UnitState::Restarting
        );
    }

    /// A daemon coming up for the first time is `activating` too, and must keep reading as running
    /// — `padd` is `activating` while it connects to `robotd` on every boot.
    #[test]
    fn starting_still_counts_as_active() {
        assert_eq!(
            state_of("activating", "start", "padd.service"),
            proto::UnitState::Active
        );
        assert_eq!(
            state_of("active", "running", "padd.service"),
            proto::UnitState::Active
        );
    }

    /// `failed` and a deliberate stop both mean "not running" and are not the same news.
    #[test]
    fn failed_is_not_a_stop() {
        assert_eq!(
            state_of("failed", "failed", "mediad.service"),
            proto::UnitState::Failed
        );
        assert_eq!(
            state_of("inactive", "dead", "padd.service"),
            proto::UnitState::Inactive
        );
    }

    /// An unfamiliar `ActiveState` reports "I do not know" rather than guessing at one of the
    /// answers that would be acted on.
    #[test]
    fn an_unknown_state_stays_unknown() {
        assert_eq!(
            state_of("maintenance", "whatever", "robotd.service"),
            proto::UnitState::Unknown
        );
    }
}
