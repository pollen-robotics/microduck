//! Is `padd` running?
//!
//! Reported alongside the pads because a connected pad and a dead `padd` is the failure that looks
//! like working hardware: the light on the controller is on, the robot ignores it, and nothing in
//! either place says why. Answering it here means `robotctl pad status` covers the whole path in one
//! command instead of "and now also check systemd".
//!
//! Asked of systemd rather than tracked: `padd` is started, stopped and restarted by systemd, so
//! systemd is the only thing that knows. `configd` deliberately holds no opinion — it does not start
//! `padd`, does not restart it, and reporting is the whole of its involvement.

use duck_ipc_proto as proto;

/// The unit that turns a pad into intents.
pub const UNIT: &str = "padd.service";

/// What systemd says about `padd.service`.
#[cfg(target_os = "linux")]
pub async fn state() -> proto::DriverState {
    match query().await {
        Ok(state) => state,
        Err(e) => {
            // A warning, not an error: this is one line of a status report, and failing to read it
            // must not fail the report.
            tracing::warn!(error = %e, unit = UNIT, "could not ask systemd about the pad driver");
            proto::DriverState::Unknown
        }
    }
}

#[cfg(target_os = "linux")]
async fn query() -> Result<proto::DriverState, String> {
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
            &(UNIT),
        )
        .await
    {
        Ok(reply) => reply.body().deserialize().map_err(|e| e.to_string())?,
        Err(e) => {
            // No such unit: a board on a release older than the one that added `padd.service`.
            // That is a fact about the install, not a failure to report as one.
            tracing::debug!(error = %e, unit = UNIT, "no such unit");
            return Ok(proto::DriverState::Absent);
        }
    };

    let active: String = bus
        .call_method(
            Some("org.freedesktop.systemd1"),
            &path,
            Some("org.freedesktop.DBus.Properties"),
            "Get",
            &("org.freedesktop.systemd1.Unit", "ActiveState"),
        )
        .await
        .map_err(|e| e.to_string())?
        .body()
        .deserialize::<zbus::zvariant::Value>()
        .map_err(|e| e.to_string())
        .and_then(|value| String::try_from(value).map_err(|e| e.to_string()))?;

    Ok(match active.as_str() {
        // `activating` counts as active: `padd` spends its first moments connecting to `robotd`, and
        // reporting that as "not running" would make a robot mid-boot look broken.
        "active" | "activating" | "reloading" => proto::DriverState::Active,
        // `failed` is inactive with a reason, and the reason is in the journal rather than here.
        // Collapsing them keeps this a status line rather than a diagnosis.
        "inactive" | "deactivating" | "failed" => proto::DriverState::Inactive,
        other => {
            tracing::warn!(state = other, unit = UNIT, "unfamiliar unit state");
            proto::DriverState::Unknown
        }
    })
}

/// Off the board there is no systemd to ask, and inventing an answer would make a laptop look like
/// a robot with a broken pad driver.
#[cfg(not(target_os = "linux"))]
pub async fn state() -> proto::DriverState {
    proto::DriverState::Unknown
}
