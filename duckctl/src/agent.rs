//! A BlueZ pairing agent for the length of one command, on Linux only.
//!
//! The robot pairs just-works (`btd/src/pairing.rs`), and BlueZ on the laptop still asks an agent
//! to consent to it. A desktop registers one only while its Bluetooth settings are open, so
//! otherwise nothing answers: `bluetoothd` logs `No agent available for request type 2`, refuses
//! the bond the robot has just accepted, and the encrypted read that started it times out. That was
//! the first thing anyone met running `duckctl` on Linux against a robot it had never paired with.
//!
//! So `duckctl` registers its own, `NoInputNoOutput` like the robot's, and asks to be the default
//! for as long as it runs: pairing triggered by an ATT error belongs to no D-Bus caller, so BlueZ
//! asks the default agent. Leaving releases it — the bus connection closes with the process, and
//! BlueZ hands the default back to whichever agent held it before.
//!
//! **It consents for the robot being connected to and nobody else.** A just-works agent that
//! accepted every request would bond this laptop to anything in range that asked while `duckctl`
//! ran. Every other device is rejected.
//!
//! macOS and Windows need none of this: the platform consents to just-works pairing itself.

use std::time::Duration;

use dbus::MethodErr;
use dbus::channel::MatchingReceiver as _;
use dbus::message::MatchRule;
use dbus::nonblock::{Proxy, SyncConnection};
use dbus_crossroads::Crossroads;
use std::sync::Arc;

const PATH: &str = "/com/pollenrobotics/duckctl/agent";
const CAPABILITY: &str = "NoInputNoOutput";
const CALL_TIMEOUT: Duration = Duration::from_secs(5);

/// The registration, held for as long as the connection to the robot needs consenting to.
pub struct Agent {
    _connection: Arc<SyncConnection>,
}

/// The tail of a BlueZ device path for `address`: `AA:BB:…` becomes `dev_AA_BB_…`.
fn device_suffix(address: &str) -> String {
    format!("dev_{}", address.to_ascii_uppercase().replace(':', "_"))
}

/// Whether the device BlueZ asks about is the robot. `/org/bluez/hci0/dev_8C_CD_…`.
fn is_robot(device: &str, suffix: &str) -> bool {
    device.rsplit('/').next() == Some(suffix)
}

/// Register an agent that consents to pairing with `address` alone, and make it the default.
pub async fn register(address: &str) -> Result<Agent, String> {
    let (resource, connection) =
        dbus_tokio::connection::new_system_sync().map_err(|e| e.to_string())?;
    tokio::spawn(async move {
        // Ends only when the bus connection does, which is the process ending.
        let _ = resource.await;
    });

    let suffix = device_suffix(address);
    let mut crossroads = Crossroads::new();
    let interface = crossroads.register("org.bluez.Agent1", move |b| {
        let consent = {
            let suffix = suffix.clone();
            move |device: &dbus::Path<'static>| -> Result<(), MethodErr> {
                if is_robot(device, &suffix) {
                    Ok(())
                } else {
                    Err(MethodErr::from((
                        "org.bluez.Error.Rejected",
                        "duckctl only consents to the robot it is connecting to",
                    )))
                }
            }
        };
        let confirm = consent.clone();
        b.method(
            "RequestConfirmation",
            ("device", "passkey"),
            (),
            move |_, _, (device, _passkey): (dbus::Path<'static>, u32)| confirm(&device),
        );
        let authorize = consent.clone();
        b.method(
            "RequestAuthorization",
            ("device",),
            (),
            move |_, _, (device,): (dbus::Path<'static>,)| authorize(&device),
        );
        let service = consent;
        b.method(
            "AuthorizeService",
            ("device", "uuid"),
            (),
            move |_, _, (device, _uuid): (dbus::Path<'static>, String)| service(&device),
        );
        // No keyboard and no display, as declared: a request for either is a pairing method this
        // agent did not offer, and refusing it is the honest answer.
        b.method(
            "RequestPinCode",
            ("device",),
            ("pincode",),
            |_, _, (_,): (dbus::Path<'static>,)| -> Result<(String,), MethodErr> {
                Err(MethodErr::from(("org.bluez.Error.Rejected", "no keyboard")))
            },
        );
        b.method(
            "RequestPasskey",
            ("device",),
            ("passkey",),
            |_, _, (_,): (dbus::Path<'static>,)| -> Result<(u32,), MethodErr> {
                Err(MethodErr::from(("org.bluez.Error.Rejected", "no keyboard")))
            },
        );
        b.method(
            "DisplayPinCode",
            ("device", "pincode"),
            (),
            |_, _, (_, _): (dbus::Path<'static>, String)| -> Result<(), MethodErr> {
                Err(MethodErr::from(("org.bluez.Error.Rejected", "no display")))
            },
        );
        b.method(
            "DisplayPasskey",
            ("device", "passkey", "entered"),
            (),
            |_, _, (_, _, _): (dbus::Path<'static>, u32, u16)| Ok(()),
        );
        b.method("Release", (), (), |_, _, ()| Ok(()));
        b.method("Cancel", (), (), |_, _, ()| Ok(()));
    });
    crossroads.insert(PATH, &[interface], ());
    connection.start_receive(
        MatchRule::new_method_call(),
        Box::new(move |message, connection| {
            let _ = crossroads.handle_message(message, connection);
            true
        }),
    );

    let manager = Proxy::new("org.bluez", "/org/bluez", CALL_TIMEOUT, connection.clone());
    let path = dbus::Path::from(PATH);
    manager
        .method_call::<(), _, _, _>(
            "org.bluez.AgentManager1",
            "RegisterAgent",
            (path.clone(), CAPABILITY),
        )
        .await
        .map_err(|e| e.to_string())?;
    manager
        .method_call::<(), _, _, _>("org.bluez.AgentManager1", "RequestDefaultAgent", (path,))
        .await
        .map_err(|e| e.to_string())?;

    Ok(Agent {
        _connection: connection,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn consents_to_the_robot_only() {
        let suffix = device_suffix("8c:cd:fe:3c:7c:8c");
        assert!(is_robot("/org/bluez/hci0/dev_8C_CD_FE_3C_7C_8C", &suffix));
        assert!(!is_robot("/org/bluez/hci0/dev_B0_9C_B2_C4_EE_84", &suffix));
        // A prefix of the robot's address is another device.
        assert!(!is_robot("/org/bluez/hci0/dev_8C_CD_FE_3C_7C_8C0", &suffix));
    }
}
