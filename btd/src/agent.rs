//! BlueZ's pairing agent for phones.
//!
//! `bluer` infers `DisplayYesNo` as soon as a confirmation handler is installed.
//! This board has neither input nor display, so we register directly with BlueZ
//! and keep its declared capability `NoInputNoOutput`. BlueZ can still ask for
//! confirmation when a phone requests MITM pairing. Answering that request does
//! **not** compare numbers and does **not** authenticate the link. The application
//! PIN gate remains mandatory before any sensitive RPC is served.

use zbus::zvariant::ObjectPath;

const AGENT_PATH: &str = "/com/pollenrobotics/btd/phone_agent";
const CAPABILITY: &str = "NoInputNoOutput";

#[zbus::proxy(
    interface = "org.bluez.AgentManager1",
    default_service = "org.bluez",
    default_path = "/org/bluez"
)]
trait AgentManager {
    fn register_agent(&self, agent: &ObjectPath<'_>, capability: &str) -> zbus::Result<()>;
    fn request_default_agent(&self, agent: &ObjectPath<'_>) -> zbus::Result<()>;
}

struct Agent;

#[zbus::interface(name = "org.bluez.Agent1")]
impl Agent {
    fn release(&self) {
        tracing::debug!("phone pairing agent released");
    }

    fn cancel(&self) {
        tracing::debug!("phone pairing cancelled");
    }

    fn request_confirmation(&self, device: ObjectPath<'_>, _passkey: u32) {
        // BlueZ can request this despite NoInputNoOutput (seen with Android).
        // This is just-works consent, not numeric comparison or MITM protection.
        tracing::info!(
            device = device.as_str(),
            "accepting unauthenticated phone pairing"
        );
    }

    fn request_authorization(&self, device: ObjectPath<'_>) {
        tracing::info!(
            device = device.as_str(),
            "authorizing phone bond before PIN check"
        );
    }

    fn authorize_service(&self, device: ObjectPath<'_>, uuid: String) {
        tracing::debug!(device = device.as_str(), uuid, "authorizing phone service");
    }

    fn request_pin_code(&self, _device: ObjectPath<'_>) -> zbus::fdo::Result<String> {
        Err(zbus::fdo::Error::NotSupported("no keyboard".into()))
    }

    fn request_passkey(&self, _device: ObjectPath<'_>) -> zbus::fdo::Result<u32> {
        Err(zbus::fdo::Error::NotSupported("no keyboard".into()))
    }
}

/// A dedicated bus connection keeps the agent registered for the service lifetime.
/// Dropping it withdraws the agent when BlueZ or the adapter restarts.
pub struct PhoneAgent {
    _bus: zbus::Connection,
}

impl PhoneAgent {
    pub async fn register() -> zbus::Result<Self> {
        let bus = zbus::Connection::system().await?;
        let path = ObjectPath::try_from(AGENT_PATH).expect("static agent path");
        bus.object_server().at(&path, Agent).await?;
        let manager = AgentManagerProxy::new(&bus).await?;
        manager.register_agent(&path, CAPABILITY).await?;
        manager.request_default_agent(&path).await?;
        Ok(Self { _bus: bus })
    }
}
