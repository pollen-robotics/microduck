//! The network LED: green on wifi, blue once the robot is reachable from outside its network.
//! Which LED it is, and why this daemon owns it, is `architecture.md` §3.2.
//!
//! Blue is a fact `mediad` holds — its relay is what registers with the rendezvous service — and
//! it publishes it at [`proto::remote_status_path`] for readers like this one. The LED stays one
//! writer's all the same: green and blue on two daemons would show both at once, and a robot on
//! wifi *and* reachable from outside should say the second, which is the one that took more.

use std::sync::Arc;
use std::time::Duration;

use duck_ipc_proto as proto;
use duck_ipc_proto::led::{Led, Light, name};

use crate::net::Net;

/// How often the LED catches up. NetworkManager is asked over D-Bus, which is cheap at this rate.
const PERIOD: Duration = Duration::from_secs(2);

/// A registration whose last heartbeat is older than this is a relay that has stopped, whatever
/// the file says: the heartbeat runs at most a minute apart (`relay.rs`).
const STALE_HEARTBEAT_S: i64 = 90;

/// Green, then blue.
pub fn lights(wifi: proto::NetState, remote: Option<&proto::RemoteStatus>, now: i64) -> [Light; 2] {
    let reachable = matches!(
        remote.map(|r| &r.link),
        Some(proto::RemoteLink::Registered { last_heartbeat, .. })
            if now - last_heartbeat <= STALE_HEARTBEAT_S
    );
    if reachable {
        return [Light::Off, Light::On];
    }
    match wifi {
        proto::NetState::Connected => [Light::On, Light::Off],
        proto::NetState::Connecting => [Light::Blink, Light::Off],
        proto::NetState::Disconnected | proto::NetState::Unavailable => [Light::Off, Light::Off],
    }
}

/// Keep the network LED in step for as long as the daemon runs. Returns at once on a board
/// without one.
pub async fn run(net: Arc<dyn Net>) {
    let (Some(mut green), Some(mut blue)) = (
        Led::open(name::NETWORK_GREEN),
        Led::open(name::NETWORK_BLUE),
    ) else {
        tracing::debug!("no network LED on this board");
        return;
    };
    let mut ticker = tokio::time::interval(PERIOD);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        ticker.tick().await;
        let wifi = net
            .status()
            .await
            .map_or(proto::NetState::Unavailable, |s| s.state);
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs() as i64);
        let [g, b] = lights(wifi, proto::read_remote_status().as_ref(), now);
        // Off before on, so a change of colour never shows the mix.
        let mut order = [(&mut green, g), (&mut blue, b)];
        order.sort_by_key(|(_, light)| *light != Light::Off);
        for (led, light) in order {
            if let Err(e) = led.set(light) {
                tracing::warn!(error = %e, "cannot switch the network LED");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn registered(last_heartbeat: i64) -> proto::RemoteStatus {
        proto::RemoteStatus {
            service: "https://example".into(),
            since: 0,
            link: proto::RemoteLink::Registered {
                account: None,
                peer_id: "p".into(),
                last_heartbeat,
            },
        }
    }

    #[test]
    fn green_on_wifi_blue_when_reachable_from_outside() {
        use proto::NetState::*;
        assert_eq!(lights(Disconnected, None, 0), [Light::Off, Light::Off]);
        assert_eq!(lights(Connecting, None, 0), [Light::Blink, Light::Off]);
        assert_eq!(lights(Connected, None, 0), [Light::On, Light::Off]);
        assert_eq!(
            lights(Connected, Some(&registered(100)), 110),
            [Light::Off, Light::On]
        );
    }

    #[test]
    fn a_relay_that_stopped_heartbeating_is_not_reachable() {
        let stale = registered(100);
        assert_eq!(
            lights(
                proto::NetState::Connected,
                Some(&stale),
                100 + STALE_HEARTBEAT_S + 1
            ),
            [Light::On, Light::Off]
        );
        let signed_out = proto::RemoteStatus {
            link: proto::RemoteLink::SignedOut,
            ..stale
        };
        assert_eq!(
            lights(proto::NetState::Connected, Some(&signed_out), 100),
            [Light::On, Light::Off]
        );
    }
}
