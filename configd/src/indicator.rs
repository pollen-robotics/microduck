//! The network LED: green on wifi, blue when a Bluetooth gamepad is connected and wifi is not.
//! Which LED it is, and why this daemon owns it, is `architecture.md` §3.2.
//!
//! Both facts are this daemon's own — NetworkManager for wifi, BlueZ for the pad — so the LED
//! reads nothing another daemon publishes. Wifi wins when both hold: one LED shows one thing, and
//! a robot on wifi is the more useful thing to know from across the room. Blue is the robot that
//! is off any network and still being driven by the pad in someone's hands.
//!
//! Above both: **blinking blue while ssh is open on wifi** ([`crate::wifi_ssh`]). It is the answer
//! to a press of the face button, and a robot with a root-capable login on the network should say
//! so from across the room.

use std::sync::Arc;
use std::time::Duration;

use duck_ipc_proto as proto;
use duck_ipc_proto::led::{Led, Light, name};

use crate::net::Net;
use crate::pad::Pads;

/// How often the LED catches up. NetworkManager and BlueZ are asked over D-Bus, which is cheap at
/// this rate.
const PERIOD: Duration = Duration::from_secs(2);

/// Green, then blue. `pad` is a bonded gamepad connected right now; `ssh` is ssh open on wifi.
pub fn lights(wifi: proto::NetState, pad: bool, ssh: bool) -> [Light; 2] {
    if ssh {
        return [Light::Off, Light::Blink];
    }
    match wifi {
        proto::NetState::Connected => [Light::On, Light::Off],
        proto::NetState::Connecting => [Light::Blink, Light::Off],
        proto::NetState::Disconnected | proto::NetState::Unavailable if pad => {
            [Light::Off, Light::On]
        }
        proto::NetState::Disconnected | proto::NetState::Unavailable => [Light::Off, Light::Off],
    }
}

/// Keep the network LED in step for as long as the daemon runs. Returns at once on a board
/// without one.
pub async fn run(net: Arc<dyn Net>, pads: Arc<dyn Pads>) {
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
        // A BlueZ that cannot answer is no pad, as an unanswering NetworkManager is no wifi.
        let pad = pads
            .status()
            .await
            .is_ok_and(|pads| pads.iter().any(|p| p.connected));
        let ssh = crate::wifi_ssh::state(std::path::Path::new(crate::wifi_ssh::SWITCH))
            != crate::wifi_ssh::State::Closed;
        let [g, b] = lights(wifi, pad, ssh);
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
    use proto::NetState::*;

    #[test]
    fn green_on_wifi_whether_or_not_a_pad_is_connected() {
        assert_eq!(lights(Connected, false, false), [Light::On, Light::Off]);
        assert_eq!(lights(Connected, true, false), [Light::On, Light::Off]);
        assert_eq!(lights(Connecting, false, false), [Light::Blink, Light::Off]);
        assert_eq!(lights(Connecting, true, false), [Light::Blink, Light::Off]);
    }

    #[test]
    fn blue_for_a_pad_without_wifi_and_off_with_neither() {
        assert_eq!(lights(Disconnected, true, false), [Light::Off, Light::On]);
        assert_eq!(lights(Unavailable, true, false), [Light::Off, Light::On]);
        assert_eq!(lights(Disconnected, false, false), [Light::Off, Light::Off]);
        assert_eq!(lights(Unavailable, false, false), [Light::Off, Light::Off]);
    }

    #[test]
    fn blinking_blue_while_ssh_is_open_on_wifi_whatever_else_holds() {
        for wifi in [Connected, Connecting, Disconnected, Unavailable] {
            for pad in [false, true] {
                assert_eq!(lights(wifi, pad, true), [Light::Off, Light::Blink]);
            }
        }
    }
}
