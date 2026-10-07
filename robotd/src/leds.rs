//! The face LEDs this daemon owns: the status LED, the battery gauge and the flashlight.
//!
//! What each one means is `architecture.md` §3.2, which also says why these three are `robotd`'s:
//! it is the daemon that knows its own health, reads the battery, and answers `robot.flashlight`.
//! [`duck_ipc_proto::led`] is how an LED is switched.
//!
//! A task of its own, never the control loop: the LEDs hang off an I²C expander, and a write is a
//! bus transaction the 50 Hz tick has no business waiting on. Once a second is faster than health
//! or a battery changes. The flashlight does not wait for the second — a press wakes the task.

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use duck_ipc_proto as proto;
use duck_ipc_proto::led::{Led, Light, name};

use crate::RobotState;

/// How often the status and the battery are re-read.
const PERIOD: Duration = Duration::from_secs(1);

/// The battery gauge, in the order it fills.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Gauge {
    /// No reading yet: nothing lit, rather than a guess.
    Unknown,
    /// The low LED blinking red.
    Critical,
    /// The low LED red.
    Low,
    /// One green, two greens, three greens.
    One,
    Two,
    Three,
}

/// The percentages a gauge level starts above, from [`Gauge::Low`] up. Critical is what is left.
const GAUGE_ABOVE: [f64; 4] = [5.0, 15.0, 40.0, 70.0];

/// How far past a boundary the battery has to move before the gauge follows it. The reading is
/// already a ten-second average; this is for the walk that sags it and the rest that lifts it
/// back, which would otherwise flip the top LED every few seconds right at a boundary.
const GAUGE_HYSTERESIS: f64 = 3.0;

impl Gauge {
    fn of(percent: f64) -> Gauge {
        match GAUGE_ABOVE.iter().filter(|&&above| percent > above).count() {
            0 => Gauge::Critical,
            1 => Gauge::Low,
            2 => Gauge::One,
            3 => Gauge::Two,
            _ => Gauge::Three,
        }
    }

    /// The gauge for `percent`, given what it showed before.
    pub fn next(self, percent: Option<f64>) -> Gauge {
        let Some(percent) = percent else {
            return Gauge::Unknown;
        };
        let (low, high) = (
            Gauge::of(percent - GAUGE_HYSTERESIS),
            Gauge::of(percent + GAUGE_HYSTERESIS),
        );
        // Within the band either side of the reading, the previous level stands.
        if (low..=high).contains(&self) {
            self
        } else {
            Gauge::of(percent)
        }
    }

    /// The low LED's green and red, then the middle and the high green.
    fn lights(self) -> [Light; 4] {
        use Light::{Blink, Off, On};
        match self {
            Gauge::Unknown => [Off, Off, Off, Off],
            Gauge::Critical => [Off, Blink, Off, Off],
            Gauge::Low => [Off, On, Off, Off],
            Gauge::One => [On, Off, Off, Off],
            Gauge::Two => [On, Off, On, Off],
            Gauge::Three => [On, Off, On, On],
        }
    }
}

/// What the status LED says.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// Blinking green: the control loop has not ticked yet, and nothing says it will not.
    Starting,
    /// Green: `robot.health` answers healthy.
    Healthy,
    /// Blinking red: unhealthy for a reason the board owns rather than the release — no servo
    /// power, a servo unplugged, a policy override that would not load (`HealthResult::degraded`).
    Degraded,
    /// Red: unhealthy.
    Fault,
}

impl Status {
    /// `starting` is "no tick yet and no failed attempt at the bus", which `health` reports as
    /// unhealthy because the update gate must not commit on it — but it is not a fault to a person
    /// looking at the robot for the first seconds after boot.
    pub fn of(health: &proto::HealthResult, starting: bool) -> Status {
        if health.healthy {
            Status::Healthy
        } else if starting {
            Status::Starting
        } else if health.degraded {
            Status::Degraded
        } else {
            Status::Fault
        }
    }

    /// Green, then red.
    fn lights(self) -> [Light; 2] {
        use Light::{Blink, Off, On};
        match self {
            Status::Starting => [Blink, Off],
            Status::Healthy => [On, Off],
            Status::Degraded => [Off, Blink],
            Status::Fault => [Off, On],
        }
    }
}

/// The flashlight as `RobotState` stores it: zero is off, otherwise the colour's position in
/// [`COLORS`] plus one.
pub type Flashlight = u8;

const COLORS: [proto::FlashlightColor; 7] = [
    proto::FlashlightColor::White,
    proto::FlashlightColor::Red,
    proto::FlashlightColor::Green,
    proto::FlashlightColor::Blue,
    proto::FlashlightColor::Yellow,
    proto::FlashlightColor::Cyan,
    proto::FlashlightColor::Magenta,
];

/// The stored flashlight after `p`, from `current`.
pub fn flashlight_after(current: Flashlight, p: &proto::FlashlightParams) -> Flashlight {
    let on = if p.toggle { current == 0 } else { p.on };
    if on {
        COLORS
            .iter()
            .position(|&c| c == p.color)
            .map_or(1, |i| i as u8 + 1)
    } else {
        0
    }
}

fn flashlight_lights(stored: Flashlight) -> [Light; 3] {
    let channels = match stored {
        0 => [false; 3],
        n => COLORS
            .get(usize::from(n) - 1)
            .map_or([false; 3], |c| c.channels()),
    };
    channels.map(Light::from)
}

/// Whether this board has a flashlight. Read once at startup: an LED does not appear on a
/// running board.
pub fn flashlight_fitted() -> bool {
    Led::open(name::FLASHLIGHT_RED).is_some()
}

/// Every LED this daemon owns, each `None` on a board without it.
struct Face {
    status: [Option<Led>; 2],
    battery: [Option<Led>; 4],
    flashlight: [Option<Led>; 3],
}

impl Face {
    fn open() -> Option<Face> {
        let face = Face {
            status: [name::STATUS_GREEN, name::STATUS_RED].map(Led::open),
            battery: [
                name::BATTERY_LOW_GREEN,
                name::BATTERY_LOW_RED,
                name::BATTERY_MID,
                name::BATTERY_HIGH,
            ]
            .map(Led::open),
            flashlight: [
                name::FLASHLIGHT_RED,
                name::FLASHLIGHT_GREEN,
                name::FLASHLIGHT_BLUE,
            ]
            .map(Led::open),
        };
        let any = face
            .status
            .iter()
            .chain(&face.battery)
            .chain(&face.flashlight)
            .any(Option::is_some);
        any.then_some(face)
    }

    fn show(&mut self, status: [Light; 2], battery: [Light; 4], flashlight: [Light; 3]) {
        // Off before on within each LED, so a colour change never shows the mix of both.
        let leds = self
            .status
            .iter_mut()
            .zip(status)
            .chain(self.battery.iter_mut().zip(battery))
            .chain(self.flashlight.iter_mut().zip(flashlight));
        let (offs, ons): (Vec<_>, Vec<_>) = leds.partition(|(_, light)| *light == Light::Off);
        for (led, light) in offs.into_iter().chain(ons) {
            if let Some(led) = led
                && let Err(e) = led.set(light)
            {
                tracing::warn!(error = %e, "cannot switch an LED");
            }
        }
    }
}

/// Drive the LEDs until the daemon shuts down, then switch them off. Returns at once on a board
/// without any of them.
pub async fn run(state: Arc<RobotState>) {
    let Some(mut face) = Face::open() else {
        tracing::debug!("no face LEDs on this board");
        return;
    };
    let mut gauge = Gauge::Unknown;
    loop {
        if state.shutdown.load(Ordering::Relaxed) {
            // A stopped robotd leaves nothing lit that it would have kept up to date: a green
            // status from a daemon that is gone would be a lie.
            face.show([Light::Off; 2], [Light::Off; 4], [Light::Off; 3]);
            return;
        }
        let starting = state.ticks.load(Ordering::Relaxed) == 0
            && state.startup_bus_failures.load(Ordering::Relaxed) == 0
            && !state.force_unhealthy;
        let health = state.health();
        gauge = gauge.next(health.battery.as_ref().map(|b| b.percent));
        face.show(
            Status::of(&health, starting).lights(),
            gauge.lights(),
            flashlight_lights(state.flashlight.load(Ordering::Relaxed)),
        );
        let _ = tokio::time::timeout(PERIOD, state.leds_changed.notified()).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_gauge_fills_by_level_and_shows_nothing_it_has_not_read() {
        let fresh = |p| Gauge::Unknown.next(Some(p));
        assert_eq!(fresh(2.0), Gauge::Critical);
        assert_eq!(fresh(10.0), Gauge::Low);
        assert_eq!(fresh(30.0), Gauge::One);
        assert_eq!(fresh(55.0), Gauge::Two);
        assert_eq!(fresh(90.0), Gauge::Three);
        assert_eq!(Gauge::Three.next(None), Gauge::Unknown);
    }

    #[test]
    fn the_gauge_holds_its_level_near_a_boundary() {
        // A sag from 72% to 69% under load keeps three; a fall to 66% drops to two.
        assert_eq!(Gauge::Three.next(Some(69.0)), Gauge::Three);
        assert_eq!(Gauge::Three.next(Some(66.0)), Gauge::Two);
        // And the rest that lifts it back to 71% does not climb again.
        assert_eq!(Gauge::Two.next(Some(71.0)), Gauge::Two);
        assert_eq!(Gauge::Two.next(Some(74.0)), Gauge::Three);
        // Far from the previous level it jumps straight there.
        assert_eq!(Gauge::Three.next(Some(10.0)), Gauge::Low);
    }

    #[test]
    fn the_low_led_is_green_until_the_battery_is_low() {
        assert_eq!(
            Gauge::One.lights(),
            [Light::On, Light::Off, Light::Off, Light::Off]
        );
        assert_eq!(
            Gauge::Low.lights(),
            [Light::Off, Light::On, Light::Off, Light::Off]
        );
        assert_eq!(Gauge::Critical.lights()[1], Light::Blink);
    }

    #[test]
    fn status_tells_starting_from_degraded_from_broken() {
        let health = |healthy, degraded| proto::HealthResult {
            healthy,
            degraded,
            ..Default::default()
        };
        assert_eq!(Status::of(&health(true, false), false), Status::Healthy);
        assert_eq!(Status::of(&health(false, false), true), Status::Starting);
        assert_eq!(Status::of(&health(false, true), false), Status::Degraded);
        assert_eq!(Status::of(&health(false, false), false), Status::Fault);
    }

    #[test]
    fn the_flashlight_toggles_and_remembers_nothing_it_was_not_told() {
        let toggle = |color| proto::FlashlightParams {
            toggle: true,
            color,
            ..Default::default()
        };
        let on = flashlight_after(0, &toggle(proto::FlashlightColor::Cyan));
        assert_eq!(flashlight_lights(on), [Light::Off, Light::On, Light::On]);
        assert_eq!(
            flashlight_after(on, &toggle(proto::FlashlightColor::Cyan)),
            0
        );
        let white = flashlight_after(
            0,
            &proto::FlashlightParams {
                on: true,
                ..Default::default()
            },
        );
        assert_eq!(flashlight_lights(white), [Light::On; 3]);
        assert_eq!(
            flashlight_after(white, &proto::FlashlightParams::default()),
            0
        );
    }
}
