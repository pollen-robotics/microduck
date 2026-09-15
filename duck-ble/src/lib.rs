//! The BLE wire contract, and nothing that serves it.
//!
//! Three things a client and the robot must agree on exactly, extracted from
//! `btd` so that agreeing does not require depending on the daemon:
//!
//! | | |
//! |---|---|
//! | [`gatt`] | the service and characteristic UUIDs |
//! | [`framing`] | how a line is cut into notifications and put back together |
//! | [`adv`] | what the advertisement carries besides the name |
//!
//! Each was already marked as wire contract where it lived, and each says why
//! in its own header. What they have in common is that **a second
//! implementation would agree only with itself** — a client that chunked
//! differently, or decoded the advertisement's address by hand, would work
//! until it did not, and the failure would look like a robot problem.
//!
//! **Nothing here touches a radio.** No `bluer`, no `btleplug`, no async
//! runtime, no logging, no argument parser: this crate is bytes and
//! arithmetic, so it builds for a phone, a laptop and the board alike. That is
//! the whole reason it exists apart from `btd`, which needs all of those and
//! only runs on Linux — `mobile-app.md` §3 records the app carrying `clap` and
//! `tracing-subscriber` into an iPhone binary for the sake of one module, and
//! this is that being fixed rather than noted.

pub mod adv;
pub mod framing;
pub mod gatt;
