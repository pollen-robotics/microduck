//! `btd` — the BLE front door onto the robot's API.
//!
//! **A transport adapter and nothing else** (`architecture.md` §4.1). `btd` owns no state, and
//! that is load-bearing rather than tidy: if provisioning or config lived here, every other
//! service would depend on `btd`, and an SDK would absurdly have to go through Bluetooth to
//! set a robot's name.
//!
//! So the design is a pipe. A GATT service with one characteristic — written to for requests,
//! subscribed to for answers — carries **the same NDJSON JSON-RPC lines as every other
//! transport** (see [`gatt`] for why one and not two), and `btd`
//! reassembles them, checks the method against [`route`]'s table, forwards them verbatim to
//! the owning service's unix socket, and chunks the replies back. Adding a method to the
//! protocol needs no change here beyond one line in that table.
//!
//! It is also the process that parses bytes from anyone in radio range, which is why it runs
//! unprivileged while `configd` — which only ever sees typed JSON from a peer-credentialled
//! local socket — is the one running as root. Putting the parser on the safe side of that
//! boundary matters more than hardening the dispatcher.
//!
//! ## Layout
//!
//! [`framing`] and [`route`] are pure logic. [`session`] is the whole of the behaviour, and it
//! reaches the radio only through [`link::Link`] — two channels, not a trait — so the tests
//! drive a complete session over real unix sockets with no Bluetooth involved. [`upstream`]
//! holds the connections to the services that own the answers.
//!
//! `net.*` and `system.*` — wifi, name, reboot — go to `configd`, one arm each in [`route`]'s
//! table. The robot's name and its IPv4 address are the two things `btd` reads back rather than
//! only forwarding: both go in the advertisement, so [`bluez`] asks `configd` for them and keeps
//! the advertisement in step. [`adv`] is the layout of the address field, shared with the client
//! that decodes it.

#[cfg(target_os = "linux")]
pub mod bluez;
pub mod chorale;
pub mod link;
pub mod pairing;
pub mod route;
pub mod session;
pub mod upstream;
