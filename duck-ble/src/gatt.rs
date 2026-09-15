//! The GATT contract: the UUIDs a client must know.
//!
//! Platform-independent on purpose. These live here rather than in [`crate::bluez`] because they
//! are part of the wire contract, exactly like the method names in `duck-ipc-proto` — the robot
//! serves them and every client must look for them, so a client that cannot compile for Linux
//! still needs them. `examples/duckctl.rs`, which runs on a laptop, is the case that proves it.
//!
//! Random v4 UUIDs rather than anything derived: they are ours, and they must not change once an
//! app has shipped against them. Written out in full so that grepping for a value finds this
//! comment.
//!
//! ## One characteristic, both directions
//!
//! A client **writes** requests to [`RPC_UUID`] and **subscribes** to the same characteristic for
//! responses. Two characteristics — a write one and a notify one — is the more conventional
//! shape, and this was written that way first. It is worse here for a specific reason: BlueZ
//! reports a write and a subscription as separate events, so with two characteristics a robot has
//! to pair the write half of one with the notify half of the other by device address, guessing at
//! the association. With one characteristic both events belong to it by construction, and a
//! connection is a genuine duplex stream.
//!
//! A characteristic with both `write` and `notify` is ordinary BLE. The cost is that it reads
//! slightly oddly in a generic browser like nRF Connect, where the same row is both.

/// The robot's service. What a client scans for.
pub const SERVICE_UUID: uuid::Uuid = uuid::uuid!("6f5d2a10-3b47-4c8e-9a1f-2d7e8c4b6019");

/// The RPC pipe: **read** it once for the API version, write NDJSON request bytes to it, and
/// subscribe to it for the answers.
///
/// Chunked in both directions, delimited by the newline that already separates NDJSON messages —
/// see [`crate::framing`], which is the module both the robot and `duckctl` use.
///
/// The read is part of the contract, not an optional nicety. It requires an authenticated
/// encrypted link, so it is what makes a central pair *before* it writes — a subscribe needs no
/// encryption, so without the read a client subscribes, has its first write silently refused, and
/// (on macOS) sees neither a prompt nor an error. It also returns the robot's `API_VERSION`, so a
/// mismatched client can say so before sending anything.
pub const RPC_UUID: uuid::Uuid = uuid::uuid!("6f5d2a11-3b47-4c8e-9a1f-2d7e8c4b6019");

#[cfg(test)]
mod tests {
    use super::*;

    /// The service and the characteristic must differ, and both are frozen once an app ships
    /// against them.
    #[test]
    fn the_uuids_are_distinct() {
        assert_ne!(SERVICE_UUID, RPC_UUID);
    }
}
