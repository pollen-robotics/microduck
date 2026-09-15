//! What the advertisement carries besides the name: the robot's IPv4 address.
//!
//! Platform-independent, and here rather than in [`crate::bluez`] for [`crate::gatt`]'s reason —
//! it is wire contract. The robot encodes with [`address_data`] and `duckctl` decodes with
//! [`address_in`], so the two halves cannot disagree about the layout. A decoder written
//! separately in the client would agree only with itself.
//!
//! ## Why the advertisement rather than a call
//!
//! `net.status` already reports the address, and reading it costs a connection, a bond and the
//! PIN — per robot. `duckctl scan` deliberately connects to nothing, which is what makes it the
//! command to reach for when a robot is unreachable, so a listing can only report what an
//! advertisement carries. Broadcasting the address is therefore the only way `scan` can answer
//! "where do I ssh?", and that is the question a listing is most often read to answer.
//!
//! ## Why four bytes and no more
//!
//! A legacy advertisement holds **31 bytes**, and `btd` already spends 21: flags (3) and a
//! 128-bit service UUID (2 + 16). One manufacturer-data field costs 2 for its header and 2 for the
//! company id, so the payload has 6 bytes to live in and this one uses 4. The name is not in that
//! budget — BlueZ puts a Local Name in the scan response, which has 31 bytes of its own.
//!
//! That is the whole reason the SSID is not here too: an SSID is up to 32 bytes on its own, so no
//! version of it fits. It stays a `wifi status` question.
//!
//! ## Why the address is always present
//!
//! A robot with no wifi advertises [`Ipv4Addr::UNSPECIFIED`] rather than dropping the field. The
//! field is then evidence in itself: absent means a robot on a release that predates this, present
//! and zero means a robot that has no address, and those two want different next moves. Dropping
//! the field would collapse them into one blank column.

use std::collections::HashMap;
use std::net::Ipv4Addr;

/// The company id the payload is filed under.
///
/// `0xFFFF` is the id the Bluetooth SIG reserves for internal and interoperability testing, and it
/// is the correct choice for a project that has not been assigned one. Anyone else may use it too,
/// so **this is not an identity check**: [`address_in`] is only ever asked about a device that
/// already advertised [`crate::gatt::SERVICE_UUID`], which is the discriminator.
pub const COMPANY_ID: u16 = 0xFFFF;

/// The robot's address as it goes into the advertisement.
///
/// `None` — no wifi, or `configd` would not say — becomes [`Ipv4Addr::UNSPECIFIED`] rather than an
/// absent field, for the reason in this module's docs.
pub fn address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    address.unwrap_or(Ipv4Addr::UNSPECIFIED).octets().to_vec()
}

/// The address a scan reported, if it reported one.
///
/// `None` covers three cases that a listing renders differently and this function does not
/// distinguish, because it cannot: no field at all, a field of the wrong length, and a field
/// saying `0.0.0.0`. The caller has the advertisement and can tell the first from the third; see
/// `duckctl`'s listing, which does.
pub fn address_in(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<Ipv4Addr> {
    let bytes: [u8; 4] = manufacturer_data
        .get(&COMPANY_ID)?
        .as_slice()
        .try_into()
        .ok()?;
    Some(Ipv4Addr::from(bytes)).filter(|address| !address.is_unspecified())
}

/// Whether this device broadcast an address field at all, however it reads.
///
/// Separate from [`address_in`] because "an older release, which broadcasts nothing" and "a robot
/// that is not on wifi" are the two things a blank address could mean, and a listing that cannot
/// tell them apart sends the reader to check the wrong thing.
pub fn has_address_field(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    manufacturer_data
        .get(&COMPANY_ID)
        .is_some_and(|data| data.len() == 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The round trip both halves depend on.
    #[test]
    fn an_address_survives_the_advertisement() {
        let address = Ipv4Addr::new(192, 168, 1, 42);
        let data = HashMap::from([(COMPANY_ID, address_data(Some(address)))]);
        assert_eq!(address_in(&data), Some(address));
        assert!(has_address_field(&data));
    }

    /// A robot with no wifi is distinguishable from one that never spoke about addresses: both
    /// have no address, and only one carried the field.
    #[test]
    fn no_wifi_is_a_present_field_and_no_address() {
        let data = HashMap::from([(COMPANY_ID, address_data(None))]);
        assert_eq!(address_in(&data), None);
        assert!(has_address_field(&data));

        let nothing = HashMap::new();
        assert_eq!(address_in(&nothing), None);
        assert!(!has_address_field(&nothing));
    }

    /// Four bytes exactly. Anything else is another vendor using `0xFFFF`, or a format this
    /// client does not know, and either way it is not an address.
    #[test]
    fn a_payload_of_the_wrong_length_is_not_an_address() {
        for length in [0, 1, 3, 5, 16] {
            let data = HashMap::from([(COMPANY_ID, vec![1; length])]);
            assert_eq!(address_in(&data), None, "{length} bytes");
            assert!(!has_address_field(&data), "{length} bytes");
        }
    }

    /// The field is a fifth of the advertisement, and the budget in this module's docs is the
    /// reason nothing else fits. Asserted so that a payload that grows has to come back here.
    #[test]
    fn the_payload_fits_the_budget() {
        const FLAGS: usize = 3;
        const SERVICE_UUID: usize = 2 + 16;
        const MANUFACTURER_HEADER: usize = 2 + 2;
        let spent = FLAGS + SERVICE_UUID + MANUFACTURER_HEADER + address_data(None).len();
        assert!(spent <= 31, "{spent} bytes of a 31-byte advertisement");
    }
}
