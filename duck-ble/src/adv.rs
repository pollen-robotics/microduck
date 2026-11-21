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
//!
//! ## The extended twin carries one byte more
//!
//! `btd` puts the same payload on the air twice: once as a legacy advertisement and once as an
//! extended one, because the robot's controller cannot take a connection from a central that offers
//! channel selection algorithm #2 over a legacy advertisement (`app-path-design.md` §3.8). The two
//! arrive from different random addresses, so to a scanner they are two peripherals with the same
//! name — which `duckctl` refuses as a collision unless it can tell they are one robot.
//!
//! [`EXTENDED_MARK`] after the address is how it tells. The extended advertisement has no 31-byte
//! budget, so the byte costs nothing there, and the legacy payload is unchanged.

use std::collections::HashMap;
use std::net::Ipv4Addr;

/// The company id the payload is filed under.
///
/// `0xFFFF` is the id the Bluetooth SIG reserves for internal and interoperability testing, and it
/// is the correct choice for a project that has not been assigned one. Anyone else may use it too,
/// so **this is not an identity check**: [`address_in`] is only ever asked about a device that
/// already advertised [`crate::gatt::SERVICE_UUID`], which is the discriminator.
pub const COMPANY_ID: u16 = 0xFFFF;

/// The byte after the address that says "this is the extended twin of a legacy advertisement".
pub const EXTENDED_MARK: u8 = 0x01;
/// A non-connectable broadcast from a robot already serving another central.
pub const BUSY_MARK: u8 = 0x02;

/// Address payload for the non-connectable busy advertisement.
pub fn busy_address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    let mut data = address_data(address);
    data.push(BUSY_MARK);
    data
}

/// True only for the explicitly marked non-connectable broadcast.
pub fn is_busy(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    manufacturer_data
        .get(&COMPANY_ID)
        .is_some_and(|data| data.len() == 5 && data[4] == BUSY_MARK)
}

/// The robot's address as it goes into the advertisement.
///
/// `None` — no wifi, or `configd` would not say — becomes [`Ipv4Addr::UNSPECIFIED`] rather than an
/// absent field, for the reason in this module's docs.
pub fn address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    address.unwrap_or(Ipv4Addr::UNSPECIFIED).octets().to_vec()
}

/// [`address_data`] followed by [`EXTENDED_MARK`], for the extended advertisement.
pub fn extended_address_data(address: Option<Ipv4Addr>) -> Vec<u8> {
    let mut data = address_data(address);
    data.push(EXTENDED_MARK);
    data
}

/// The four address bytes of a field in either layout, or `None` for anything else.
fn address_bytes(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<[u8; 4]> {
    match manufacturer_data.get(&COMPANY_ID)?.as_slice() {
        [a, b, c, d] | [a, b, c, d, EXTENDED_MARK | BUSY_MARK] => Some([*a, *b, *c, *d]),
        _ => None,
    }
}

/// Whether this is the extended twin — see this module's docs.
pub fn is_extended(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    manufacturer_data
        .get(&COMPANY_ID)
        .is_some_and(|data| data.len() == 5 && data[4] == EXTENDED_MARK)
}

/// The address a scan reported, if it reported one.
///
/// `None` covers three cases that a listing renders differently and this function does not
/// distinguish, because it cannot: no field at all, a field of the wrong length, and a field
/// saying `0.0.0.0`. The caller has the advertisement and can tell the first from the third; see
/// `duckctl`'s listing, which does.
pub fn address_in(manufacturer_data: &HashMap<u16, Vec<u8>>) -> Option<Ipv4Addr> {
    Some(Ipv4Addr::from(address_bytes(manufacturer_data)?))
        .filter(|address| !address.is_unspecified())
}

/// Whether this device broadcast an address field at all, however it reads.
///
/// Separate from [`address_in`] because "an older release, which broadcasts nothing" and "a robot
/// that is not on wifi" are the two things a blank address could mean, and a listing that cannot
/// tell them apart sends the reader to check the wrong thing.
pub fn has_address_field(manufacturer_data: &HashMap<u16, Vec<u8>>) -> bool {
    address_bytes(manufacturer_data).is_some()
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_busy_broadcast_keeps_its_address_but_is_never_a_connectable_twin() {
        use super::*;
        let address = Ipv4Addr::new(192, 0, 2, 1);
        let data = HashMap::from([(COMPANY_ID, busy_address_data(Some(address)))]);
        assert!(is_busy(&data));
        assert!(!is_extended(&data));
        assert_eq!(address_in(&data), Some(address));
        assert!(!is_busy(&HashMap::from([(
            COMPANY_ID,
            address_data(Some(address))
        )])));
        assert!(!is_busy(&HashMap::from([(
            COMPANY_ID,
            extended_address_data(Some(address))
        )])));
    }
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

    /// The extended twin reads as the same address, and is the only one that says it is the twin.
    #[test]
    fn the_extended_twin_carries_the_same_address() {
        let address = Ipv4Addr::new(192, 168, 1, 42);
        let legacy = HashMap::from([(COMPANY_ID, address_data(Some(address)))]);
        let extended = HashMap::from([(COMPANY_ID, extended_address_data(Some(address)))]);
        assert_eq!(address_in(&extended), Some(address));
        assert!(has_address_field(&extended));
        assert!(is_extended(&extended));
        assert!(!is_extended(&legacy));
    }

    /// Four bytes, or four and the mark. Anything else is another vendor using `0xFFFF`, or a
    /// format this client does not know, and either way it is not an address.
    #[test]
    fn a_payload_of_the_wrong_length_is_not_an_address() {
        let mut five = vec![1; 4];
        five.push(0xff);
        let data = HashMap::from([(COMPANY_ID, five)]);
        assert_eq!(address_in(&data), None, "five bytes without the mark");
        assert!(!is_extended(&data));
        for length in [0, 1, 3, 6, 16] {
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
