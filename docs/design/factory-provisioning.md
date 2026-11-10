# Factory provisioning through Recovery USB (proposal)

Provision a unique authentication secret after flashing the common face-board-v1 OS image,
then print and verify its matching QR label before the robot leaves the factory.

**Status: proposed manufacturing process, not an implemented or qualified factory tool.**
A robot with a setup code in its ID EEPROM serves secure mode, and the phone app scans the
printed QR; the station tooling is not implemented, nor is production ownership/recovery. Manufacturing must
confirm the fixture and workflow; hardware must confirm the actual board revisions; security
must review the bootstrap access and exit policy. Keep the security PR draft until those gates
and production-image tests pass. [App authentication](app-path-design.md)
owns the protocol; [the MVP guide](../robot/authenticated-app-mvp.md) owns development setup and
the existing phone/robot test results.

## Which USB port

The available V0.1 schematics show **two different connectors**:

| Connector | Evidence | Factory consequence |
| --- | --- | --- |
| Power-board `USB1`, charge USB-C | Power schematic, PDF page 4 of 5: D+/D- pins have no-connect marks; VBUS feeds the charger. | Do not plan to transfer a secret through this connector unless a newer board routes its data lines. Software cannot enable missing wiring. |
| Main-board `J1`, **Recovery USB** | Main schematic, PDF page 24 of 24: D+/D- connect to `USB_OTG0_DP/DM`, towards the RK3566. | Candidate for both flashing and post-boot provisioning, using a data cable and access before enclosure closure. |

These drawings do **not** establish which connector the customer's enclosure exposes or whether
the latest production PCB/routing is unchanged. `face-board-v1` identifies the face variant;
it is not proof of the main- or power-board revision. Record all those revisions separately.

The Yocto BSP already includes an enabled `microduck-usb-gadget` recipe inherited by
face-board-v1. After Linux boots, it configures an ACM serial console and NCM Ethernet:
the robot uses `192.168.42.1`, and the host receives a `192.168.42.x` address without a default
gateway. Flashing instead uses Rockchip's recovery protocol; the same cable must re-enumerate
when Linux starts. This is not a USB mass-storage filesystem that the workstation can simply mount.

A read-only check of the beta test robot on 9 November 2026 found both gadget/DHCP services
active and the gadget bound to controller `fcc00000.usb`. The controller reported `not attached`
and `usb0` was down. This confirms configured device-side support, **not** a working host cable,
the identity of the external charge connector, or an end-to-end USB provisioning test.

The BSP notes also require battery/bench power: USB alone must not be assumed to power the SoC.
Confirm the supply arrangement on the actual production revision rather than treating that
V0.1 observation as a qualified electrical specification.

## Proposed factory sequence

The secret lives in the robot's **ID EEPROM**, written with an external programmer, not on the data
partition. A full `.wic` reflash or a factory reset keeps it, the printed QR stays valid for the
robot's life, and the at24 driver exposes the chip read-only, so software on the robot cannot
replace it. [`robotd_params::label`](../../robotd-params/src/label.rs) owns the layout:

```text
offset 0    18 ASCII digits            label serial: SKU (9) + year (2) + week (2) + unit (5)
offset 18   :                          separator
offset 19   md1:<first name>:<64 hex>  setup code, ended by a NUL or blank (0xFF) byte
```

The whole line, `<label serial>:md1:<first name>:<64 hex>` (97 characters), is also the QR
printed on the robot.

`md1` is format 1 of a microduck setup code. The first name is the robot's `duck-xxxx` default name,
`duck-` and the first four hex characters of the SHA-256 of its SoC serial
(`robotd_params::serial::first_name`). The 64 hex characters are its 32-byte key.

1. **Flash the approved common image.** Select the correct board variant and approved signed
   image/loader, and record their build identifiers and hashes. Use the main-board Recovery USB
   connector with a data cable and the approved power fixture. No per-robot firmware build or
   secret embedded in the common image is required.
2. **Boot and read the first name.** The SoC serial is only readable from the running robot:
   `/proc/cpuinfo`'s `Serial` line on face-board-v1. Read it over the post-boot USB session and
   compute the first name. Refuse a unit with no SoC serial: it cannot have a first name, so it
   cannot have a setup code. The read needs no write access to the robot, but the session still
   needs authorising (a physical cable is not authentication).
3. **Generate the key on the trusted station.** 32 bytes from a cryptographic random source,
   **random, not derived from the serial, the first name or anything else printed on the robot**:
   the printed QR is what proves somebody has the robot in hand, and a derivable key would let
   anyone compute it. Keep it out of command arguments, shell tracing, CI output and logs.
4. **Program the EEPROM** with the external programmer: the label serial from offset 0, `:` at
   offset 18 and `md1:<first name>:<64 hex>` from offset 19, exactly as above.
5. **Print the QR from the same line**, bytes 0 to 96 of the EEPROM. Render it locally, never with a public QR-generation
   website, and print the first name beside it. Treat the QR, render files and printer spool as
   secrets and remove temporary copies. Decide label placement and reprint handling before
   production.
6. **Boot and verify the robot against the printed label.**
   `sudo robotctl system setup-code` must say `from the ID EEPROM`, with the robot's own first
   name. The robot refuses an EEPROM code whose id is not its first name (another unit's image),
   and secure mode stays off. Then scan the **physical printed label** with the app and connect:
   a digital preview or a matching filename is not evidence that the right label is on the right
   robot. Reboot and connect again.
7. **Close the session and release the unit.** Remove the station's access over USB, and verify
   unauthorised USB and LAN clients cannot regain it. Record the first name, label serial, hardware
   revisions, firmware build and test results — never the key or the QR payload.

On every boot, `configd`'s unit runs `robotctl system setup-code export`, which checks the
EEPROM's code and leaves the key in systemd's credential store in `/run`. `btd` and `duck-gateway`
import it and serve secure mode; a robot with no code serves the PIN path, so a dev board or an
unprovisioned unit still onboards. So does a robot whose EEPROM code is unreadable or belongs to
another unit: secure mode stays off, the error is in `configd`'s journal and in
`robotctl system setup-code`, and the robot stays reachable over Bluetooth with the PIN from its
label serial, which a bad code does not touch. The station tooling (steps 2–5 and 7), session authorisation
and closing, and the printing workflow remain to be implemented.

## Hardware and image acceptance gates

- Obtain the controlled main-board and power-board schematics/BOM for the actual production
  revisions, including any newer supplier or Notion records. Confirm whether the charge USB
  still has no data wiring and map the externally accessible connector to the correct schematic.
- Confirm the fixture can reach Recovery USB and, when needed, the Recovery button before
  enclosure closure. If only the power-board charge port is accessible, change the fixture/access
  or hardware plan; do not substitute that connector on the assumption that USB-C implies data.
- Validate the battery/bench supply, cable, orientation and mechanical access on the production
  unit. Establish the required power sequencing from the current hardware specification.
- On the exact shipping image, verify flash enumeration, post-boot ACM/NCM enumeration, stable
  hardware serial, DHCP/interface routing and the full inject/print/scan/reboot/exit sequence
  on each supported station OS. Do not assume Linux's `/dev/ttyACM*` name exists on macOS.
- Inject faults: unplug USB or cut power during provisioning, interrupt printing, swap labels,
  retry a completed job, remove/corrupt the data mount or key, and present an unauthorized
  station. Each incomplete/mismatched unit must remain unshippable without opening a legacy gate.
- Verify a daemon update, RAUC A/B update/rollback, booting the recovery image and a full `.wic`
  reflash all leave the robot serving the EEPROM's setup code. None of them touches the EEPROM, so
  the printed QR must keep working through all of them.

Root on the robot can read the key, and anyone with the board in hand can read the EEPROM over
I2C: the key is stored unencrypted. The at24 driver exposes the chip read-only, so neither can
replace it through the kernel, but a direct I2C write could. Closing the Linux factory
session does not prove that Rockchip recovery/boot-ROM access is locked. Confirm bootloader/USB
recovery policy and the physical-attacker threat model separately with the hardware supplier.
The board-ID EEPROM and Rockchip identity storage are not established non-exportable key stores.
The QR is currently a bearer credential, not a completed first-owner enrollment or revocation
system. Lost-label/reprint policy, any encrypted credential escrow, ownership transfer and factory
recovery require explicit decisions; none is implied by storing manufacturing test metadata.

## Evidence

Pinned electronics source revision: `4bf092a4db0efd54464f7e957e425d7b9c3d4f5d`:

- [Power-board V0.1 schematic, PDF page 4 of 5](https://github.com/pollen-robotics/elec_microduck/blob/4bf092a4db0efd54464f7e957e425d7b9c3d4f5d/MicroDuck%20Power%20Board%20%20V0.1_sch_260907.pdf).
- [Main-board V0.1 schematic, PDF page 24 of 24](https://github.com/pollen-robotics/elec_microduck/blob/4bf092a4db0efd54464f7e957e425d7b9c3d4f5d/MicroDuck%20Main%20Board%20V0.1_sch_260907.pdf).

Pinned OS source revision: `63b47a8a25428d3c6fc55c23b0016931176d5236`:

- [Factory flashing and post-boot USB check](https://github.com/pollen-robotics/microduck_yocto/blob/63b47a8a25428d3c6fc55c23b0016931176d5236/docs/flashing.md).
- [Beta main-board USB/power notes](https://github.com/pollen-robotics/microduck_yocto/blob/63b47a8a25428d3c6fc55c23b0016931176d5236/docs/agents/beta-board.md).
- [USB gadget recipe](https://github.com/pollen-robotics/microduck_yocto/blob/63b47a8a25428d3c6fc55c23b0016931176d5236/meta-microduck-bsp/recipes-bsp/microduck-usb-gadget/microduck-usb-gadget.bb)
  and [implementation](https://github.com/pollen-robotics/microduck_yocto/blob/63b47a8a25428d3c6fc55c23b0016931176d5236/meta-microduck-bsp/recipes-bsp/microduck-usb-gadget/files/microduck-usb-gadget).
- [face-board-v1 inherits the same main-board BSP](https://github.com/pollen-robotics/microduck_yocto/blob/63b47a8a25428d3c6fc55c23b0016931176d5236/meta-microduck-bsp/conf/machine/microduck-face-board-v1.conf).
