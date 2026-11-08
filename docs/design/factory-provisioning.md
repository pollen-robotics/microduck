# Factory provisioning through Recovery USB (proposal)

Provision a unique authentication secret after flashing the common face-board-v1 OS image,
then print and verify its matching QR label before the robot leaves the factory.

**Status: proposed manufacturing process, not an implemented or qualified factory tool.**
The current opt-in app/daemon MVP accepts provisioned secrets, but does not automate this process,
scan QR codes in the phone app, or implement production ownership/recovery. Manufacturing must
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

1. **Flash the approved common image.** Select the correct board variant and approved signed
   image/loader, and record their build identifiers and hashes. Use the main-board Recovery USB
   connector with a data cable and the approved power fixture. No per-robot firmware build or
   secret embedded in the common image is required.
2. **Boot and establish the factory session over the same cable.** Wait for the post-boot USB
   device to appear, then open the authorized factory channel over NCM or ACM. Verify the
   intended device/fixture rather than trusting the IP address alone. A physical USB cable is
   not authentication: the temporary SSH-key or console authorization mechanism remains to be
   implemented and approved. Do not use the documented shared bench password as the production
   bootstrap credential. Isolate or explicitly bind interfaces when several robots share a
   workstation; they otherwise all use the same USB IP address.
3. **Bind the job to the physical unit.** Read the hardware serial through that session, check
   it against the connected unit, and allocate a unique public identifier such as
   `MD-20261109-0042`. Refuse missing/invalid/duplicate identities. A date and sequence number
   identify a unit; they are not its authentication secret.
4. **Generate the secret on the trusted station.** Generate 32 bytes with a cryptographic random
   source, independently of the identifier/serial. Pass it through the authorized session without
   placing it in command arguments, shell tracing, CI output or manufacturing logs. The existing
   `scripts/generate-ble-dev-credential.sh` creates a valid development key but prints its setup
   code; it is not the production station tool.
5. **Install and commit the credential on the robot.** A privileged provisioning helper must
   verify that the expected persistent `data` partition is actually mounted at `/var/lib/robot`,
   not a fallback rootfs directory. The current MVP key path is
   `/var/lib/robot/config/ble-psk` (64 hex characters, root-owned, mode `0600`); verify parent
   ownership/ACLs also prevent replacement by unauthorized users. Bind the same robot identifier
   to BLE and the gateway. Atomically commit the key/identifier as one consistent provisioning
   generation and validate the 32-byte key and exact identifier before marking the unit provisioned.
   A power cut must leave either a valid committed generation or an unprovisioned unit.
   Refuse silent overwrites; an interrupted
   job may resume only after checking it is the same unit and credential generation. Do not
   return the secret in an acknowledgement. The common image must supply production service
   configuration; temporary MVP drop-ins are not the manufacturing implementation.
6. **Print the QR from the same committed credential.** Encode
   `md1:MD-20261109-0042:<64_HEX_CHARACTERS>` using the exact identifier and secret just programmed.
   Render locally and print the public identifier beside it. Treat the QR, render files and
   printer spool as secrets; do not use a public QR-generation website. Control label access and
   remove temporary copies. Decide label placement and reprint handling before production.
7. **Scan the physical printed label and verify the robot.** Use a bench scanner to decode the
   actual label, then complete authentication to that exact hardware serial. Verify a permitted
   read, reject a wrong key/plaintext privileged request, and check that the gateway uses the same
   identity. Reboot and repeat authentication to prove persistence. A digital preview or a matching
   filename is not evidence that the correct label was attached to the correct robot. The phone
   MVP currently accepts pasted `md1:` payloads; a production app QR reader is still required.
8. **Exit factory mode and release the unit.** Remove temporary station credentials, provisioning
   endpoints and any factory console auto-login according to the approved maintenance policy.
   Verify unauthorized USB and LAN clients cannot regain provisioning/admin access, and that
   missing/invalid production credentials fail closed instead of enabling the legacy PIN path.
   Record public robot ID, hardware serial/revisions, firmware build, credential generation,
   station/test result and exit-check result. Keep raw secrets/QR payloads out of ordinary records.
   Only a completed scan/authentication/reboot/exit test permits shipment.

The station orchestrator, privileged helper, atomic commit/retry format, factory-mode bootstrap
and exit controls, QR printing/scanning and traceability integration remain to be implemented.
The current installed default is still legacy PIN mode: production fail-closed startup is a
required change, not a property already demonstrated by this workflow.

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
- Verify a daemon update and RAUC A/B update/rollback preserve both the credential and the
  access policy, including booting the recovery image. A full `.wic` reflash erases the proposed
  data-backed secret: require an authorized reprovisioning/label replacement procedure.

Root access or a physical eMMC dump can expose a software-held secret. Closing the Linux factory
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
