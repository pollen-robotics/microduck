# Authenticated app development MVP

This is an **opt-in development path**, not the production provisioning procedure. `btd`'s
normal PIN mode remains the installed default. The app branch is
[`codex/authenticated-ble-mvp`](https://github.com/pollen-robotics/microduck-app/tree/codex/authenticated-ble-mvp).
Do not install the secure mode until both matching builds are available. The development setup
code is a bearer secret: anyone who sees it can authenticate to the robot.

## Production roadmap and current scope

The production goal is to protect camera/control access and Wi-Fi credentials, then qualify the
exact shipping image. Work is ordered as follows:

1. Reliable in-app Bluetooth onboarding, reconnects, wrong-key and unauthenticated-access tests
   on Android and iPhone.
2. Complete Wi-Fi provisioning, network rediscovery, camera/control and connection-loss tests.
3. Factory provisioning of unique robot secrets, QR labels, protected app storage, ownership,
   revocation and recovery.
4. Triage and fix exploitable OS/kernel/bootloader/library findings on the actual shipping image.
5. Qualify installation, update, rollback and recovery; independently review authentication.

Steps 1 and 2 are the active scope. The beta board is a compatibility test device; results on it
do not replace production-image qualification. Movement tests require a clear test area and a
supported robot pose; read-only control-channel tests can run on the desk.

Step 3's proposed factory workflow, Recovery USB versus charge-port wiring, and outstanding
hardware confirmations are documented in [factory provisioning](../design/factory-provisioning.md).

## Prepare one robot

The setup code is `md1:<first name>:<64 hex characters>`: format 1 of a microduck setup code, the
robot's `duck-xxxx` first name (derived from its SoC serial, so a rename does not change it), and
its 32-byte key. A production robot carries it in its ID EEPROM, written by the factory, and printed
as the QR on the robot ([factory provisioning](../design/factory-provisioning.md)). A robot with a
code there needs nothing below: secure mode is on from its first boot.

A dev board has nothing in its EEPROM. Generate a key on the robot and install it:

```bash
key=$(head -c 32 /dev/urandom | od -An -tx1 -v | tr -d ' \n')
```

```bash
echo "$key" | sudo robotctl system setup-code install
```

```bash
unset key
```

Restart the services so they pick it up:

```bash
sudo systemctl restart configd btd duck-gateway
```

Show the code, then scan the QR or paste the `md1:` line into the app:

```bash
sudo robotctl system setup-code
```

`configd`'s unit hands the key to `btd` and `duck-gateway` at every start
(`robotctl system setup-code export`). With a key, both serve secure mode; without one, `btd`
serves the PIN path and the gateway stays stopped. An EEPROM code whose id is not this robot's
first name is refused, and secure mode stays off. `install` refuses to replace an existing key
without `--force`, because that invalidates every phone holding the old code.

## App flow

Paste the `md1:` code into the app's access-code field. This MVP does not yet scan QR codes or
save the secret: it stays in app memory and must be entered again after restarting the app. The
app performs the Noise handshake before `hello`, then encrypts `net.connect` and subsequent BLE
requests and replies. A robot in secure mode rejects the six-digit PIN and plaintext RPCs. The
app never writes the `md1:` code to `robots.json`.

The app's legacy PIN path still opens LAN media directly. Treat this as an opt-in development
MVP, not a production security claim. Before production: QR scanning, owner enrollment and
revocation, protected device storage, factory credential provisioning and recovery, independent
protocol review, rate limiting, iPhone testing, and tests on the production face-board-v1 image
remain required.

## Beta board validation (8 November 2026)

The paragraphs in this section describe the initial test. The follow-up results below supersede
the Android system-settings workaround and the previously untested Wi-Fi/reconnection cases.

The signed `0.18.9-dev.1646.c829879` daemon release passed this robot's update dry run and health
gate on a face-board-v1 beta unit. With a temporary per-robot key and runtime-only service
overrides, the Android debug app authenticated over BLE, read robot and Wi-Fi status, and displayed
the camera through `duck-gateway`. The WebRTC control channel opened; no movement command was
tested. A wrong gateway key was refused before `mediad` opened, and direct LAN access to port 8443
was refused. The app's `robots.json` did not contain the setup code or key.

Android's automatic GATT-triggered first bond timed out on the tested phone. Pairing in Android
Bluetooth Settings first succeeded, after which the app's secure handshake and onboarding worked.
The app PR fixes Android keyboard capitalization of `md1:` and explains the pairing recovery step;
that follow-up build still needs a phone retest. iPhone, Wi-Fi password entry, unpaired writes,
reconnection, and movement control remain unverified. The test key, phone bond, and runtime
overrides were removed, and the robot was restored to its exact pre-test daemon release.

## Follow-up: onboarding and connection loss

Tested on the same beta robot and Nothing Phone (3), using the Android app through `88f4d50`
and the `6ca1d79` native ARM board-test `btd` artifact (SHA-256
`9c39a000781f6c30926bfb07b802694ee576e8e9d23308437edd2908690ee59e`). Other services used the
previously signed development release. This is a bench compatibility test, not a production image
qualification. No secret, passphrase or raw Bluetooth capture belongs in the repository.

iPhone follow-up used the app through `a5146a3`; a clean installation of `7455e2c` also completed
BLE authentication. It used the `8d999f5` native ARM board-test `btd` artifact, SHA-256
`98ed385c3921cb6078682b2c3c6377c519b79880e83a3797fca6be6b69a00c91`.

| Check | Observed result |
| --- | --- |
| First Android bond | System pairing dialog opened from the app, before protected GATT access; no trip to Bluetooth Settings. |
| Cancel and retry | Cancelling returned to the access-code form; retry could start pairing again. |
| Wrong setup key | No application session opened. Correcting the key on the same form then succeeded without a `btd` restart. |
| Fresh Wi-Fi provisioning | The saved profile was removed with a timed recovery plan in place. The password was entered through the app over authenticated BLE; a new NetworkManager profile joined and obtained IPv4. Reassembling the 31 ATT writes produced four `data` envelopes, no plaintext RPCs, and no password in raw or base64-decoded payloads. |
| Camera/control | Live 1280×720 video and an open WebRTC control channel after Wi-Fi provisioning. A 0.03-radian head-yaw request was accepted and appeared in robot state; no joint motion was measured in the held/fallen pose. |
| App backgrounding | The app closed its WebRTC peer/control channel. Robot telemetry first reported `deadman` about 0.616 s after Home; applied forward intent fell below 0.001 m/s after 0.918 s. |
| Phone Wi-Fi loss | Eight seconds without phone Wi-Fi closed the control session. `deadman` appeared after about 0.614 s; applied forward intent fell below 0.001 m/s after 0.918 s. Wi-Fi was re-enabled in a `finally` cleanup. |
| Explicit media reconnect | Video/control reopened; requested velocity was zero and the previous nonzero test intent did not resume. |
| Forced app termination | `btd` detected the disconnected phone, discarded authentication/CCC state and registered again after its five-second retry delay. A relaunched app rediscovered the actual robot and could reauthenticate without a service restart. |
| Busy broadcasts | The shared advertisement format marks non-connectable broadcasts; the app filters them rather than offering their random address as a pairing target. The matched app rediscovered the robot after rapid relaunch and rescan. |
| iOS build | Full unsigned Xcode archive succeeded after explicitly linking AuthenticationServices. CI now links the complete iOS app too, rather than checking only its Rust library. |
| iPhone BLE onboarding | After Apple account reauthentication, an isolated development app signed, installed and ran on the real iPhone. Discovery and secure authentication passed. A wrong key was refused; cancelling/reopening the form with the correct key succeeded without restarting `btd`. |
| iPhone camera and backgrounding | The authenticated camera/control channel opened and camera frames were inspected privately. Home closed media; returning showed video paused. Explicit reconnect opened the authenticated channel again and displayed a new camera frame. No movement was requested in this test. |
| iPhone fresh Wi-Fi provisioning | The original saved profile was removed with a five-minute recovery timer. App password entry created a new profile and obtained IPv4. All 37 captured ATT writes reassembled as encrypted `data` envelopes; no plaintext RPC/password was present, including after base64 decoding. A quiet foreground setup step lasted 65 seconds with lease renewal enabled. |

The two stop tests used a 0.03 m/s command on a robot reporting `policy=held` and `fallen=true`.
These timings measure the command gate, not physical stopping distance during walking. Actual
walking and physical stop behaviour, an adversarial unpaired GATT test, and iPhone phone-Wi-Fi-loss
testing remain open. Apple signing is resolved; iPhone fresh password provisioning and
camera/background reconnect now pass. Phone Wi-Fi loss is not tested while the iPhone's debug
connection itself depends on Wi-Fi. On a clean iPhone install, the first LAN connection failed
while iOS asked for local-network permission; after Allow, the app's explicit video retry is needed.

After fresh provisioning, the original Wi-Fi profile was restored with its original UUID and
verified byte-for-byte against the protected pre-test backup. No Wi-Fi password was logged here.

The follow-up changes cap unfinished secure handshakes at 15 seconds, explicitly unsubscribe
before a client disconnect, and clear joystick state/close media on backgrounding or connection
failure. Authenticated sessions expire after 60 seconds without inbound requests: iOS can retain
the physical BLE link after an app exits. A visible app renews that lease with a read-only request
every 20 seconds, including on quiet setup and update screens. Extended connectable broadcasts
take priority over the board's legacy twins before remembered-device matching.

The iPhone background test exposed a security downgrade: a lost BLE ready flag made
`open_secure_media` return the legacy direct-LAN path. Secure mode now follows the selected
in-memory credential across BLE loss. Every new LAN tunnel independently authenticates with the
gateway; it never falls back to legacy signalling because BLE dropped. A regression test covers
teardown retaining secure media mode and unbinding clearing it.

Production storage, ownership, manufacturing and independent review remain roadmap steps 3–5,
not claims made by these bench results.
