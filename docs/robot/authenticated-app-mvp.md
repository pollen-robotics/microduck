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

## Prepare one robot

Choose a unique ASCII robot identifier. On a trusted computer, run this from the source checkout
and write the credential **outside** it:

```bash
scripts/generate-ble-dev-credential.sh test-duck-1 /tmp/test-duck-1-ble-psk
```

The script creates a 0600 file containing 64 hex characters and prints a
`md1:test-duck-1:...` code for the app. Keep the code private. Install the file on the robot as
`/var/lib/robot/config/ble-psk`, owned by root and mode 0600. It is then handed to unprivileged
`btd` using systemd's `LoadCredential=`. A temporary service override can use:

```ini
[Service]
LoadCredential=ble-psk:/var/lib/robot/config/ble-psk
ExecStart=
ExecStart=/opt/robot/daemon/current/bin/btd --secure-psk-file %d/ble-psk --secure-robot-id test-duck-1
```

Set `--secure-robot-id` to the exact identifier in the setup code. A missing, malformed, or
unreadable key makes `btd` exit rather than serving a weaker protocol. The `LoadCredential=` file
is copied into a read-only per-service credential directory accessible to the service user.
See [systemd credentials](https://systemd.io/CREDENTIALS/).

For local camera and control, write `/var/lib/robot/config/ble-gateway.env` containing
`ROBOT_ID=test-duck-1` (the same identifier), then start the packaged `duck-gateway.service`.
It has no `[Install]` section and is never enabled automatically. It reads the same protected
credential through systemd, listens on TCP 9443 for Noise-authenticated app connections, and
forwards them only to `mediad` on `127.0.0.1:8443`. Keep `mediad` bound to loopback. The app's
native Rust opens a one-use localhost listener for its webview, authenticates the robot over the
network, then tunnels the WebSocket through the encrypted connection.

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
