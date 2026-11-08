# Authenticated app development MVP

This is an **opt-in development path**, not the production provisioning procedure. `btd`'s
normal PIN mode remains the installed default. The app branch is
[`codex/authenticated-ble-mvp`](https://github.com/pollen-robotics/microduck-app/tree/codex/authenticated-ble-mvp).
Do not install the secure mode until both matching builds are available. The development setup
code is a bearer secret: anyone who sees it can authenticate to the robot.

## Prepare one robot

Choose a unique ASCII robot identifier. On a trusted computer, outside the source checkout:

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

## App flow

Paste the `md1:` code into the app's access-code field. This MVP does not yet scan QR codes or
save the secret: it stays in app memory and must be entered again after restarting the app. The
app performs the Noise handshake before `hello`, then encrypts `net.connect` and subsequent BLE
requests and replies. A robot in secure mode rejects the six-digit PIN and plaintext RPCs. The
app never writes the `md1:` code to `robots.json`.

The app's normal LAN camera/control path is still unauthenticated. Treat this MVP as a protected
BLE Wi-Fi setup and configuration path, not authorization for production camera or movement.
Before production: owner enrollment, protected device storage, a secure local gateway, factory
credential provisioning and recovery, independent protocol review, and iOS/Android tests on the
face-board-v1 image remain required.
