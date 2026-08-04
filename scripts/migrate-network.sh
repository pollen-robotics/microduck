#!/bin/sh
# Move wifi from netplan to NetworkManager, once, on a board that arrived on Armbian's stock
# networking.
#
# Split from `setup-board.sh` for two reasons that are not about file size:
#
#  1. **Different lifetime.** The overlay fix and the ONNX install in `setup-board.sh` are
#     permanent — every board needs them forever. This script exists *only* because Armbian's
#     stock image ships netplan + systemd-networkd + wpa_supplicant. The day we build a robot
#     image with NetworkManager already in it, this whole file is deleted and nothing else
#     changes.
#  2. **Different risk.** Everything in `setup-board.sh` is safe to run at any time. This is
#     the one step that can make a headless board unreachable, and it needs a reboot to take
#     effect. That belongs behind an explicit decision rather than inside routine bring-up.
#
# Why NetworkManager at all: netplan is a config *generator*, not a runtime network manager.
# It has no scan API, and `netplan apply` reports "config applied" rather than whether
# association actually succeeded. "Show me the networks" and "that password was wrong" are
# the two things a phone provisioning a robot needs most, so wifi goes to NetworkManager —
# which `docs/architecture.md` §3 already names as the owner of wifi credentials. Ethernet
# stays with netplan and networkd, untouched.
#
# Run it twice, either side of the reboot — the second run retires the backstop:
#
#   sudo sh migrate-network.sh
#   sudo reboot
#   sudo /usr/local/sbin/robot-migrate-network
#
# Then continue with `setup-board.sh`.
#
# Idempotent, and safe to re-run. It never reboots on its own.
set -eu

NM_CONF_DIR=/etc/NetworkManager/conf.d
# One profile name, so re-runs modify rather than accumulate.
NM_PROFILE=robot-wifi
# The boot-time backstop that undoes the cutover if it does not come up.
NET_CHECK=/usr/local/sbin/robot-net-check
NET_CHECK_UNIT=/etc/systemd/system/robot-net-check.service

# Where this script puts itself so it is still around after the reboot it asks for.
SELF=/usr/local/sbin/robot-migrate-network

# Whether this run staged the cutover, which decides what the closing advice must warn about.
cutover=0

say()  { printf '\033[1m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33mwarning:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# These four helpers are duplicated from setup-board.sh rather than sourced. Both scripts are
# fetched and run standalone — `curl … | sh` cannot source a sibling — so sharing them would
# cost a delivery mechanism to save twenty lines of shell.

# Write stdin to $1, and say whether that changed anything.
#
# Returns 0 when the file was written, 1 when it already had exactly this content. The return
# value is what keeps re-runs quiet: a script that reports work on every run teaches you to
# ignore its output.
write_file_if_changed() {
    _wfic_path=$1
    _wfic_tmp="$(mktemp)"
    cat > "$_wfic_tmp"
    if [ -f "$_wfic_path" ] && cmp -s "$_wfic_tmp" "$_wfic_path"; then
        rm -f "$_wfic_tmp"
        return 1
    fi
    mkdir -p "$(dirname "$_wfic_path")"
    install -m 0644 "$_wfic_tmp" "$_wfic_path"
    rm -f "$_wfic_tmp"
    return 0
}

# Leave a copy somewhere that survives the reboot this script asks for.
#
# Not possible when piped (`curl | sh`), because then there is no file to copy — `$0` is the
# shell. That is fine; the closing message adapts.
persisted=0
persist_self() {
    case "$0" in
        sh|-sh|bash|-bash|/dev/fd/*|/proc/self/fd/*) return 0 ;;
    esac
    [ -f "$0" ] || return 0

    if [ "$(readlink -f "$0" 2>/dev/null)" = "$(readlink -f "$SELF" 2>/dev/null)" ]; then
        persisted=1
        return 0
    fi

    if install -m 0755 "$0" "$SELF" 2>/dev/null; then
        persisted=1
    else
        warn "could not copy this script to ${SELF}; you will need to fetch it again after
  the reboot."
    fi
}

check_environment() {
    [ "$(id -u)" = 0 ] || die "run as root (sudo sh migrate-network.sh)"

    for tool in systemctl apt-get install mktemp; do
        command -v "$tool" >/dev/null 2>&1 || die "${tool} is required"
    done

    if [ ! -d /etc/netplan ] && ! command -v nmcli >/dev/null 2>&1; then
        die "neither netplan nor NetworkManager found, so there is nothing here to migrate.
  Not an Armbian image? Provision wifi by whatever means this one provides."
    fi
}

# Is wlan0 NetworkManager's already? "unmanaged" and "absent" both mean no.
nm_owns_wifi() {
    command -v nmcli >/dev/null 2>&1 || return 1
    _state="$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | sed -n 's/^wlan0://p')"
    case "$_state" in
        ''|unmanaged) return 1 ;;
        *) return 0 ;;
    esac
}

# The netplan file holding a `wifis:` stanza, if any. Found by content rather than by name:
# Armbian's first-run writes `30-wifis-dhcp.yaml`, but that is a convention, not a contract.
netplan_wifi_file() {
    for _f in /etc/netplan/*.yaml; do
        [ -f "$_f" ] || continue
        if grep -q '^[[:space:]]*wifis:' "$_f"; then
            printf '%s\n' "$_f"
            return 0
        fi
    done
    return 1
}

install_networkmanager() {
    if command -v nmcli >/dev/null 2>&1; then
        return 0
    fi

    # Pre-seed "manage nothing" *before* the package can start. Without this, NM comes up and
    # immediately contends with netplan's wpa_supplicant over wlan0 — two supplicants on one
    # netdev — and the link you are running this over is the one that drops.
    write_file_if_changed "${NM_CONF_DIR}/99-robot-wifi-only.conf" <<'EOF' || true
[keyfile]
unmanaged-devices=*
EOF

    say "installing NetworkManager"
    # --no-install-recommends: the recommends pull in ModemManager, and this robot has no
    # modem. apt-get rather than apt, which warns about unstable CLI output in scripts.
    export DEBIAN_FRONTEND=noninteractive
    if ! apt-get update -qq; then
        die "apt-get update failed, so NetworkManager cannot be installed.
  On a board with no battery-backed RTC this is usually the clock: a system reading 1970
  fails TLS certificate validation, and apt reports it as a repository error. Check with
  \`timedatectl\` and re-run once NTP has synchronised."
    fi
    apt-get install -y --no-install-recommends network-manager \
        || die "could not install network-manager"

    command -v nmcli >/dev/null 2>&1 || die "network-manager installed but nmcli is missing"
}

# DNS, which is the wrinkle in letting NM own an interface on a networkd box.
#
# It matters more than it looks: a robot whose wifi associates but cannot resolve names looks
# connected and cannot reach GitHub, so `updaterd` fails in a way that reads as a broken
# release rather than a broken resolver.
configure_nm_dns() {
    if systemctl is-active --quiet systemd-resolved; then
        # NM registers per-link DNS with resolved over D-Bus and leaves /etc/resolv.conf —
        # which is resolved's stub symlink — alone.
        _dns=dns=systemd-resolved
    else
        # No resolved: whatever wrote /etc/resolv.conf owns it, and it is not NM.
        _dns=dns=none
    fi

    # A separate file from 99-robot-wifi-only.conf on purpose: the backstop rewrites that one
    # wholesale, and DNS configuration living in it would vanish on a revert.
    if write_file_if_changed "${NM_CONF_DIR}/98-robot-dns.conf" <<EOF
[main]
${_dns}
rc-manager=unmanaged
EOF
    then
        say "configured NetworkManager DNS (${_dns})"
    fi
}

# Copy the credentials netplan is currently using into an NM profile.
#
# Reads `/run/netplan/wpa-wlan0.conf` rather than parsing the YAML: netplan generates it from
# the YAML in a flat, regular format, so it is both authoritative and parseable without a YAML
# parser. It only exists while netplan still owns wifi — which is exactly now.
#
# Returns 0 if a profile exists afterwards, 1 if there was nothing to migrate.
migrate_wifi_profile() {
    if nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$NM_PROFILE"; then
        say "NetworkManager profile ${NM_PROFILE} already exists"
        return 0
    fi

    _wpa=/run/netplan/wpa-wlan0.conf
    if [ ! -f "$_wpa" ]; then
        return 1
    fi

    _ssid="$(sed -n 's/^[[:space:]]*ssid="\(.*\)"[[:space:]]*$/\1/p' "$_wpa" | head -1)"
    [ -n "$_ssid" ] || return 1

    _psk="$(sed -n 's/^[[:space:]]*psk=\(.*\)$/\1/p' "$_wpa" | head -1)"
    # A quoted passphrase and a 64-hex pre-shared key are both valid here, and NM accepts
    # either — so the quotes are stripped and the value passed through as it came.
    _psk="$(printf '%s' "$_psk" | sed 's/^"\(.*\)"$/\1/')"

    # WPA3-only networks are key_mgmt=SAE and must not be described as wpa-psk, or the profile
    # is created and silently never associates.
    if grep -qi '^[[:space:]]*key_mgmt=.*SAE' "$_wpa"; then
        _keymgmt=sae
    else
        _keymgmt=wpa-psk
    fi

    say "migrating wifi credentials for \"${_ssid}\" into NetworkManager"
    nmcli connection add type wifi con-name "$NM_PROFILE" ifname wlan0 ssid "$_ssid" \
        >/dev/null || { warn "could not create the ${NM_PROFILE} profile"; return 1; }

    if [ -n "$_psk" ]; then
        # The key is on this command line, so it is briefly visible in /proc to anything else
        # on the board. Accepted: this runs once, as root, during bring-up, and the
        # alternative is hand-writing a keyfile and reimplementing NM's escaping rules. NM
        # stores it 0600 root-only from here on.
        nmcli connection modify "$NM_PROFILE" \
            wifi-sec.key-mgmt "$_keymgmt" wifi-sec.psk "$_psk" >/dev/null \
            || { warn "could not set the wifi key on ${NM_PROFILE}"; return 1; }
    fi

    return 0
}

# The backstop. Armed before the cutover, disarmed by the run after the reboot.
#
# Same principle as the update system's boot counter: the change that could make the board
# unreachable verifies itself after the reboot and undoes itself if it did not work. Without
# this, a wrong key or a missed step costs a serial cable or a card reader.
arm_net_check() {
    write_file_if_changed "$NET_CHECK" <<'EOF' || true
#!/bin/sh
# Boot-time backstop for the netplan -> NetworkManager wifi cutover, installed by
# migrate-network.sh. If wlan0 has no IPv4 address within the grace period, put netplan back
# and reboot.
#
# Every failure path still reverts: a backstop that dies halfway leaves the board dark, which
# is the outcome it exists to prevent.
i=0
while [ "$i" -lt 90 ]; do
    if ip -4 addr show dev wlan0 2>/dev/null | grep -q 'inet '; then
        exit 0
    fi
    i=$((i + 1))
    sleep 1
done
for f in /etc/netplan/*.yaml.disabled; do
    [ -f "$f" ] || continue
    mv "$f" "${f%.disabled}"
done
printf '[keyfile]\nunmanaged-devices=*\n' > /etc/NetworkManager/conf.d/99-robot-wifi-only.conf
netplan generate 2>/dev/null || true
systemctl disable robot-net-check.service 2>/dev/null || true
reboot
EOF
    chmod 0755 "$NET_CHECK"

    write_file_if_changed "$NET_CHECK_UNIT" <<'EOF' || true
[Unit]
Description=Revert the wifi cutover if it did not come up
After=NetworkManager.service

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/robot-net-check

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable robot-net-check.service >/dev/null 2>&1 \
        || warn "could not enable robot-net-check.service; the cutover has no backstop"
}

# Disarm it. Called once wifi is demonstrably NM's, because a backstop left enabled reboots
# the board on any future boot where wifi is merely slow.
retire_net_check() {
    [ -f "$NET_CHECK_UNIT" ] || return 0
    say "retiring the wifi cutover backstop"
    systemctl disable robot-net-check.service >/dev/null 2>&1 || true
    rm -f "$NET_CHECK_UNIT" "$NET_CHECK"
    systemctl daemon-reload
}

# Armbian ships its own drop-in turning this into `--any`: succeed when *any* networkd link
# comes online. Once wifi belongs to NM, networkd's only link is an ethernet port that usually
# has no cable, so `--any` can never be satisfied — it burns its whole timeout on every boot
# and then fails. `NetworkManager-wait-online` is the honest gate now, and the package enables
# it already.
#
# Note this cannot be fixed with `RequiredForOnline=no` on the ethernet link: that makes the
# link ineligible, which under `--any` guarantees the failure rather than avoiding it.
mask_networkd_wait_online() {
    if [ "$(systemctl is-enabled systemd-networkd-wait-online.service 2>/dev/null)" = masked ]; then
        return 0
    fi
    say "masking systemd-networkd-wait-online (networkd has no link left to wait for)"
    systemctl mask systemd-networkd-wait-online.service >/dev/null 2>&1 \
        || warn "could not mask systemd-networkd-wait-online; boot will stall on its timeout"
}

cut_over() {
    _netplan_wifi=""
    if _found="$(netplan_wifi_file)"; then
        _netplan_wifi="$_found"
    fi

    if [ -n "$_netplan_wifi" ] && ! migrate_wifi_profile; then
        die "netplan has a wifis: stanza in ${_netplan_wifi} but its credentials could not be
  read from /run/netplan/wpa-wlan0.conf, so handing wifi to NetworkManager would take this
  board off the network with no way back. Nothing was changed.
  Create the profile by hand, then re-run:
    nmcli connection add type wifi con-name ${NM_PROFILE} ifname wlan0 ssid \"YOUR_SSID\"
    nmcli connection modify ${NM_PROFILE} wifi-sec.key-mgmt wpa-psk wifi-sec.psk \"YOUR_PASSWORD\""
    fi

    # Only arm the backstop when there is a wifi network to come back up on. On a board with
    # no credentials at all — ethernet-only, or waiting to be provisioned over BLE — wlan0
    # will never get an address, and an armed backstop would reboot it forever.
    if [ -n "$_netplan_wifi" ]; then
        arm_net_check
    fi

    say "handing wifi to NetworkManager"

    # Renaming the netplan file rather than masking the generated unit: this stops the
    # generator emitting *both* netplan-wpa-wlan0.service and 10-netplan-wlan0.network, so NM
    # ends up owning association and DHCP with no competing config left behind. The file stays
    # on disk as the manual undo.
    if [ -n "$_netplan_wifi" ]; then
        mv "$_netplan_wifi" "${_netplan_wifi}.disabled"
    fi

    write_file_if_changed "${NM_CONF_DIR}/99-robot-wifi-only.conf" <<'EOF' || true
[keyfile]
unmanaged-devices=*,except:type:wifi
EOF

    # `netplan generate` regenerates config without the wifi stanza but does not apply it, and
    # NM is deliberately not reloaded here. Both together mean the link this script is running
    # over stays up until the reboot.
    if command -v netplan >/dev/null 2>&1; then
        netplan generate || warn "netplan generate failed; check /etc/netplan by hand"
    fi

    cutover=1
}

report() {
    echo
    if [ "$cutover" = 1 ]; then
        say "reboot required — this is the one that moves wifi"
        echo
        echo "  sudo reboot"
        if [ "$persisted" = 1 ]; then
            echo "  sudo ${SELF}"
        else
            echo "  # then fetch and run this script again — it was not copied anywhere"
            echo "  # persistent, so /tmp will have cleared it"
        fi
        cat <<EOF

  A backstop is armed: if wlan0 has no address 90s after boot, netplan is restored and the
  board reboots again by itself. If it comes back on netplan rather than NetworkManager, the
  migrated key was wrong — check it with \`nmcli -s connection show ${NM_PROFILE}\`.

  Re-running this after the reboot retires the backstop. Do not skip that: left armed, any
  later boot where wifi is merely slow reverts the board.
EOF
        return 0
    fi

    say "wifi belongs to NetworkManager — nothing left to migrate"
    if command -v nmcli >/dev/null 2>&1; then
        nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status 2>/dev/null \
            | sed 's/^/  /' || true
    fi
    cat <<'EOF'

  Continue with board bring-up:

  sudo sh setup-board.sh
EOF
}

main() {
    check_environment
    persist_self
    install_networkmanager
    configure_nm_dns

    if nm_owns_wifi; then
        # Already cut over: confirm, tidy up, and touch nothing else. Re-running must never
        # disturb a working network.
        retire_net_check
        mask_networkd_wait_online
        report
        return 0
    fi

    # Staged by an earlier run but not yet rebooted: NM is configured to take wifi and has not
    # been reloaded, so it still manages nothing. Redoing the cutover here would misreport the
    # work and, worse, find no netplan file to migrate from.
    if grep -qs 'except:type:wifi' "${NM_CONF_DIR}/99-robot-wifi-only.conf" \
        && ! netplan_wifi_file >/dev/null; then
        say "cutover already staged — pending reboot"
        mask_networkd_wait_online
        cutover=1
        report
        return 0
    fi

    cut_over
    mask_networkd_wait_online
    report
}

# Called on the last line so a truncated download — the real failure mode of `curl | sh` —
# defines functions and then does nothing, rather than running half a migration.
main "$@"
