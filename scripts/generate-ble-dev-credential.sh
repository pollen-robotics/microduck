#!/usr/bin/env bash
# Generate one development BLE credential. Never run this in a source checkout.
set -euo pipefail
set -C
umask 077

if [[ $# -ne 2 || ! "$1" =~ ^[A-Za-z0-9._-]{1,128}$ ]]; then
    echo 'usage: generate-ble-dev-credential.sh ROBOT_ID OUTPUT_FILE' >&2
    exit 2
fi

robot_id=$1
output_file=$2
openssl rand -hex 32 > "$output_file"
key=$(< "$output_file")
printf 'Development app setup code: md1:%s:%s\n' "$robot_id" "$key"
