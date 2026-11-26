#!/usr/bin/env bash
# End-to-end test of the real app on a device or emulator: the duck out on the screen over another
# app — which is where its owner found it ignoring food and the ball — driven through the debug
# bridge (DebugBridge.kt, debuggable builds only) and checked against what it actually did.
#
#   virtual-duck/tools/e2e.sh [path/to/app-debug.apk]
#
# SPEED runs the duck's logic that many times faster than the frames it draws — a software-rendered
# emulator manages a couple of frames a second, a phone does not need it. TIMEOUT is per scenario.
set -u

ADB=${ADB:-adb}
P=io.github.samimreza.virtualduck
SPEED=${SPEED:-1}
TIMEOUT=${TIMEOUT:-90}
APK=${1:-}
passed=0
failed=0
results=()

send() { $ADB shell am broadcast -a $P.DEBUG -p $P --es target "$1" --es cmd "$2" >/dev/null 2>&1; }
status() { $ADB shell run-as $P cat "files/debug/$1.json" 2>/dev/null; }

# get <target> <python expression over d> — e.g. get overlay "d['pellets']"
get() {
    send "$1" status
    status "$1" | python3 -c "import json, sys
try:
    d = json.load(sys.stdin)
    print($2)
except Exception:
    print('')"
}

# until <target> <python condition over d> [seconds] — true once the condition holds.
until_true() {
    local deadline=$((SECONDS + ${3:-$TIMEOUT}))
    while [ $SECONDS -lt $deadline ]; do
        [ "$(get "$1" "bool($2)")" = "True" ] && return 0
        sleep 2
    done
    return 1
}

alive() { [ -n "$($ADB shell pidof $P 2>/dev/null | tr -d '\r')" ]; }
overlay_up() { $ADB shell dumpsys activity services $P 2>/dev/null | grep -q 'overlay.PetService'; }

check() {
    local name=$1
    shift
    if "$@"; then
        passed=$((passed + 1))
        results+=("PASS  $name")
        echo "PASS  $name"
    else
        failed=$((failed + 1))
        results+=("FAIL  $name   $(get "${TARGET:-overlay}" "d")")
        echo "FAIL  $name"
    fi
}

# The owner throws seeds: they are all eaten, and the count goes up.
eats() {
    local before
    before=$(get "$1" "d['pellets_eaten']")
    send "$1" feed
    until_true "$1" "d['pellets'] == 0 and d['pellets_eaten'] > ${before:-0}"
}

# The owner throws the ball: it is kicked.
kicks() {
    local before
    before=$(get "$1" "d['kicks']")
    send "$1" ball
    until_true "$1" "d['kicks'] > ${before:-0}"
}

wake_screen() {
    $ADB shell input keyevent KEYCODE_WAKEUP
    $ADB shell wm dismiss-keyguard >/dev/null 2>&1
    sleep 2
}

# ------------------------------------------------------------------ setup

if [ -n "$APK" ]; then
    # Built with the voices it reads: an APK packed before render_voices.py ran has an index it cannot use.
    unzip -p "$APK" assets/duck/voices.json 2>/dev/null | head -c 4096 | grep -q '"styles"' || {
        echo "$APK has no current voice index: run tools/render_voices.py --out android/app/src/main/assets/duck, then rebuild" >&2
        exit 1
    }
    $ADB install -r "$APK" >/dev/null || exit 1
fi
$ADB shell am force-stop $P
# The last run's status files would answer the readiness waits before this run's duck exists.
$ADB shell run-as $P rm -rf files/debug
$ADB shell appops set $P SYSTEM_ALERT_WINDOW allow
$ADB shell pm grant $P android.permission.POST_NOTIFICATIONS >/dev/null 2>&1
$ADB logcat -c
wake_screen

# ------------------------------------------------------------------ home

$ADB shell am start -n $P/.home.HomeActivity >/dev/null
for _ in $(seq 1 30); do [ -n "$(get home "d.get('activity')")" ] && break; sleep 2; done
send home "speed:$SPEED"
TARGET=home check "home: seeds are eaten" eats home
TARGET=home check "home: the ball is kicked" kicks home

# ------------------------------------------------------------------ out on the screen, over another app

# Let it out the way the owner does: the button on the home screen.
$ADB shell uiautomator dump /sdcard/ui.xml >/dev/null 2>&1
xy=$($ADB shell cat /sdcard/ui.xml | python3 -c "import re, sys
m = re.search(r'text=\"Put the duck on my screen\"[^>]*bounds=\"\[(\d+),(\d+)\]\[(\d+),(\d+)\]\"', sys.stdin.read())
print((int(m[1]) + int(m[3])) // 2, (int(m[2]) + int(m[4])) // 2) if m else print('')")
if [ -n "$xy" ]; then
    $ADB shell input tap $xy
else
    # Scrolled out of view: the service can be started from the home screen's own code path only, so scroll.
    $ADB shell input swipe 540 1700 540 900 300
    sleep 1
    $ADB shell uiautomator dump /sdcard/ui.xml >/dev/null 2>&1
    xy=$($ADB shell cat /sdcard/ui.xml | python3 -c "import re, sys
m = re.search(r'text=\"Put the duck on my screen\"[^>]*bounds=\"\[(\d+),(\d+)\]\[(\d+),(\d+)\]\"', sys.stdin.read())
print((int(m[1]) + int(m[3])) // 2, (int(m[2]) + int(m[4])) // 2) if m else print('')")
    $ADB shell input tap $xy
fi
for _ in $(seq 1 30); do [ -n "$(get overlay "d.get('activity')")" ] && break; sleep 2; done
check "the duck comes out on the screen" overlay_up

# The other screen: an ordinary app in front, the duck over it.
$ADB shell am start -a android.settings.SETTINGS >/dev/null
sleep 3
send overlay "speed:$SPEED"
TARGET=overlay

check "over another app: seeds are eaten" eats overlay
check "over another app: the ball is kicked" kicks overlay

# A sound starts — a video in the app underneath, a door — and never becomes speech.
send overlay listen
sleep 3
check "after a noise that was not speech: seeds are eaten" eats overlay
send overlay listen
send overlay "heard:3"
sleep 4
check "after being talked to: the ball is kicked" kicks overlay

send overlay sleep
sleep 4
check "asleep: food wakes it and is eaten" eats overlay

send overlay dance
sleep 1
check "mid-dance: seeds are eaten after" eats overlay

send overlay shake
check "after a tumble: the ball is kicked" kicks overlay

$ADB shell input keyevent KEYCODE_SLEEP
sleep 4
wake_screen
$ADB shell am start -a android.settings.SETTINGS >/dev/null
sleep 2
check "after the screen was off: seeds are eaten" eats overlay

# Home opened and closed again: the overlay steps aside and comes back.
$ADB shell am start -n $P/.home.HomeActivity >/dev/null
sleep 5
$ADB shell am start -a android.settings.SETTINGS >/dev/null
sleep 3
check "after home was opened and closed: the ball is kicked" kicks overlay

# A few minutes of somebody playing with everything at once.
for i in $(seq 1 40); do
    case $((RANDOM % 9)) in
        0) send overlay feed ;; 1) send overlay ball ;; 2) send overlay dance ;; 3) send overlay trick ;;
        4) send overlay poke ;; 5) send overlay stroke ;; 6) send overlay listen ;; 7) send overlay "heard:2" ;;
        8) send overlay sleep ;;
    esac
    sleep 1
done
check "after a burst of everything: still alive" alive
check "after a burst of everything: seeds are eaten" eats overlay

crashes() { $ADB logcat -d | grep -E "FATAL EXCEPTION|AndroidRuntime: .*$P" | head -3; }
check "no crashes in the log while playing" test -z "$(crashes)"

# Every voice in every pitch loads and speaks; then back to the default.
$ADB logcat -c
for v in babble duckling squeaky; do
    for p in 0 1 2; do
        send overlay "voice:$v"
        send overlay "pitch:$p"
        sleep 2
    done
done
voice_errors() { $ADB logcat -d | grep -E "SoundVoice|FileNotFound" | grep -iE "could not|error|exception" | head -3; }
check "every voice loads" test -z "$(voice_errors)"
check "and the duck is set to the last one asked for" test "$(get overlay "d['voice']")" = "squeaky-2"
send overlay "voice:babble"
send overlay "pitch:1"

check "no crashes in the log while changing voices" test -z "$(crashes)"

echo
echo "${passed} passed, ${failed} failed"
printf '%s\n' "${results[@]}" | grep FAIL
[ $failed -eq 0 ]
