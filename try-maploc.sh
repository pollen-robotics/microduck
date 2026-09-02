#!/bin/zsh
# Bring up one duck in the MuJoCo apartment with maploc mapping, and a viewer
# that draws the map it builds. Investigation scaffolding -- not for merging.
#
#   ./try-maploc.sh up        body_server + tofd + robotd, viewer with overlay
#   ./try-maploc.sh walk      stop-and-scan route (this is what makes a map)
#   ./try-maploc.sh monitor   robotctl monitor, map panel and all
#   ./try-maploc.sh map       print one MapFrame's stats
#   ./try-maploc.sh down      stop only what this script started
#
# Uses port 7871 and /tmp/dsm so it cannot collide with the 7801+ ducks.
set -eu

REPO=${0:A:h}
RL=/Users/schade/Pollen/microduck_rl
STATE=/tmp/dsm
PORT=7871
SOCK=$STATE/a.sock
TOFSOCK=$STATE/a-tof.sock
ctl() { $REPO/target/debug/robotctl --robot-socket $SOCK "$@"; }

# robotd runs from a copy, not from target/debug: a stray
# `pkill -f "target/debug/robotd --sim"` on this machine killed it twice.
ROBOTD=$STATE/robotd-maploc

# Start a process in its own session, so a Ctrl-C or a process-group TERM aimed
# at this shell does not reach it. robotd handles SIGTERM and SIGINT -- that is
# how systemd stops it -- so it must not share our group.
detach() { /usr/bin/python3 $REPO/sim-maploc/detach.py "$@"; }

case ${1:-up} in
up)
    mkdir -p $STATE
    # Refuse to start on top of a previous run. Without this the new
    # body_server loses the port, the new robotd loses the socket, both exit,
    # and `health` cheerfully answers from the old pair -- which looks like
    # success and is not.
    if /usr/bin/python3 -c "import socket,sys;s=socket.socket();s.settimeout(.3);sys.exit(s.connect_ex(('127.0.0.1',$PORT)))" 2>/dev/null; then
        echo "port $PORT is already serving -- run './try-maploc.sh down' first" >&2
        exit 1
    fi
    ORT="$(ls $RL/.venv/lib/python*/site-packages/onnxruntime/capi/libonnxruntime*.dylib | head -1)"
    cp -f $REPO/target/debug/robotd $ROBOTD

    cat > $STATE/robotd.toml <<TOML
[policy]
enabled = true
walk = "$REPO/policies/alpha_walking.onnx"
stand = "$REPO/policies/alpha_stand.onnx"
sitstand = "$REPO/policies/alpha_sitstand.onnx"
ground_pick = "$REPO/policies/alpha_ground_pick.onnx"
kick_left = "$REPO/policies/ball_kick_left.onnx"
kick_right = "$REPO/policies/ball_kick_right.onnx"
roulade = "$REPO/policies/roulade.onnx"

[chorale]
accept = false

[audio]
enabled = false
bank = "$STATE/sounds/duck-m"
device = "default"

# maploc reads tofd's socket through this path. Without the --socket override
# committed alongside this script it would look in /run/tofd, which macOS has
# no such thing as.
[theremin]
enabled = false
socket = "$TOFSOCK"

[maploc]
enabled = true
mode = "stop_and_scan"
map_path = "$STATE/maploc.session"
wipe_on_boot = true
search_sweep = true
# Every odometry tick and depth frame the mapper consumes, appended to a
# timestamped .mdlg. maploc's evaluate example replays one byte-for-byte,
# so the debugging loop stops being "re-run the simulator and hope".
# Kept inside the worktree, not /tmp, so a reboot does not eat the sessions.
record_dir = "$REPO/recordings"
TOML

    echo "== body_server + map overlay (viewer) on $PORT"
    ( cd $RL && PYTHONPATH=src MAP_FALLBACK= \
        detach $STATE/body.log $RL/.venv/bin/mjpython \
        $REPO/sim-maploc/body_with_map.py --port $PORT --ducks 1 --keyframe SIT \
        --scene $RL/src/mjlab_microduck/robot/microduck/scene_apartment.xml \
        --robot-socket $SOCK ) > $STATE/body.pid
    for i in $(seq 1 120); do
        /usr/bin/python3 -c "import socket,sys;s=socket.socket();s.settimeout(.3);sys.exit(s.connect_ex(('127.0.0.1',$PORT)))" 2>/dev/null && break
        sleep 0.5
    done

    echo "== tofd --sim"
    detach $STATE/tofd.log $REPO/target/debug/tofd \
        --sim 127.0.0.1:$PORT --socket $TOFSOCK --hz 15 > $STATE/tofd.pid
    sleep 1

    echo "== robotd --sim (maploc on)"
    DUCK_IDENTITY=duck-m DUCK_RUNTIME_DIR=$STATE ORT_DYLIB_PATH="$ORT" RUST_LOG=info \
        detach $STATE/robotd.log $ROBOTD --sim 127.0.0.1:$PORT \
        --params $STATE/robotd.toml --socket $SOCK > $STATE/robotd.pid
    for i in $(seq 1 150); do
        [ -S $SOCK ] && ctl health >/dev/null 2>&1 && break
        sleep 0.2
    done
    ctl health | head -4
    grep -q "connected to tofd" $STATE/robotd.log && echo "  maploc    connected to tofd"
    echo
    echo "pids: body $(cat $STATE/body.pid) tofd $(cat $STATE/tofd.pid) robotd $(cat $STATE/robotd.pid)"
    echo "next: ./try-maploc.sh walk    (a map needs the stops -- see below)"
    ;;

walk)
    ctl robot enable --on >/dev/null 2>&1 || \
        /usr/bin/python3 -c '
import json, socket
s = socket.socket(socket.AF_UNIX); s.connect("'$SOCK'")
f = s.makefile("rw")
f.write(json.dumps({"jsonrpc":"2.0","id":1,"method":"robot.enable",
                    "params":{"on":True}}) + "\n"); f.flush()
print(f.readline().strip())'
    sleep 8
    detach $STATE/scan.log $RL/.venv/bin/python -u $REPO/sim-maploc/scan_walk.py ${2:-300} \
        > $STATE/scan.pid
    echo "walking (pid $(cat $STATE/scan.pid)); tail -f $STATE/scan.log"
    ;;

# --tof-socket, or the depth panel looks for /run/tofd/tof.sock -- the
# board's path, which macOS has no such thing as, and the panel then
# reads "no depth stream" as if no sensor were fitted.
monitor) exec $REPO/target/debug/robotctl --robot-socket $SOCK --tof-socket $TOFSOCK monitor ;;
map)     exec $RL/.venv/bin/python $REPO/sim-maploc/mapdump.py ${2:-1} ;;
health)  ctl health ;;

down)
    for n in scan robotd tofd body; do
        [ -f $STATE/$n.pid ] || continue
        p=$(cat $STATE/$n.pid)
        if ps -p $p >/dev/null 2>&1; then echo "stop $n ($p)"; kill $p; fi
        rm -f $STATE/$n.pid
    done
    ;;
*) echo "usage: $0 {up|walk|monitor|map|health|down}"; exit 2 ;;
esac
