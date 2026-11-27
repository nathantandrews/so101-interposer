#!/usr/bin/env bash
# Build and flash the Pico. Refuses if any follower joint has torque on (the bus pauses while flashing).
#   scripts/flash.sh monitor|passthrough
set -e; source "$(dirname "$0")/env.sh"
fw=${1:-}
[ -d "$ROOT/src/$fw" ] && [ -n "$fw" ] || { echo "usage: $0 monitor|passthrough"; exit 1; }
if [ -e "$FOLLOWER" ]; then
  if fuser "$FOLLOWER" >/dev/null 2>&1; then echo "follower port is in use (teleop running?)"; exit 1; fi
  on=$(cd "$ROOT/tests" && "$BIN/python" -c "
from vuln_limits import make_bus, NAMES
b = make_bus('$FOLLOWER'); b.connect()
try: print(sum(int(b.read('Torque_Enable', m, normalize=False, num_retry=3)) for m in NAMES))
finally: b.disconnect(disable_torque=False)" 2>/dev/null | tail -1)
  [ "$on" = 0 ] || { echo "torque is on (or the follower didn't answer): '$on'. Fold the arm, stop teleop."; exit 1; }
fi
pico=$(readlink -f /dev/serial/by-id/*Pico* 2>/dev/null) || { echo "Pico not on USB"; exit 1; }
cd "$ROOT"
arduino-cli compile --fqbn rp2040:rp2040:rpipico --output-dir "build/$fw" "src/$fw" | tail -1
arduino-cli upload --fqbn rp2040:rp2040:rpipico -p "$pico" --input-dir "build/$fw" "src/$fw" | tail -1
sleep 3; echo "Pico firmware now: $(pico_mode)"
if [ "$fw" = monitor ] && [ -e "$FOLLOWER" ]; then "$BIN/python" tests/probe_block.py "$FOLLOWER" | tail -1; fi
