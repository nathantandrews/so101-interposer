#!/usr/bin/env bash
# Recalibrate one or both arms. Needs the pass-through firmware: the monitor blocks calibration writes.
#   scripts/calibrate.sh follower|leader|both
# Put every joint at the middle of its travel when asked (wrist_roll: halfway between its two stops).
set -e; source "$(dirname "$0")/env.sh"
which=${1:-both}
case "$which" in follower|leader|both) ;; *) echo "usage: $0 follower|leader|both"; exit 1 ;; esac
mode=$(pico_mode)
if [ "$which" != leader ] && [ "$mode" = monitor ]; then
  echo "The Pico runs the MONITOR, which blocks calibration writes."
  echo "Flash pass-through first:  scripts/flash.sh passthrough"; exit 1
fi
stamp=$(date +%Y%m%d-%H%M%S); cal=~/.cache/huggingface/lerobot/calibration
mkdir -p "$ROOT/results/calibration-before-$stamp"
cp "$cal"/robots/so_follower/my_follower.json "$cal"/teleoperators/so_leader/my_leader.json \
   "$ROOT/results/calibration-before-$stamp/" 2>/dev/null || true
echo "Old calibration backed up to results/calibration-before-$stamp/"
if [ "$which" != leader ]; then need "$FOLLOWER"; "$BIN/lerobot-calibrate" "${FOLLOWER_ARGS[@]}"; fi
if [ "$which" != follower ]; then need "$LEADER"; "$BIN/lerobot-calibrate" "${LEADER_ARGS[@]}"; fi
if [ "$which" != leader ]; then
  echo; echo "Follower recalibrated. The monitor's limits (src/monitor/hold.h JOINT) come from this file:"
  python3 -c "
import json; c=json.load(open('$cal/robots/so_follower/my_follower.json'))
print('  ' + ', '.join(f\"{m} {v['range_min']}..{v['range_max']}\" for m,v in c.items()))"
  echo "Update JOINT (keep wrist_roll inside its physical stops), then: scripts/flash.sh monitor"
fi
