#!/usr/bin/env bash
# Teleoperate the follower from the leader. Extra arguments go to lerobot-teleoperate.
#   scripts/teleop.sh
set -e; source "$(dirname "$0")/env.sh"
need "$FOLLOWER" "$LEADER"
echo "Pico firmware: $(pico_mode)"
exec "$BIN/lerobot-teleoperate" "${FOLLOWER_ARGS[@]}" "${LEADER_ARGS[@]}" "$@"
