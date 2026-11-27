#!/usr/bin/env bash
# Run the broker in this terminal (as root) until Ctrl-C. In another terminal:
#   FOLLOWER=/tmp/so101-follower scripts/teleop.sh
set -e; source "$(dirname "$0")/env.sh"
need "$FOLLOWER"
lib="$ROOT/broker/libpolicy.so"
for f in src/monitor/policy.h src/monitor/hold.h broker/policy_shim.cpp; do
  [ "$lib" -nt "$ROOT/$f" ] || { g++ -O2 -shared -fPIC -o "$lib" "$ROOT/broker/policy_shim.cpp"; break; }
done
log="$ROOT/results/broker-$(date +%Y%m%d-%H%M%S).log"
echo "Pico firmware: $(pico_mode)   log: $log" | tee "$log"
sudo "$BIN/python" -u "$ROOT/broker/broker.py" "$FOLLOWER" --link /tmp/so101-follower --owner "$USER" 2>&1 | tee -a "$log"
