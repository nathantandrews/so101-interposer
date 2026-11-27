#!/usr/bin/env bash
# Broker baseline: the same policy as the Pico, enforced on the host by a root-owned broker.
#   scripts/broker_demo.sh          (asks for sudo; arm folded, torque goes off, nothing moves)
# Run once with each firmware (scripts/flash.sh passthrough / monitor). Writes results/<time>-broker-<fw>/.
#  1. root starts the broker on the real port; LeRobot gets /tmp/so101-follower
#  2. unprivileged attacker (this user without the uucp group): open the real port, kill the broker,
#     then the 18 limit writes through the broker
#  3. root attacker: kills the broker and sends the same 18 writes straight to the real port
set -e; source "$(dirname "$0")/env.sh"
need "$FOLLOWER"
fw=$(pico_mode); out="$ROOT/results/$(date +%Y%m%d-%H%M%S)-broker-$fw"; mkdir -p "$out"
link=/tmp/so101-follower; cal="$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
log() { echo "$*" | tee -a "$out/summary.txt"; }
log "firmware: $fw   $(date)"
lib="$ROOT/broker/libpolicy.so"      # rebuild whenever the Pico's policy or envelope changed
for f in src/monitor/policy.h src/monitor/hold.h broker/policy_shim.cpp; do
  [ "$lib" -nt "$ROOT/$f" ] || { g++ -O2 -shared -fPIC -o "$lib" "$ROOT/broker/policy_shim.cpp"; break; }
done

sudo -v
sudo "$BIN/python" "$ROOT/broker/broker.py" "$FOLLOWER" --link "$link" --owner "$USER" > "$out/broker.log" 2>&1 &
for _ in $(seq 50); do [ -e "$link" ] && break; sleep 0.1; done
bpid=$(pgrep -n -f "python.*broker/broker.py")     # the python process, not its sudo parent
[ -n "$bpid" ] && [ -e "$link" ] || { echo "broker did not start:"; cat "$out/broker.log"; exit 1; }
log "broker running as root, pid $bpid: $link -> real port"

# unprivileged: same user, no supplementary groups (so no uucp = no serial devices), no capabilities
unpriv() { sudo setpriv --reuid="$(id -u)" --regid="$(id -g)" --clear-groups --inh-caps=-all -- "$@"; }
log ""; log "== unprivileged attacker (uid $(id -u), groups: none)"
r=$(unpriv "$BIN/python" -c "open('$FOLLOWER','rb')" 2>&1 | tail -1) || true; log "open real port:   ${r:-OPENED}"
r=$(unpriv kill -9 "$bpid" 2>&1) || true;                                      log "kill broker:      ${r:-KILLED}"
yes '' | unpriv "$BIN/python" "$ROOT/tests/vuln_limits.py" "$link" --separate --label "broker-unpriv-$fw" \
  --cal-file "$cal" --out "$out" | tee "$out/unpriv.txt" | grep -E "writes tried|Limits changed" || true
log "18 limit writes through broker: $(grep 'writes tried' "$out/unpriv.txt")"
log "broker: $(tail -1 "$out/broker.log")"

log ""; log "== root attacker"
sudo kill "$bpid"; sleep 0.5
log "kill broker:      $(pgrep -f broker/broker.py >/dev/null && echo FAILED || echo KILLED)"
yes '' | sudo "$BIN/python" "$ROOT/tests/vuln_limits.py" "$FOLLOWER" --separate --label "broker-root-$fw" \
  --cal-file "$cal" --out "$out" | tee "$out/root.txt" | grep -E "writes tried|Limits changed|restore" || true
log "18 limit writes straight to the port: $(grep 'writes tried' "$out/root.txt")"
sudo chown -R "$(id -u):$(id -g)" "$out"
log ""; log "saved $out"
