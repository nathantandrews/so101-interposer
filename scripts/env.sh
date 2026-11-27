# Shared settings, sourced by the other scripts.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FOLLOWER=${FOLLOWER:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90103276-if00}
LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A001234-if00
FOLLOWER_ARGS=(--robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower)
LEADER_ARGS=(--teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id=my_leader)
BIN="$ROOT/.venv/bin"

need() {
  for p in "$@"; do
    [ -e "$p" ] && continue
    case "$p" in /tmp/so101-*) echo "missing: $p (start the broker first: scripts/broker.sh, in another terminal)" ;;
                 *) echo "missing: $p (plugged in? powered?)" ;; esac
    exit 1
  done
}

# Prints "monitor", "passthrough" or "none" from the Pico's once-a-second status line.
pico_mode() {
  "$BIN/python" - <<'PY' 2>/dev/null || echo none
import glob, time, serial
p = glob.glob("/dev/serial/by-id/*Pico*")
mode = "none"
if p:
    s = serial.Serial(p[0], 115200, timeout=0.5)
    end = time.time() + 3.5                  # status prints once a second; skip partial lines
    while time.time() < end:
        line = s.readline().decode(errors="replace")
        if line.endswith("\n") and "H->S" in line:
            mode = "monitor" if "BLOCKED" in line else "passthrough"
            break
print(mode)
PY
}
