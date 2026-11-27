# Shared settings, sourced by the other scripts.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FOLLOWER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B90103276-if00
LEADER=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A001234-if00
FOLLOWER_ARGS=(--robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower)
LEADER_ARGS=(--teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id=my_leader)
BIN="$ROOT/.venv/bin"

need() { for p in "$@"; do [ -e "$p" ] || { echo "missing: $p (plugged in? powered?)"; exit 1; }; done; }

# Prints "monitor", "passthrough" or "none" from the Pico's once-a-second status line.
pico_mode() {
  local p; p=$(ls /dev/serial/by-id/*Pico* 2>/dev/null | head -1)
  [ -n "$p" ] || { echo none; return; }
  local line; line=$(timeout 2.5 head -1 "$p" 2>/dev/null)
  case "$line" in *BLOCKED*) echo monitor ;; *H-\>S*) echo passthrough ;; *) echo none ;; esac
}
