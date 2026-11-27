"""Host-side broker: the software baseline the interposer is compared against.

It owns the real servo port and offers LeRobot a pseudo-terminal instead. Every host packet goes through
the SAME policy and envelope the Pico runs (src/monitor/policy.h and hold.h, via libpolicy.so):
blocked writes are dropped (the host times out, as with the monitor), out-of-envelope goals are dropped
until RELEASE_MS of safe goals, small overshoots are clamped. Servo replies are relayed unchanged.
Difference from the monitor: no position read / hold write; a dropped goal leaves the servo on its last
safe goal, which is what keeps the arm still in the monitor too.

Run as root, so an unprivileged process can neither kill it nor open the real port:
  sudo .venv/bin/python broker/broker.py /dev/serial/by-id/<follower> --link /tmp/so101-follower --owner nathan
Then point LeRobot at /tmp/so101-follower."""
import argparse, ctypes, os, pwd, signal, sys, threading, time, tty
from pathlib import Path

import serial

HERE = Path(__file__).resolve().parent
lib = ctypes.CDLL(str(HERE / "libpolicy.so"))
lib.verdict_name.restype = ctypes.c_char_p
lib.hold_count.restype = ctypes.c_uint32
lib.hold_goal.argtypes = [ctypes.c_int, ctypes.c_uint32]
T0 = time.monotonic()


class Counters:
    fwd = blocked = bad = dropped = clamped = replies = 0
    last = "-"


def packets(buf):
    """Yield complete packets from a bytearray, consuming them. Junk before FF FF is discarded."""
    while True:
        i = buf.find(b"\xff\xff")
        if i < 0:
            del buf[:max(0, len(buf) - 1)]
            return
        del buf[:i]
        if len(buf) < 4:
            return
        if buf[2] == 0xFF:                      # FF FF FF: resync one byte later
            del buf[:1]
            continue
        n = buf[3] + 4
        if len(buf) < n:
            return
        pk = bytes(buf[:n])
        del buf[:n]
        yield pk


def host_to_servo(master, ser, c):
    buf = bytearray()
    while True:
        try:
            data = os.read(master, 4096)
        except OSError:                         # client closed the pty: wait for the next one
            time.sleep(0.05)
            continue
        buf += data
        for pk in packets(buf):
            if (~sum(pk[2:-1])) & 0xFF != pk[-1]:
                c.bad += 1
                continue
            v = lib.check_packet(pk, len(pk))
            if v != 0:
                c.blocked += 1
                c.last = f"{lib.verdict_name(v).decode()} id={pk[2]} ins=0x{pk[4]:02X} addr={pk[5] if len(pk) > 6 else 0}"
                continue
            if pk[4] == 0x83:
                safe = lib.unsafe_joint(pk, len(pk)) == 0
                now = int((time.monotonic() - T0) * 1000) & 0xFFFFFFFF
                if not lib.hold_goal(int(safe), now):
                    c.dropped += 1
                    continue
                b = ctypes.create_string_buffer(pk, len(pk))
                lib.clamp_packet(b, len(pk))
                if b.raw != pk:
                    c.clamped += 1
                pk = b.raw
            ser.write(pk)
            c.fwd += 1


def servo_to_host(master, ser, c):
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            try:
                os.write(master, data)
                c.replies += len(data)
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="real follower port")
    ap.add_argument("--link", default="/tmp/so101-follower", help="path LeRobot opens")
    ap.add_argument("--owner", help="user allowed to open the pty (default: the pty stays root-only)")
    a = ap.parse_args()

    ser = serial.Serial(a.port, 1_000_000, timeout=0.05)
    master, slave = os.openpty()               # broker keeps slave open, so clients can come and go
    tty.setraw(slave)
    name = os.ttyname(slave)
    if a.owner:
        u = pwd.getpwnam(a.owner)
        os.chown(name, u.pw_uid, u.pw_gid)
        os.chmod(name, 0o600)
    link = Path(a.link)
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(name)

    def bye(*_):
        if link.is_symlink():
            link.unlink()
        sys.exit(0)
    signal.signal(signal.SIGTERM, bye)
    signal.signal(signal.SIGINT, bye)

    c = Counters()
    for f in (host_to_servo, servo_to_host):
        threading.Thread(target=f, args=(master, ser, c), daemon=True).start()
    print(f"broker pid {os.getpid()} uid {os.getuid()}: {a.link} -> {name} -> {a.port}", flush=True)
    while True:
        time.sleep(1)
        print(f"fwd {c.fwd}  replies {c.replies}B  bad {c.bad}  BLOCKED {c.blocked} last={c.last}  "
              f"HOLD {lib.hold_on()} n={lib.hold_count()} drop={c.dropped} clamp={c.clamped}", flush=True)


if __name__ == "__main__":
    main()
