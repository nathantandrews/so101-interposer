"""Adaptive attacker: the protected-register writes in every packet form a smarter attacker would try.
Raw packets (no LeRobot), follower only, torque OFF, nothing moves.

  .venv/bin/python tests/vuln_adaptive.py /dev/serial/by-id/<follower> [--runs 3] [--label monitor]

Each trial tries to change one register and reads it back: changed = the attack got through.
Values are harmless (a limit tightened by 1 tick, torque limit or goal speed off by 1), except broadcast,
which can only send one value to every servo and so uses the real attack (Max_Position_Limit = 4095).
Anything that changes is written straight back (works whenever writes get through, e.g. pass-through).
If something can't be restored: flash pass-through, then
  .venv/bin/python tests/vuln_adaptive.py <port> --restore results/<run>-adaptive/registers.json
Not tried on the arm: RESET / factory reset (would wipe IDs and calibration if it ever got through);
the policy denies it like every unknown instruction, see tests/test_policy.cpp."""
import argparse, glob, json, re, sys, time
from datetime import datetime
from pathlib import Path

import serial

IDS = range(1, 7)
LOCK, TORQUE, MIN, MAX, OFFSET, TLIM, GTIME, GSPEED, POS = 55, 40, 9, 11, 31, 48, 44, 46, 56
REGS = {"Min_Position_Limit": MIN, "Max_Position_Limit": MAX, "Homing_Offset": OFFSET,
        "Torque_Limit": TLIM, "Goal_Speed": GSPEED}


def pk(i, ins, params=()):
    b = [i, len(params) + 2, ins, *params]
    return bytes([0xFF, 0xFF, *b, (~sum(b)) & 0xFF])


def le(v):
    return [v & 0xFF, v >> 8 & 0xFF]


class Bus:
    def __init__(self, port):
        self.s = serial.Serial(port, 1_000_000, timeout=0.03)

    def send(self, data, drip=0.0):
        self.s.reset_input_buffer()
        if drip:
            for b in data:
                self.s.write(bytes([b]))
                self.s.flush()
                time.sleep(drip)
        else:
            self.s.write(data)
        time.sleep(0.004)
        return self.s.read(256)                 # whatever came back (status packets, or nothing)

    def read(self, i, addr, n, tries=5):
        for _ in range(tries):
            r = self.send(pk(i, 0x02, [addr, n]))
            k = r.find(bytes([0xFF, 0xFF, i]))
            if k >= 0 and len(r) >= k + 6 + n:
                p = r[k:k + 6 + n]
                if (~sum(p[2:-1])) & 0xFF == p[-1]:
                    return list(p[5:5 + n])
            time.sleep(0.01)
        raise IOError(f"id {i}: read {addr}+{n} failed")

    def reg(self, i, addr):
        lo, hi = self.read(i, addr, 2)
        return lo | hi << 8

    def write(self, i, addr, data):
        return self.send(pk(i, 0x03, [addr, *data]))


def snapshot(bus):
    return {i: {name: bus.reg(i, a) for name, a in REGS.items()} for i in IDS}


# ---- attack forms. Each returns [(id, register name, attempted value)] and sends its packet(s). ----
def f_write(bus, i, b):                           # naive control: plain WRITE
    v = b[i]["Max_Position_Limit"] - 1
    bus.write(i, MAX, le(v))
    return [(i, "Max_Position_Limit", v)]


def f_reg_action(bus, i, b):                      # REG_WRITE (deferred) then broadcast ACTION
    v = b[i]["Max_Position_Limit"] - 1
    bus.send(pk(i, 0x04, [MAX, *le(v)]))
    bus.send(pk(0xFE, 0x05))
    return [(i, "Max_Position_Limit", v)]


def f_span_limits(bus, i, b):                     # one WRITE covering Min and Max together
    lo, hi = b[i]["Min_Position_Limit"] + 1, b[i]["Max_Position_Limit"] - 1
    bus.write(i, MIN, le(lo) + le(hi))
    return [(i, "Min_Position_Limit", lo), (i, "Max_Position_Limit", hi)]


def f_span_from_allowed(bus, i, b):               # starts at an allowed register (7 = 0) and runs into 9..12
    cur = bus.read(i, 7, 6)                       # 7 delay, 8 baud, 9-10 min, 11-12 max: all kept but max
    v = b[i]["Max_Position_Limit"] - 1
    bus.write(i, 7, [0, cur[1], cur[2], cur[3], *le(v)])
    return [(i, "Max_Position_Limit", v)]


def f_smuggle(bus, i, b):                         # allowed READ and the attack WRITE in one burst
    v = b[i]["Max_Position_Limit"] - 1
    bus.send(pk(i, 0x02, [POS, 2]) + pk(i, 0x03, [MAX, *le(v)]))
    return [(i, "Max_Position_Limit", v)]


def f_drip(bus, i, b):                            # the attack WRITE one byte at a time, 1 ms apart
    v = b[i]["Max_Position_Limit"] - 1
    bus.send(pk(i, 0x03, [MAX, *le(v)]), drip=0.001)
    return [(i, "Max_Position_Limit", v)]


def f_extra_header(bus, i, b):                    # FF FF FF ...: hoping two parsers frame it differently
    v = b[i]["Max_Position_Limit"] - 1
    bus.send(b"\xff" + pk(i, 0x03, [MAX, *le(v)]))
    return [(i, "Max_Position_Limit", v)]


def f_torque_write(bus, i, b):                    # RAM torque limit, plain WRITE
    v = b[i]["Torque_Limit"] - 1
    bus.write(i, TLIM, le(v))
    return [(i, "Torque_Limit", v)]


PER_JOINT = {"write": f_write, "reg_write+action": f_reg_action, "span_min_max": f_span_limits,
             "span_from_allowed": f_span_from_allowed, "smuggled_after_read": f_smuggle,
             "byte_drip": f_drip, "extra_header_byte": f_extra_header, "torque_write": f_torque_write}


def w_broadcast(bus, b):                          # WRITE to 0xFE: every servo, no replies
    bus.write(0xFE, MAX, le(4095))
    return [(i, "Max_Position_Limit", 4095) for i in IDS]


def w_sync_limits(bus, b):                        # SYNC WRITE to the limit register instead of the goal
    rows = [(i, "Max_Position_Limit", b[i]["Max_Position_Limit"] - 1) for i in IDS]
    bus.send(pk(0xFE, 0x83, [MAX, 2, *[x for i, _, v in rows for x in (i, *le(v))]]))
    return rows


def w_sync_torque(bus, b):
    rows = [(i, "Torque_Limit", b[i]["Torque_Limit"] - 1) for i in IDS]
    bus.send(pk(0xFE, 0x83, [TLIM, 2, *[x for i, _, v in rows for x in (i, *le(v))]]))
    return rows


def w_sync_goal_span(bus, b):                     # Goal_Position sync write stretched over time + speed
    rows, params = [], [42, 6]
    for i in IDS:
        pos, gt = bus.reg(i, POS), bus.reg(i, GTIME)
        v = b[i]["Goal_Speed"] + 1
        params += [i, *le(pos), *le(gt), *le(v)]
        rows.append((i, "Goal_Speed", v))
    bus.send(pk(0xFE, 0x83, params))
    return rows


WIDE = {"broadcast_write": w_broadcast, "sync_write_limits": w_sync_limits,
        "sync_write_torque": w_sync_torque, "sync_goal_span_speed": w_sync_goal_span}


def put_back(bus, base):
    """Write back every register that differs from base. Returns what still differs."""
    for _ in range(3):
        now = snapshot(bus)
        bad = [(i, n) for i in IDS for n in REGS if now[i][n] != base[i][n]]
        if not bad:
            return []
        for i, n in bad:
            bus.write(i, LOCK, [0])
            bus.write(i, REGS[n], le(base[i][n]))
        time.sleep(0.05)
    now = snapshot(bus)
    return [f"id {i} {n}: {now[i][n]} (was {base[i][n]})" for i in IDS for n in REGS if now[i][n] != base[i][n]]


def pico_blocked():
    p = glob.glob("/dev/serial/by-id/*Pico*")
    if not p:
        return None
    try:
        s = serial.Serial(p[0], 115200, timeout=0.5)
        end = time.time() + 3
        while time.time() < end:
            m = re.search(r"BLOCKED (\d+)", s.readline().decode(errors="replace"))
            if m:
                return int(m.group(1))
    except serial.SerialException:
        pass
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="results")
    ap.add_argument("--restore", metavar="REGISTERS_JSON", help="only write back a saved registers.json")
    a = ap.parse_args()
    bus = Bus(a.port)
    if a.restore:
        want = {int(k): v for k, v in json.loads(Path(a.restore).read_text()).items()}
        left = put_back(bus, want)
        print("RESTORED" if not left else "NOT RESTORED: " + "; ".join(left))
        return bool(left)
    base = snapshot(bus)
    out = Path(a.out) / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-adaptive")
    out.mkdir(parents=True)
    (out / "registers.json").write_text(json.dumps(base, indent=1))
    print("Registers saved to", out)
    input("Torque goes OFF, nothing moves. Enter to start (Ctrl-C to quit). ")
    pico0 = pico_blocked()
    locks = {i: bus.read(i, LOCK, 1)[0] for i in IDS}
    for i in IDS:
        bus.write(i, TORQUE, [0])
        bus.write(i, LOCK, [0])                 # give the attacker the best case: EEPROM unlocked

    rows, left = [], []
    try:
        for r in range(a.runs):
            print(f"\n-- run {r + 1}/{a.runs}")
            todo = [(name, f, i) for name, f in PER_JOINT.items() for i in IDS] + [(name, f, None) for name, f in WIDE.items()]
            for name, f, i in todo:
                tried = f(bus, i, base) if i else f(bus, base)
                time.sleep(0.03)                # EEPROM write time
                got = []
                for j, reg, v in tried:
                    now = bus.reg(j, REGS[reg])
                    got.append({"id": j, "register": reg, "before": base[j][reg], "attempted": v, "after": now,
                                "changed": now != base[j][reg]})
                hit = any(g["changed"] for g in got)
                rows.append({"run": r, "form": name, "target": i or "all", "changed": hit, "checks": got})
                print(f"{name:22} {'id ' + str(i) if i else 'all ids':8} {'CHANGED' if hit else 'blocked'}")
                if hit:
                    left = put_back(bus, base)
                    if left:
                        raise RuntimeError("could not restore: " + "; ".join(left))
    finally:
        left = put_back(bus, base)
        for i in IDS:
            bus.write(i, LOCK, [locks[i]])
        pico1 = pico_blocked()
        forms = {}
        for x in rows:
            k = forms.setdefault(x["form"], [0, 0])
            k[0] += x["changed"]
            k[1] += 1
        summary = {"trials": len(rows), "changed": sum(x["changed"] for x in rows),
                   "blocked": sum(not x["changed"] for x in rows),
                   "by_form": {k: {"changed": v[0], "tried": v[1]} for k, v in forms.items()},
                   "pico_blocked_delta": pico1 - pico0 if pico0 is not None and pico1 is not None else None,
                   "restore_left": left}
        (out / "result.json").write_text(json.dumps(
            {"time": datetime.now().isoformat(), "label": a.label, "port": a.port, "runs": a.runs,
             "summary": summary, "trials": rows}, indent=1))
        print("\nby form (changed / tried):")
        for k, v in forms.items():
            print(f"  {k:22} {v[0]:3} / {v[1]}")
        print(f"\n{summary['trials']} trials: {summary['changed']} changed a register, {summary['blocked']} blocked")
        print("restore:", "all registers match the start" if not left else "FAILED: " + "; ".join(left))
        print("Saved", out / "result.json")


if __name__ == "__main__":
    sys.exit(main())
