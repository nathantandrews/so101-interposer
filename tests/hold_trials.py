"""Evaluation trials through the MONITOR firmware. Follower only, 12 V on, Pico on USB.

  .venv/bin/python tests/hold_trials.py /dev/serial/by-id/<follower>

1. Attacks (torque off, no motion): --attacks runs of the 18 limit writes from vuln_limits --separate,
   plus a Torque_Limit rewrite on every joint. Each write is read back. Registers are restored after.
2. Wrist trials (ONLY wrist_roll gets torque; the rest stays limp, so keep the arm folded):
   - sweep: slow full sweeps of the allowed range, edges included. Expect 0 holds.
   - clamp: goal exactly SLACK past an edge. Expect no hold, wrist stops at the edge.
   - hold:  goal past the edge by SLACK+1, SLACK+100, or to the encoder end. Expect a hold
            (Pico counter), the wrist staying put, then normal tracking after safe goals.
   These goals lie beyond the wrist's physical stops (about 172 and 3976). If the monitor ever let one
   through, the script cuts torque as soon as the wrist moves more than FAIL_MOVE ticks.
Hand on the E-stop throughout. Results go to results/<time>-trials/result.json."""
import argparse, glob, json, math, re, sys, threading, time
from datetime import datetime
from pathlib import Path
from vuln_limits import CAL_FILE, NAMES, check_file, make_bus, restore, separate_writes, snapshot

W, WID = "wrist_roll", 5
LO, HI, SLACK = 322, 3826, 200         # wrist_roll in src/monitor/hold.h
SPEED, DT = 400, 0.02                  # ticks/s (~35 deg/s), 50 Hz goals like teleop
EDGE_GAP, STAY_TOL, TRACK_TOL, MAX_LAG, FAIL_MOVE = 40, 25, 30, 200, 60


class Abort(Exception):
    pass


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(c - h, 4), round(c + h, 4)]


class Pico:
    """Reads the monitor's once-a-second status line in the background."""
    RX = re.compile(r"BLOCKED (\d+).*HOLD (\d) n=(\d+) joint=(\d+) drop=(\d+) fail=(\d+)")

    def __init__(self):
        import serial
        ports = glob.glob("/dev/serial/by-id/*Pico*")
        if not ports:
            raise SystemExit("Pico not on USB")
        self.s, self.last, self.stop = serial.Serial(ports[0], 115200, timeout=0.3), None, False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.stop:
            m = self.RX.search(self.s.readline().decode(errors="replace"))
            if m:
                b, on, n, j, d, f = map(int, m.groups())
                self.last = (time.monotonic(), dict(blocked=b, hold=on, n=n, joint=j, drop=d, fail=f))

    def after(self, t, timeout=3.5):
        """First status line that arrived after time t."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.last and self.last[0] > t:
                return self.last[1]
            time.sleep(0.05)
        raise Abort("no status line from the Pico (is it running the monitor firmware?)")


class Wrist:
    def __init__(self, bus):
        self.bus = bus

    def pos(self):
        return int(self.bus.read("Present_Position", W, normalize=False, num_retry=3))

    def goal(self, g):
        self.bus.sync_write("Goal_Position", {W: int(g)}, normalize=False)

    def ramp(self, target, log=None):
        """Software-limited speed. Aborts if the wrist falls far behind its goal."""
        start = self.pos()
        steps = max(1, int(abs(target - start) / (SPEED * DT)))
        for i in range(1, steps + 1):
            g = round(start + (target - start) * i / steps)
            self.goal(g)
            time.sleep(DT)
            p = self.pos()
            if log is not None:
                log.append(g)
            if abs(p - g) > MAX_LAG:
                raise Abort(f"wrist lag: goal {g}, at {p}")
        return self.settle(target)

    def settle(self, g, secs=0.5):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.goal(g)
            time.sleep(DT)
        return self.pos()

    def push(self, g, secs=1.0, guard=None):
        """Send the same goal at 50 Hz for secs; return the wrist positions seen meanwhile.
        guard = start position: abort (torque goes off in main) if the wrist runs away from it."""
        seen, end = [], time.monotonic() + secs
        while time.monotonic() < end:
            self.goal(g)
            time.sleep(DT)
            seen.append(self.pos())
            if guard is not None and abs(seen[-1] - guard) > FAIL_MOVE:
                raise Abort(f"MONITOR LET A GOAL THROUGH: goal {g}, wrist moved {guard} -> {seen[-1]}")
        return seen


def attacks(bus, runs, pico):
    rows, torque_rows = [], []
    base = snapshot(bus)
    t0 = time.monotonic()
    c0 = pico.after(t0 - 5)
    for r in range(runs):
        print(f"\n-- attack run {r + 1}/{runs}")
        bus.disable_torque(num_retry=5)
        batch = []
        separate_writes(bus, base, batch)
        for m in NAMES:
            old = int(bus.read("Torque_Limit", m, normalize=False, num_retry=5))
            val = old - 1 if old > 0 else 1
            err = None
            try:
                bus.write("Torque_Limit", m, val, normalize=False)
            except (ConnectionError, RuntimeError) as e:
                err = str(e)
            now = int(bus.read("Torque_Limit", m, normalize=False, num_retry=5))
            out = ("changed" if now == val else "unchanged" if now == old else "other") + \
                  ("/no-reply" if err else "/ack")
            print(f"{m:14} {'Torque_Limit':20} {old:>5} -> tried {val:<5} now {now:<5} {out}")
            torque_rows.append({"run": r, "joint": m, "register": "Torque_Limit", "before": old,
                                "attempted": val, "after": now, "outcome": out, "error": err})
            if now != old:
                bus.write("Torque_Limit", m, old, normalize=False, num_retry=3)
        for b in batch:
            b["run"] = r
        rows += batch
        left = restore(bus, base)
        if left:
            raise Abort("restore failed, rerun vuln_limits.py --restore:\n  " + "\n  ".join(left))
    c1 = pico.after(time.monotonic())
    allr = rows + torque_rows
    blocked = sum(x["after"] == x["before"] for x in allr)
    return {"limit_writes": rows, "torque_writes": torque_rows, "tried": len(allr), "blocked": blocked,
            "blocked_ci95": wilson(blocked, len(allr)), "pico_blocked_delta": c1["blocked"] - c0["blocked"]}


def sweeps(wr, n, pico):
    goals = []
    c0 = pico.after(time.monotonic())
    for i in range(n):
        print(f"sweep {i + 1}/{n}")
        wr.ramp(HI, goals)
        wr.ramp(LO, goals)
    wr.ramp((LO + HI) // 2, goals)
    c1 = pico.after(time.monotonic())
    holds = c1["n"] - c0["n"]
    return {"sweeps": n, "goals_sent": len(goals), "min_goal": min(goals), "max_goal": max(goals),
            "holds": holds, "dropped": c1["drop"] - c0["drop"]}


def clamp_trial(wr, side, pico):
    edge, g = (HI, HI + SLACK) if side > 0 else (LO, LO - SLACK)
    p0 = wr.ramp(edge - side * EDGE_GAP)
    c0 = pico.after(time.monotonic())
    end = wr.push(g, 1.0, guard=p0)[-1]
    c1 = pico.after(time.monotonic())
    ok = c1["n"] == c0["n"] and abs(end - edge) <= TRACK_TOL
    return {"side": side, "goal": g, "final": end, "holds": c1["n"] - c0["n"], "pass": ok}


def hold_trial(wr, side, kind, pico):
    edge = HI if side > 0 else LO
    past = {"slack+1": SLACK + 1, "slack+100": SLACK + 100, "end": None}[kind]
    g = min(4095, max(0, edge + side * past)) if past else (4095 if side > 0 else 0)
    p0 = wr.ramp(edge - side * EDGE_GAP)
    c0 = pico.after(time.monotonic())
    seen = wr.push(g, 1.0, guard=p0)
    moved = max(abs(p - p0) for p in seen)
    c1 = pico.after(time.monotonic())
    held = c1["n"] == c0["n"] + 1 and c1["joint"] == WID and c1["fail"] == c0["fail"]
    wr.push(p0, 0.7)                                   # safe goals: release after 500 ms
    target = p0 - side * 300
    final = wr.ramp(target)
    c2 = pico.after(time.monotonic())
    released = abs(final - target) <= TRACK_TOL and c2["hold"] == 0
    ok = held and moved <= STAY_TOL and released
    print(f"hold {'+' if side > 0 else '-'} {kind:7} goal {g:4}  counted={held} moved={moved:3} "
          f"released={released}  {'PASS' if ok else 'FAIL'}")
    return {"side": side, "kind": kind, "goal": g, "start": p0, "max_moved": moved, "hold_counted": held,
            "dropped": c1["drop"] - c0["drop"], "released": released, "final": final, "pass": ok}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--attacks", type=int, default=5, help="attack runs (24 writes each)")
    ap.add_argument("--sweeps", type=int, default=3)
    ap.add_argument("--clamps", type=int, default=4, help="clamp trials per side")
    ap.add_argument("--holds", type=int, default=20)
    ap.add_argument("--out", default="results")
    a = ap.parse_args()

    pico = Pico()
    bus = make_bus(a.port)
    bus.connect()
    res = {"time": datetime.now().isoformat(), "port": a.port, "limits": {"lo": LO, "hi": HI, "slack": SLACK},
           "speed_ticks_s": SPEED}
    out = Path(a.out) / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-trials")
    wr = Wrist(bus)
    try:
        before = snapshot(bus)
        diff = check_file(before, CAL_FILE)
        if diff:
            print("Servo registers do not match the calibration file; stopping:\n  " + "\n  ".join(diff))
            return 1
        out.mkdir(parents=True)
        (out / "backup.json").write_text(json.dumps(before, indent=1))
        res["pico_start"] = pico.after(time.monotonic())
        print("Pico:", res["pico_start"])

        if a.attacks:
            input(f"\n{a.attacks} attack runs, torque OFF, no motion. Arm folded? Enter to start. ")
            res["attacks"] = attacks(bus, a.attacks, pico)
            r = res["attacks"]
            print(f"\nATTACKS: {r['blocked']}/{r['tried']} blocked, 95% CI {r['blocked_ci95']}")

        p = wr.pos()
        if not LO <= p <= HI:
            raise Abort(f"wrist at {p}, outside {LO}..{HI}: turn it toward center by hand")
        input("\nOnly the WRIST gets torque and turns. Arm folded, hands clear, hand on E-stop. Enter. ")
        bus.disable_torque(num_retry=5)
        wr.goal(p)
        bus.enable_torque(W, num_retry=5)

        res["sweep"] = sweeps(wr, a.sweeps, pico)
        print("SWEEP:", res["sweep"])

        res["clamp"] = [clamp_trial(wr, s, pico) for _ in range(a.clamps) for s in (1, -1)]
        for c in res["clamp"]:
            print("clamp", c)

        kinds = ["slack+1", "slack+100", "end"]
        res["hold"] = [hold_trial(wr, 1 if i % 2 == 0 else -1, kinds[(i // 2) % 3], pico) for i in range(a.holds)]
        wr.ramp((LO + HI) // 2)
    except Abort as e:
        res["aborted"] = str(e)
        print("\nABORTED:", e)
    finally:
        try:
            bus.disable_torque(num_retry=5)
        except Exception as e:
            print("could not disable torque:", e)
        try:
            res["pico_end"] = pico.after(time.monotonic())
        except Abort:
            pass
        pico.stop = True
        left = check_file(snapshot(bus), CAL_FILE)
        res["calibration_after"] = left or "matches file"
        bus.disconnect(disable_torque=False)
        if out.exists():
            h, c = res.get("hold", []), res.get("clamp", [])
            res["summary"] = {
                "attacks_blocked": res.get("attacks", {}).get("blocked"),
                "attacks_tried": res.get("attacks", {}).get("tried"),
                "holds_passed": sum(x["pass"] for x in h), "holds_tried": len(h),
                "holds_ci95": wilson(sum(x["pass"] for x in h), len(h)),
                "clamps_passed": sum(x["pass"] for x in c), "clamps_tried": len(c),
                "false_holds_in_sweep": res.get("sweep", {}).get("holds")}
            (out / "result.json").write_text(json.dumps(res, indent=1))
            print("\nSUMMARY:", json.dumps(res["summary"]))
            print("calibration after:", res["calibration_after"])
            print("Saved", out / "result.json")
    return 1 if "aborted" in res else 0


if __name__ == "__main__":
    sys.exit(main())
