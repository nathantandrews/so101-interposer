"""Phase 1 video demo. MOVES shoulder_pan (slowly, ~26 degrees, toward mid-range). Follower only.
A test limit is set inside the calibrated range. The same slow goal past it is sent twice:
  A) test limit in place   B) after LeRobot's write_calibration() opens the limit to 0..4095
The goal always stays inside the calibrated range and the homing offset never changes.
Pass-through or direct only. Fold the arm at rest, clear its swing, hand on the power switch.

  .venv/bin/python tests/vuln_motion.py /dev/serial/by-id/<follower> --label passthrough
Recovery: .venv/bin/python tests/vuln_limits.py <port> --restore results/<run>/backup.json"""
import argparse, json, sys, time
from datetime import datetime
from pathlib import Path
from vuln_limits import CAL_FILE, check_file, make_bus, restore, snapshot

J = "shoulder_pan"
TO_LIMIT, TO_GOAL, MARGIN = 200, 300, 200   # ticks from start / from calibrated ends (11.4 ticks = 1 degree)
STEP, DT = 5, 0.05                          # 100 ticks/s = about 9 degrees/s
TORQUE, MAX_LAG, MAX_CURRENT = 500, 120, 150  # torque 50%; current in servo units (nominally 6.5 mA)
HELD, PASSED = 20, 60                       # ticks past the limit that count as held / passed


class Abort(Exception):
    pass


def pos(bus):
    return int(bus.read("Present_Position", J, normalize=False, num_retry=2))


def ramp(bus, start, goal, box, log, phase, t0, lag_ok=False):
    """Step the goal in software so speed doesn't depend on servo settings. box = allowed positions."""
    d = 1 if goal >= start else -1
    for g in [*range(start, goal, d * STEP), goal, *[goal] * 20]:       # ends with a 1 s dwell
        bus.write("Goal_Position", J, g, normalize=False, num_retry=2)
        time.sleep(DT)
        p = pos(bus)
        c = int(bus.read("Present_Current", J, normalize=False, num_retry=2))
        log.append([phase, round(time.monotonic() - t0, 3), g, p, c])
        if not box[0] <= p <= box[1]:
            raise Abort(f"position {p} left the allowed band {box}")
        if abs(c) > MAX_CURRENT:
            raise Abort(f"current {c} too high (is something blocking the arm?)")
        if not lag_ok and abs(p - g) > MAX_LAG:
            raise Abort(f"joint is {abs(p - g)} ticks behind its goal")
    return p


def torque_on(bus):
    bus.write("Goal_Position", J, pos(bus), normalize=False)            # goal = here, so no jump
    bus.enable_torque(J, num_retry=2)


def main():
    from lerobot.motors import MotorCalibration
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--label", default="", help="direct / passthrough, saved in motion.json")
    ap.add_argument("--cal-file", default=CAL_FILE)
    ap.add_argument("--out", default="results")
    a = ap.parse_args()

    bus = make_bus(a.port)
    bus.connect()
    try:
        before = snapshot(bus)
        if diff := check_file(before, a.cal_file):
            print("Servo registers do not match the follower calibration file; nothing changed:\n  "
                  + "\n  ".join(diff))
            return 1
        if mode := int(bus.read("Operating_Mode", J, normalize=False)):
            print(f"{J} is in operating mode {mode}, not position mode; nothing changed")
            return 1
        input("Torque will go OFF. Fold the arm at rest and support it, then press Enter. ")
        bus.disable_torque(num_retry=5)
        input("Place the arm so it can swing ~30 degrees toward mid-range, clear of everything. Enter when resting. ")

        lo, hi = before[J]["range_min"], before[J]["range_max"]
        s = pos(bus)
        d = 1 if s < (lo + hi) // 2 else -1
        limit, goal = s + d * TO_LIMIT, s + d * TO_GOAL
        if not all(lo + MARGIN <= x <= hi - MARGIN for x in (s, goal)):
            print(f"start {s} / goal {goal} too close to the calibrated ends {lo}..{hi}; nothing changed")
            return 1
        box = (min(s, goal) - 60, max(s, goal) + 60)
        print(f"start {s}, test limit {limit}, goal {goal} (calibrated {lo}..{hi})")

        out = Path(a.out) / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-motion")
        out.mkdir(parents=True)
        (out / "backup.json").write_text(json.dumps(before, indent=1))
        old_torque = int(bus.read("Torque_Limit", J, normalize=False))
        if input("Start filming. Type GO to move the arm: ").strip() != "GO":
            return 1

        log, res, why, t0 = [], {}, None, time.monotonic()
        def cal(mn, mx):
            return {J: MotorCalibration(1, 0, before[J]["homing_offset"], mn, mx)}
        left = ["restore did not run"]
        try:
            bus.write("Torque_Limit", J, TORQUE, normalize=False, num_retry=2)
            for phase, rng in (("A_limit_set", (lo, limit) if d > 0 else (limit, hi)), ("B_limit_opened", (0, 4095))):
                bus.disable_torque(num_retry=5)                         # opens the EEPROM lock
                bus.write_calibration(cal(*rng), cache=False)
                got = snapshot(bus)[J]
                if (got["range_min"], got["range_max"]) != rng:
                    raise Abort(f"limits read back {got}, wanted {rng}")
                torque_on(bus)
                p = pos(bus)
                end = ramp(bus, p, goal, box, log, phase, t0, lag_ok=phase.startswith("A"))
                past = d * (end - limit)
                res[phase] = {"end": end, "ticks_past_limit": past}
                print(f"{phase}: ended at {end}, {past:+d} ticks relative to the test limit")
                ramp(bus, end, s, box, log, phase + "_return", t0, lag_ok=phase.startswith("A"))
        except (Abort, ConnectionError, RuntimeError) as e:
            why = str(e)
            print("STOPPED:", why)
        finally:
            for _ in range(3):
                try:
                    bus.disable_torque(num_retry=5)
                    bus.write("Torque_Limit", J, old_torque, normalize=False, num_retry=3)
                    break
                except (ConnectionError, RuntimeError) as e:
                    print("  torque off / torque limit:", e)
            try:
                left = restore(bus, before)
            finally:
                print("restore:", "verified, all registers match the backup" if not left else
                      "FAILED, run vuln_limits.py --restore " + str(out / "backup.json") + "\n  " + "\n  ".join(left))
                (out / "motion.json").write_text(json.dumps(
                    {"time": datetime.now().isoformat(), "label": a.label, "port": a.port, "joint": J, "start": s,
                     "test_limit": limit, "goal": goal, "calibrated": [lo, hi], "phases": res, "stopped": why,
                     "restore_left": left, "log_columns": ["phase", "t", "goal", "position", "current"], "log": log}))
        ok = len(res) == 2 and abs(res["A_limit_set"]["ticks_past_limit"]) <= HELD \
            and res["B_limit_opened"]["ticks_past_limit"] >= PASSED
        print("DEMONSTRATED: held by the limit, then passed it once opened" if ok else "NOT demonstrated, see motion.json")
        print("Saved", out / "motion.json")
        return bool(left) or not ok
    finally:
        bus.disconnect()


if __name__ == "__main__":
    sys.exit(main())
