"""Safe on-arm check: rewrites each register with the value it already holds (nothing changes
even if blocking fails). Blocked = no reply; PASSED = the servo acknowledged it.
Usage: python tests/probe_block.py /dev/serial/by-id/<follower>"""
import serial, sys
def pk(i, b):
    b = [i, len(b) + 1] + list(b); return bytes([255, 255] + b + [(~sum(b)) & 255])
def xfer(s, i, b):
    s.reset_input_buffer(); s.write(pk(i, b)); r = s.read(6 + (b[2] if b[0] == 2 else 0))
    return r if len(r) >= 6 and (~sum(r[2:-1])) & 255 == r[-1] else None
bad = 0
with serial.Serial(sys.argv[1], 1000000, timeout=0.05) as s:
    for i in range(1, 7):
        for name, addr, n, want in (("Min_Position_Limit", 9, 2, False), ("Max_Position_Limit", 11, 2, False),
                                    ("Homing_Offset", 31, 2, False), ("Torque_Limit", 48, 2, False),
                                    ("Operating_Mode", 33, 1, True)):
            r = xfer(s, i, [2, addr, n])
            if not r: print(i, name, "READ FAILED"); bad += 1; continue
            passed = xfer(s, i, [3, addr, *r[5:5 + n]]) is not None
            ok = passed == want; bad += not ok
            print(i, f"{name:19}", "PASSED " if passed else "blocked", "ok" if ok else "<-- UNEXPECTED")
        alive = xfer(s, i, [1]) is not None; bad += not alive
        if not alive: print(i, "ping failed after test")
print("ALL AS EXPECTED" if not bad else f"{bad} PROBLEMS")
