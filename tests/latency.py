"""Per-packet latency through the interposer from a logic-analyzer capture.

Capture (fx2lafw clone; host-side and servo-side bus lines on D1 and D2, GND to bus ground):
  sigrok-cli -d fx2lafw --config samplerate=12m --channels D0,D1,D2,D3,D4,D5,D6,D7 \
             --time 10000 -O binary -o cap.bin
  .venv/bin/python tests/latency.py cap.bin [--rate 12e6] [--json out.json]

Decodes 1 Mbaud 8N1 on both lines, splits Feetech packets (FF FF ID LEN ... CHK), and matches every
packet to its copy on the other line. Whichever line a packet appears on first is its origin; requests
come from the host, so the line where instruction packets originate is the host side.
  delay      = copy's first start bit - original's first start bit
  turnaround = copy's first start bit - original's last stop bit (time the Pico holds the packet)
Hold events (a goal sync write that never appears on the servo side) are reported separately with the
time to the monitor's own sync read (FF FF FE .. 82 28 12 ..) and its hold goal write."""
import argparse, json, sys
import numpy as np

BAUD = 1_000_000


def uart(bits, rate):
    """Bytes with start/end times (s) from one line sampled at rate. 8N1, LSB first, idle high."""
    spb = rate / BAUD
    k = max(3, int(spb / 4) | 1)                 # majority filter drops probe glitches shorter than ~k/2 samples
    bits = (np.convolve(bits.astype(np.int16), np.ones(k, np.int16), "same") > k // 2).astype(np.uint8)
    fall = np.flatnonzero((bits[:-1] == 1) & (bits[1:] == 0)) + 1
    out, nxt, n = [], 0, len(bits)
    for s in fall:
        if s < nxt:
            continue
        mid = s + spb * (np.arange(10) + 0.5)
        if mid[-1] >= n:
            break
        v = bits[mid.astype(int)]
        if v[0] != 0 or v[9] != 1:              # bad start or stop bit: resync on next edge
            nxt = s + 1
            continue
        out.append((int(sum(int(v[k + 1]) << k for k in range(8))), s / rate, (s + 10 * spb) / rate))
        nxt = s + 9.5 * spb
    return out


def packets(stream):
    """Split a byte stream into checksummed Feetech packets: (bytes, t_start, t_end)."""
    out, i, n = [], 0, len(stream)
    while i + 5 < n:
        if stream[i][0] != 0xFF or stream[i + 1][0] != 0xFF or stream[i + 2][0] == 0xFF:
            i += 1
            continue
        ln = stream[i + 3][0]
        j = i + 4 + ln
        if j > n:
            break
        pk = bytes(b for b, _, _ in stream[i:j])
        if (~sum(pk[2:-1])) & 0xFF == pk[-1]:
            out.append((pk, stream[i][1], stream[j - 1][2]))
            i = j
        else:
            i += 1
    return out


def same(x, p):
    """Identical, or the same goal sync write with values clamped by the monitor (same header and length)."""
    return x == p or (p[4] == 0x83 and len(x) == len(p) and x[:7] == p[:7])


def match(a, b, window=0.001):
    """For each packet in a, the first matching packet in b starting within window after it.
    1 ms is far above any forwarding delay and below the monitor's own hold write (>= 1 ms of bus quiet)."""
    res, k = [], 0
    for pk, s, e in a:
        while k < len(b) and b[k][1] < s:
            k += 1
        m = next((x for x in b[k:k + 20] if same(x[0], pk) and x[1] - s <= window), None)
        res.append((pk, s, e, m))
    return res


def stats(xs):
    xs = np.array(xs) * 1e6
    if len(xs) == 0:
        return None
    return {"n": int(len(xs)), "median_us": round(float(np.median(xs)), 1),
            "p99_us": round(float(np.percentile(xs, 99)), 1), "max_us": round(float(xs.max()), 1),
            "min_us": round(float(xs.min()), 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--rate", type=float, default=12e6)
    ap.add_argument("--lines", default="1,2", help="analyzer channels of the two bus lines")
    ap.add_argument("--json")
    a = ap.parse_args()
    raw = np.fromfile(a.capture, np.uint8)
    ch = [int(c) for c in a.lines.split(",")]
    pk = {c: packets(uart(((raw >> c) & 1).astype(np.uint8), a.rate)) for c in ch}
    print(f"{len(raw) / a.rate:.2f} s captured; packets: " + ", ".join(f"D{c} {len(pk[c])}" for c in ch))

    # origin side of instruction packets = host side
    c0, c1 = ch
    m01, m10 = match(pk[c0], pk[c1]), match(pk[c1], pk[c0])
    instr = lambda p: p[4] in (0x01, 0x02, 0x03, 0x82, 0x83)
    host = c0 if sum(1 for p, s, e, m in m01 if m and instr(p)) >= sum(1 for p, s, e, m in m10 if m and instr(p)) else c1
    servo = c1 if host == c0 else c0
    print(f"host side = D{host}, servo side = D{servo}")
    h2s, s2h = match(pk[host], pk[servo]), match(pk[servo], pk[host])

    res = {"capture_s": round(len(raw) / a.rate, 3), "host_line": f"D{host}", "servo_line": f"D{servo}"}
    for name, rows, want in (("host_to_servo", h2s, True), ("servo_to_host", s2h, False)):
        rows = [r for r in rows if r[3] and instr(r[0]) == want]
        res[name] = {"delay": stats([m[1] - s for p, s, e, m in rows]),
                     "turnaround": stats([m[1] - e for p, s, e, m in rows]),
                     "packet_time": stats([e - s for p, s, e, m in rows])}
        by = {}
        for p, s, e, m in rows:
            by.setdefault(f"0x{p[4]:02X} len{len(p)}" if want else f"reply len{len(p)}", []).append(m[1] - e)
        res[name]["turnaround_by_type"] = {k: stats(v) for k, v in sorted(by.items())}

    # dropped goals and the monitor's hold response
    own_read = lambda p: p[2] == 0xFE and p[4] == 0x82 and p[5] == 40 and p[6] == 18
    holds, prev = [], -1.0
    for p, s, e, m in h2s:
        if p[4] == 0x83 and m is None and s - prev > 0.05:     # first dropped goal of an episode
            prev = s
            rd = next((x for x in pk[servo] if x[1] > e and own_read(x[0])), None)
            wr = rd and next((x for x in pk[servo] if x[1] > rd[2] and x[0][4] == 0x83 and x[0][5] == 42
                              and not any(x[0] == q[0] and 0 <= x[1] - q[1] < 0.005 for q in pk[host])), None)
            holds.append({"t_s": round(float(s), 4),
                          "to_hold_read_us": rd and round(float(rd[1] - e) * 1e6, 1),
                          "to_hold_write_us": wr and round(float(wr[1] - e) * 1e6, 1)})
        elif p[4] == 0x83 and m is None:
            prev = s
    # a dropped-goal episode with the monitor's read right after it is a new hold; the rest are safe goals
    # dropped while a hold waits out RELEASE_MS
    res["holds"] = [h for h in holds if h["to_hold_read_us"] is not None and h["to_hold_read_us"] < 10000]
    res["release_wait_episodes"] = len(holds) - len(res["holds"])
    for f in ("to_hold_read_us", "to_hold_write_us"):
        v = [h[f] for h in res["holds"] if h[f] is not None]
        res[f.replace("_us", "")] = v and {"n": len(v), "median_us": float(np.median(v)), "max_us": max(v)}
    res["goals_clamped"] = sum(1 for p, s, e, m in h2s if m and p[4] == 0x83 and m[0] != p)
    came_back = {(p, round(m[1], 6)) for p, s, e, m in s2h if m}     # replies the Pico forwarded to the host
    unmatched = sum(1 for p, s, e, m in h2s if m is None and p[4] != 0x83 and (p, round(s, 6)) not in came_back)
    res["unmatched_nongoal_host_packets"] = unmatched

    for k in ("host_to_servo", "servo_to_host"):
        print(f"\n{k}:")
        for f in ("delay", "turnaround", "packet_time"):
            print(f"  {f:12} {res[k][f]}")
        for t, v in res[k]["turnaround_by_type"].items():
            print(f"    turnaround {t:12} {v}")
    print("\ngoals clamped:", res["goals_clamped"])
    print("holds:", res["holds"] or "none")
    print("  unsafe goal -> monitor's position read:", res["to_hold_read"])
    print("  unsafe goal -> hold goal write:        ", res["to_hold_write"])
    print("release-wait episodes (safe goals dropped until RELEASE_MS):", res["release_wait_episodes"])
    print("unmatched non-goal host packets:", unmatched)
    if a.json:
        open(a.json, "w").write(json.dumps(res, indent=1))
        print("saved", a.json)


if __name__ == "__main__":
    sys.exit(main())
