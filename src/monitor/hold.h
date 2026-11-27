// Joint envelope and hold state. Pure C++ so tests/ can run it on a PC.
// Limits = follower calibration of 2026-11-27 (my_follower.json). Recalibrating means editing and reflashing.
// wrist_roll is calibrated 0..4095 (full turn), so its reading wraps 4095 <-> 0 and a goal can jump a whole
// turn. Recentered on 2026-11-27, its physical stops read about 172 and 3976; the limit sits 150 ticks
// inside each, so a clamped goal (forwarded as the limit) stops short of the stop, and a wrap is a violation.
#pragma once
#include <stdint.h>

struct Range { int lo, hi; };
static const Range JOINT[7] = {{0, 0}, {728, 3472}, {802, 3192}, {916, 3128}, {794, 2848}, {322, 3826}, {2034, 3549}};
// A goal up to SLACK ticks past JOINT is forwarded clamped to JOINT; further out is a violation and holds.
// LeRobot's default degree mode does not clamp goals: measured 2026-11-25, this leader's calibrated ends ask
// for up to 57 ticks past the follower's shoulder_lift range. 80 ticks = 7 degrees.
// The leader's wrist travels further than the follower's (measured 2026-11-27): wrist_flex asks 150 ticks past
// both ends, wrist_roll's stops ask 185 / 112 past the limits above. Those two get 200 (18 degrees); a wrap
// (goal near 0 or 4095) still lands outside and holds.
static const int SLACK[7] = {0, 80, 80, 80, 200, 200, 80};
static const uint32_t RELEASE_MS = 500;        // only safe goals for this long end a hold

inline bool goalOk(uint8_t id, int g) { return g >= JOINT[id].lo - SLACK[id] && g <= JOINT[id].hi + SLACK[id]; }
inline int clampGoal(uint8_t id, int g) { return g < JOINT[id].lo ? JOINT[id].lo : g > JOINT[id].hi ? JOINT[id].hi : g; }

// First joint in a SYNC WRITE Goal_Position packet (already checked by checkPacket) that is out of range, or 0.
inline uint8_t unsafeJoint(const uint8_t *p, int n) {
  for (int i = 7; i + 2 < n - 1; i += 3)
    if (!goalOk(p[i], p[i + 1] | p[i + 2] << 8)) return p[i];
  return 0;
}

// Clamp every goal in a SYNC WRITE Goal_Position packet to JOINT and fix the checksum.
inline void clampPacket(uint8_t *p, int n) {
  uint8_t sum = 0;
  for (int i = 7; i + 2 < n - 1; i += 3) {
    int g = clampGoal(p[i], p[i + 1] | p[i + 2] << 8);
    p[i + 1] = g & 0xFF; p[i + 2] = g >> 8;
  }
  for (int i = 2; i < n - 1; i++) sum += p[i];
  p[n - 1] = ~sum;
}

struct Hold {
  bool on = false, timing = false;
  uint32_t t0 = 0;
  uint32_t count = 0;                          // holds entered
  // Returns true if the goal may be forwarded.
  bool goal(bool safe, uint32_t now) {
    if (!safe) { count += !on; on = true; timing = false; return false; }
    if (!on) return true;
    if (!timing) { timing = true; t0 = now; return false; }
    if (now - t0 < RELEASE_MS) return false;
    on = timing = false;
    return true;
  }
};
