// Default-deny policy for host -> servo packets. Pure C++ so tests/ can run it on a PC.
// Values are what LeRobot 0.7.0's SO-101 follower writes in normal control (see tests/).
// Not checked yet: joint range, speed, oscillation (hold stage), maintenance mode.
#pragma once
#include <stdint.h>

enum Verdict : uint8_t { ALLOW, BAD_FRAME, BAD_ID, BAD_INSTR, BAD_READ, BAD_WRITE, BAD_SYNC };
static const char *const VERDICT_NAME[] = {"allow", "frame", "id", "instr", "read", "write", "sync"};

struct WriteRule { uint8_t id, addr, len; uint8_t v[2]; };   // id 0 = any of 1..6; exact value only
static const WriteRule RULES[] = {
  {0, 40, 1, {0, 0}}, {0, 40, 1, {1, 0}},      // Torque_Enable 0/1 only (128 would re-zero the servo)
  {0, 55, 1, {0, 0}}, {0, 55, 1, {1, 0}},      // Lock
  {0,  7, 1, {0, 0}},                          // Return_Delay_Time
  {0, 85, 1, {254, 0}}, {0, 41, 1, {254, 0}},  // Maximum_Acceleration, Acceleration
  {0, 33, 1, {0, 0}},                          // Operating_Mode = position
  {0, 21, 1, {16, 0}}, {0, 22, 1, {32, 0}}, {0, 23, 1, {0, 0}},   // P, D, I
  {6, 16, 2, {0xF4, 0x01}},                    // gripper Max_Torque_Limit 500
  {6, 28, 2, {0xFA, 0x00}},                    // gripper Protection_Current 250
  {6, 36, 1, {25, 0}},                         // gripper Overload_Torque 25
};

inline bool validId(uint8_t id) { return id >= 1 && id <= 6; }

// p = whole packet FF FF ID LEN INSTR params.. CHK; checksum already verified by the parser.
inline Verdict checkPacket(const uint8_t *p, int n) {
  if (n < 6 || n != p[3] + 4) return BAD_FRAME;
  const uint8_t id = p[2], ins = p[4], *a = p + 5;
  const int np = n - 6;                        // parameter count
  switch (ins) {
    case 0x01:                                 // PING
      if (!validId(id) && id != 0xFE) return BAD_ID;
      return np == 0 ? ALLOW : BAD_FRAME;
    case 0x02:                                 // READ
      if (!validId(id)) return BAD_ID;
      if (np != 2) return BAD_FRAME;
      return a[1] >= 1 && a[0] + a[1] <= 128 ? ALLOW : BAD_READ;
    case 0x82: {                               // SYNC READ
      if (id != 0xFE) return BAD_ID;
      if (np < 3 || np > 8) return BAD_FRAME;
      if (a[1] < 1 || a[0] + a[1] > 128) return BAD_READ;
      uint8_t seen = 0;
      for (int i = 2; i < np; i++) {
        if (!validId(a[i]) || (seen & (1 << a[i]))) return BAD_ID;
        seen |= 1 << a[i];
      }
      return ALLOW;
    }
    case 0x03:                                 // WRITE: no broadcast, exact values only
      if (!validId(id)) return BAD_ID;
      for (const WriteRule &r : RULES) {
        if ((r.id && r.id != id) || np != 1 + r.len || a[0] != r.addr) continue;
        if (a[1] == r.v[0] && (r.len == 1 || a[2] == r.v[1])) return ALLOW;
      }
      return BAD_WRITE;
    case 0x83: {                               // SYNC WRITE: Goal_Position only
      if (id != 0xFE) return BAD_ID;
      if (np < 5 || a[0] != 42 || a[1] != 2 || (np - 2) % 3 || (np - 2) / 3 > 6) return BAD_SYNC;
      uint8_t seen = 0;
      for (int i = 2; i < np; i += 3) {
        if (!validId(a[i]) || (seen & (1 << a[i]))) return BAD_ID;
        seen |= 1 << a[i];
        if (a[i + 2] > 0x0F) return BAD_SYNC;  // goal must be 0..4095
      }
      return ALLOW;
    }
    default: return BAD_INSTR;                 // REG_WRITE, ACTION, RESET, REBOOT, calibrate, unknown
  }
}
