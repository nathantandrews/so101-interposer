// C wrapper so broker.py runs the exact policy and envelope the Pico runs.
//   g++ -O2 -shared -fPIC -o broker/libpolicy.so broker/policy_shim.cpp
#include "../src/monitor/policy.h"
#include "../src/monitor/hold.h"

static Hold hold;
extern "C" {
int check_packet(const uint8_t *p, int n) { return checkPacket(p, n); }
int unsafe_joint(const uint8_t *p, int n) { return unsafeJoint(p, n); }
void clamp_packet(uint8_t *p, int n) { clampPacket(p, n); }
int hold_goal(int safe, uint32_t now_ms) { return hold.goal(safe, now_ms); }
int hold_on() { return hold.on; }
uint32_t hold_count() { return hold.count; }
const char *verdict_name(int v) { return VERDICT_NAME[v]; }
}
