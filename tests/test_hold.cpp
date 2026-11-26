// Runs the real monitor.ino against six simulated servos.
// g++ -std=c++17 -Wall -Wextra -Itests/mock tests/test_hold.cpp -o /tmp/test_hold && /tmp/test_hold
#include <Arduino.h>
uint32_t simUs = 1000;
HardwareSerial Serial1, Serial2;
Usb Serial;
#include "../src/monitor/monitor.ino"
#include <map>
using B = std::vector<uint8_t>;
static int fails = 0, checks = 0;
#define CHECK(x) do { checks++; if (!(x)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)

struct Servo { int pos = 2500, goal = 2500; bool torque = true; int mute = 0; };   // mute: reads to ignore
static Servo sv[7];
static B mk(int id, B body) {
  B p{0xFF, 0xFF, (uint8_t)id, (uint8_t)(body.size() + 1)}; p.insert(p.end(), body.begin(), body.end());
  int s = 0; for (size_t i = 2; i < p.size(); i++) s += p[i];
  p.push_back(~s); return p;
}
static void put(HardwareSerial &s, const B &p, uint32_t t) { for (uint8_t b : p) s.rx.push_back({t += 10, b}); }
static void servos(const B &p) {                // what the servos do with a packet from the monitor
  if (p[4] == 0x83) for (size_t i = 7; i + 3 < p.size(); i += 3) { sv[p[i]].goal = p[i + 1] | p[i + 2] << 8; }
  if (p[4] == 0x82) {
    uint32_t t = simUs + 60;
    for (size_t i = 7; i + 1 < p.size(); i++) {
      Servo &s = sv[p[i]];
      if (s.mute > 0) { s.mute--; continue; }
      B mem(128, 0); mem[40] = s.torque; mem[56] = s.pos & 255; mem[57] = s.pos >> 8;
      B body{0}; body.insert(body.end(), mem.begin() + p[5], mem.begin() + p[5] + p[6]);
      B r = mk(p[i], body); put(Serial1, r, t); t += 10 * r.size() + 30;
    }
  }
}
static void run(int us) { for (uint32_t e = simUs + us; (int32_t)(e - simUs) > 0;) { loop(); simUs += 5; } }
static void host(const B &p, int us = 3000) { put(Serial2, p, simUs); run(us); }
static B goals(std::map<int, int> g) {
  B b{0x83, 42, 2}; for (auto &[i, v] : g) { b.push_back(i); b.push_back(v & 255); b.push_back(v >> 8); }
  return mk(254, b);
}
static const std::map<int, int> MID{{1, 2000}, {2, 2000}, {3, 2000}, {4, 2000}, {5, 2000}, {6, 2500}};
static std::map<int, int> with(int id, int g) { auto m = MID; m[id] = g; return m; }
static void fresh() {
  for (int i = 1; i <= 6; i++) sv[i] = {2100 + i, 2500, true, 0};
  Serial1.rx.clear(); Serial2.rx.clear(); Serial1.sent.clear(); Serial2.sent.clear();
  hold = Hold(); needHold = false; handled = 0; holdFail = goalDrop = 0; echoH = echoS = 0; fromHost.n = fromServo.n = 0;
}
static bool held(int id) { return sv[id].goal == sv[id].pos; }

int main() {
  setup(); Serial1.onSend = servos;
  // every joint, both ends: the calibrated ends pass, up to SLACK past is clamped, further holds in place
  for (int id = 1; id <= 6; id++) for (int end = 0; end < 2; end++) {
    int edge = end ? JOINT[id].hi : JOINT[id].lo, dir = end ? 1 : -1, past = edge + dir * (SLACK + 1);
    fresh(); host(goals(with(id, edge))); CHECK(sv[id].goal == edge && !hold.on);
    for (int over : {1, 42, 57, SLACK}) {
      int g = edge + dir * over;
      if (g < 0 || g > 4095) continue;
      fresh(); host(goals(with(id, g)));
      CHECK(sv[id].goal == edge && !hold.on);
      for (int j = 1; j <= 6; j++) if (j != id) CHECK(sv[j].goal == MID.at(j));
      uint8_t sum = 0; B &p = Serial1.sent.back();
      for (size_t i = 2; i < p.size(); i++) sum += p[i];
      CHECK(sum == 0xFF);                       // valid checksum after clamping
    }
    if (past < 0 || past > 4095) continue;      // policy.h already rejects these
    fresh(); host(goals(with(id, past)), 12000);
    CHECK(hold.on && hold.count == 1 && holdFail == 0);
    for (int j = 1; j <= 6; j++) CHECK(held(j));
    CHECK(Serial2.sent.empty());                // host never sees our read's replies
    CHECK(Serial1.sent.size() == 2);            // one read, one hold; the unsafe packet never went out
    fresh(); host(goals({{id, past}}), 12000);  // a one-joint packet holds the whole arm too
    for (int j = 1; j <= 6; j++) CHECK(held(j));
  }
  // auto release after 500 ms of only safe goals
  fresh(); host(goals(with(1, 4000)), 12000);
  host(goals(MID)); CHECK(hold.on && held(1));
  run(450000); host(goals(MID)); CHECK(hold.on && held(1));
  run(60000); host(goals(MID)); CHECK(!hold.on && sv[1].goal == 2000);
  // an unsafe goal restarts the wait, and the hold target does not creep
  fresh(); host(goals(with(1, 4000)), 12000); host(goals(MID)); run(450000);
  int target = sv[1].goal; sv[1].pos += 30;
  host(goals(with(1, 4000)), 12000); CHECK(sv[1].goal == target && hold.count == 1);
  host(goals(MID)); run(450000); host(goals(MID)); CHECK(hold.on);
  run(60000); host(goals(MID)); CHECK(!hold.on);
  // limp joints are left alone
  fresh(); sv[2].torque = false; int before = sv[2].goal;
  host(goals(with(1, 4000)), 12000); CHECK(sv[2].goal == before && held(1) && !needHold);
  // a joint that misses two reads is still held; one that never answers is counted and retried later
  fresh(); sv[3].mute = 2; host(goals(with(1, 4000)), 30000); CHECK(held(3) && holdFail == 0);
  fresh(); sv[3].mute = 3; host(goals(with(1, 4000)), 30000); CHECK(!held(3) && held(1) && holdFail == 1 && needHold);
  host(goals(with(1, 4000)), 30000); CHECK(held(3) && !needHold);
  // a joint already outside the range is held where it is, not pulled back to the edge
  fresh(); sv[5].pos = 3900; host(goals(with(1, 4000)), 12000); CHECK(hold.on && held(5));
  // wrist_roll wrap: the leader crosses 4095 -> 0, so the goal jumps a whole turn; both sides hold
  fresh(); host(goals(with(5, 3700))); CHECK(sv[5].goal == 3700 && !hold.on);
  host(goals(with(5, 3830))); CHECK(sv[5].goal == 3755 && !hold.on);
  host(goals(with(5, 4080)), 12000); CHECK(hold.on && held(5));
  host(goals(with(5, 15)), 12000); CHECK(hold.on && held(5) && hold.count == 1);
  run(600000); host(goals(with(5, 15))); CHECK(hold.on && held(5));
  // a host read in flight when the bad goal arrives still reaches the host
  fresh(); put(Serial2, mk(254, {0x82, 56, 2, 1, 2, 3, 4, 5, 6}), simUs);
  put(Serial2, goals(with(1, 4000)), simUs + 150); run(20000);
  CHECK(Serial2.sent.size() == 6 && hold.on); for (int j = 1; j <= 6; j++) CHECK(held(j));
  // reads and torque-off still work during a hold; register writes stay blocked
  size_t h = Serial2.sent.size(); host(mk(254, {0x82, 56, 2, 1})); CHECK(Serial2.sent.size() == h + 1);
  size_t s = Serial1.sent.size(); host(mk(1, {3, 40, 0})); CHECK(Serial1.sent.size() == s + 1);
  host(mk(1, {3, 9, 0, 0})); CHECK(Serial1.sent.size() == s + 1);
  // millis() rollover during the wait
  Hold r; r.goal(false, 0xFFFFFF00u); r.goal(true, 0xFFFFFF00u); CHECK(!r.goal(true, 0x10)); CHECK(r.goal(true, 0x200));
  printf("%d checks, %d failures\n", checks, fails);
  return fails != 0;
}
