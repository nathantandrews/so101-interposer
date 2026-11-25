// g++ -std=c++17 -Wall -Wextra tests/test_policy.cpp -o /tmp/test_policy && /tmp/test_policy tests/lerobot_frames.txt
#include "../src/monitor/policy.h"
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
using V = std::vector<int>;
static int fails = 0, n = 0;
static std::vector<uint8_t> mk(int id, V body) {
  std::vector<uint8_t> p{0xFF, 0xFF, (uint8_t)id, (uint8_t)(body.size() + 1)};
  int s = p[2] + p[3];
  for (int b : body) { p.push_back(b); s += b; }
  p.push_back(~s); return p;
}
static void t(const char *name, bool allow, int id, V body) {
  auto p = mk(id, body); Verdict v = checkPacket(p.data(), p.size()); n++;
  if ((v == ALLOW) != allow) { fails++; printf("FAIL %s: %s\n", name, VERDICT_NAME[v]); }
}
int main(int c, char **v) {
  int frames = 0;
  if (c < 2) return 2;
  std::ifstream f(v[1]); std::string l;
  while (f >> l) {
    std::vector<uint8_t> p;
    for (size_t i = 0; i + 1 < l.size(); i += 2) p.push_back(std::stoi(l.substr(i, 2), nullptr, 16));
    Verdict r = checkPacket(p.data(), p.size()); frames++;
    if (r != ALLOW) { fails++; printf("FAIL LeRobot frame %s: %s\n", l.c_str(), VERDICT_NAME[r]); }
  }
  for (int id = 1; id <= 6; id++) {
    t("min limit", false, id, {3, 9, 0, 0}); t("max limit", false, id, {3, 11, 255, 15});
    t("both limits", false, id, {3, 9, 0, 0, 255, 15}); t("homing", false, id, {3, 31, 0, 0});
    t("torque limit", false, id, {3, 48, 232, 3}); t("goal vel", false, id, {3, 46, 255, 15});
    t("id", false, id, {3, 5, 9}); t("baud", false, id, {3, 6, 4}); t("rezero", false, id, {3, 40, 128});
    t("mode pwm", false, id, {3, 33, 2}); t("span into 42", false, id, {3, 41, 254, 0, 0});
    t("span 40..", false, id, {3, 40, 1, 254}); t("reg write", false, id, {4, 9, 0, 0});
    t("action", false, id, {5}); t("reset", false, id, {6}); t("reboot", false, id, {8}); t("calib", false, id, {11, 0, 8});
    t("single goal write", false, id, {3, 42, 0, 8}); t("P", false, id, {3, 21, 255});
    t("read ok", true, id, {2, 56, 2}); t("read long", false, id, {2, 255, 255});
    t("gripper rule", id == 6, id, {3, 16, 0xF4, 1}); t("max torque 1000", false, id, {3, 16, 0xE8, 3});
  }
  t("bcast write", false, 254, {3, 40, 1}); t("bcast limit", false, 254, {3, 9, 0, 0});
  t("bcast action", false, 254, {5}); t("bcast ping", true, 254, {1}); t("id 7", false, 7, {3, 40, 1}); t("id 0", false, 0, {2, 56, 2});
  t("sw limits", false, 254, {0x83, 9, 2, 1, 0, 0}); t("sw torque", false, 254, {0x83, 48, 2, 1, 0, 0});
  t("sw wide", false, 254, {0x83, 42, 6, 1, 0, 8, 0, 0, 0, 0}); t("sw dup", false, 254, {0x83, 42, 2, 1, 0, 8, 1, 0, 9});
  t("sw short", false, 254, {0x83, 42, 2, 1, 0}); t("sw ffff", false, 254, {0x83, 42, 2, 1, 255, 255});
  t("sw not bcast", false, 1, {0x83, 42, 2, 1, 0, 8}); t("sw ok", true, 254, {0x83, 42, 2, 1, 0, 8});
  t("sr dup", false, 254, {0x82, 56, 2, 1, 1});
  for (int ins = 0; ins < 256; ins++)
    if (ins != 1 && ins != 2 && ins != 3 && ins != 0x82 && ins != 0x83) t("instr", false, 1, {ins, 9, 0, 0});
  printf("%d LeRobot frames, %d cases, %d failures\n", frames, n, fails);
  return fails != 0 || frames == 0;
}
