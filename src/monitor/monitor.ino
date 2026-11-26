// SO-101 interposer, stage 3: blocks unauthorized host writes (policy.h) and
// answers out-of-range goals by holding the arm at its measured position (hold.h). No speed check yet.
//
// Host side  (UART1, hardware): TX1 = GP4 -> U1B input, RX1 = GP5 <- BUS_H via 330R, DIR_H = GP6 -> U1B OE
// Servo side (UART0, hardware): TX0 = GP0 -> U1A input, RX0 = GP1 <- BUS_S via 330R, DIR_S = GP2 -> U1A OE
// TRIP  = GP15 -> relay IN (HIGH = relay energized = servo power on)
// ESTOP = GP14 <- E-stop NO contact (other side to GND), internal pull-up
//
// Build: arduino-cli compile --fqbn rp2040:rp2040:rpipico --output-dir build/monitor src/monitor

#include <Arduino.h>
#include "policy.h"
#include "hold.h"

const int DIR_H = 6, DIR_S = 2, TRIP = 15, ESTOP = 14;
const uint32_t BAUD = 1000000;

#define HostBus  Serial2                        // UART1: TX = GP4, RX = GP5
#define ServoBus Serial1                        // UART0: TX = GP0, RX = GP1

struct Parser {                                 // Feetech packet: FF FF ID LEN ...LEN bytes, last = checksum
  const int maxLen;                             // reject implausible LEN so corruption can't stall us
  uint8_t buf[264]; int n = 0, need = 0; uint32_t last = 0, bad = 0;
  explicit Parser(int m) : maxLen(m) {}

  void drop(int k) { n -= k; memmove(buf, buf + k, n); }

  void push(uint8_t b) {
    uint32_t now = micros();
    if (n && now - last > 2000) n = 0;          // drop stale partial packet
    last = now;
    if (n == (int)sizeof(buf)) drop(1);
    buf[n++] = b;
  }

  // True when buf[0..need) is a complete packet with a good checksum; call consume() after using it.
  bool next() {
    while (n >= 2) {
      if (buf[0] != 0xFF || buf[1] != 0xFF) { drop(1); continue; }
      if (n < 4) return false;
      if (buf[2] == 0xFF) { drop(1); continue; }                        // extra header byte
      if (buf[3] < 2 || buf[3] > maxLen) { bad++; drop(1); continue; }
      need = 4 + buf[3];
      if (n < need) return false;
      uint8_t sum = 0;
      for (int i = 2; i < need - 1; i++) sum += buf[i];
      if ((uint8_t)~sum == buf[need - 1]) return true;
      bad++; drop(1);                           // bad checksum: rescan the rest for a header
    }
    if (n == 1 && buf[0] != 0xFF) n = 0;
    return false;
  }
  void consume() { drop(need); }
};

Parser fromHost(250), fromServo(128);
int echoH = 0, echoS = 0;                       // our own bytes we'll hear back on each bus
uint32_t blocked = 0;
Verdict lastV = ALLOW;
uint8_t lastId = 0, lastIns = 0, lastAddr = 0;
uint32_t tH = 0, tS = 0, pktHS = 0, pktSH = 0, lastPrint = 0;

void sendOn(HardwareSerial &port, int dir, const uint8_t *p, int len, int &echo, uint32_t &t) {
  digitalWrite(dir, HIGH);                      // enable 74HC126 gate onto the bus
  port.write(p, len);
  port.flush();                                 // wait for TX to drain
  digitalWrite(dir, LOW);                       // release bus so the other side can talk
  echo += len; t = micros();
}

Hold hold;
bool needHold = false;                          // a hold was entered but not every joint has been handled
uint8_t handled = 0, lastJoint = 0;             // bit id = joint held, or limp and left alone
uint32_t holdFail = 0, goalDrop = 0;

const uint8_t H_ADDR = 40, H_LEN = 18;          // Torque_Enable (40) .. Present_Position (56-57)
bool capturing = false;                         // true while collecting replies to our own read
uint8_t got = 0, torque[7];
int where[7];

void pumpServo() {                              // servos -> host, except replies to our own read
  while (ServoBus.available()) {
    uint8_t b = ServoBus.read();
    if (echoS > 0) { echoS--; continue; }
    fromServo.push(b);
    while (fromServo.next()) {
      const uint8_t *p = fromServo.buf;
      uint8_t id = p[2];
      if (capturing && validId(id) && fromServo.need == H_LEN + 6 && !(got & (1 << id))) {
        torque[id] = p[5];
        where[id] = p[5 + 16] | p[5 + 17] << 8;
        got |= 1 << id;
      } else {
        sendOn(HostBus, DIR_H, p, fromServo.need, echoH, tH); pktSH++;
      }
      fromServo.consume();
    }
  }
  if (echoS && micros() - tS > 500) echoS = 0;  // resync if an echo byte was lost
}

void sendServo(uint8_t *p, int n) {             // fill in the checksum and send
  uint8_t sum = 0;
  for (int i = 2; i < n - 1; i++) sum += p[i];
  p[n - 1] = ~sum;
  sendOn(ServoBus, DIR_S, p, n, echoS, tS);
}

// Measure every joint now, then make that its goal. A limp joint is skipped: a goal write may re-enable torque.
void holdHere() {
  uint32_t start = micros(), quiet = start;     // let replies to earlier host reads finish first
  while (micros() - quiet < 1000 && micros() - start < 5000) {
    if (ServoBus.available()) quiet = micros();
    pumpServo();
  }
  for (int attempt = 0; attempt < 3 && handled != 0x7E; attempt++) {
    uint8_t rd[14] = {0xFF, 0xFF, 0xFE, 10, 0x82, H_ADDR, H_LEN};
    int n = 7;
    for (uint8_t id = 1; id <= 6; id++) if (!(handled & (1 << id))) rd[n++] = id;
    rd[3] = n - 3;                              // LEN = everything after it, checksum included
    const uint8_t want = 0x7E & ~handled;
    got = 0; capturing = true;
    sendServo(rd, n + 1);
    for (uint32_t t = micros(); got != want && micros() - t < 4000;) pumpServo();
    capturing = false;

    uint8_t wr[26] = {0xFF, 0xFF, 0xFE, 0, 0x83, 42, 2};
    uint8_t limp = 0, move = 0;
    n = 7;
    for (uint8_t id = 1; id <= 6; id++) {
      if (!(got & want & (1 << id))) continue;
      if (!torque[id]) { limp |= 1 << id; continue; }
      if (where[id] > 4095) continue;           // negative or bad reading: try again
      int g = clampGoal(id, where[id]);
      wr[n++] = id; wr[n++] = g & 0xFF; wr[n++] = g >> 8;
      move |= 1 << id;
    }
    if (move) { wr[3] = n - 3; sendServo(wr, n + 1); }
    handled |= limp | move;
  }
  needHold = handled != 0x7E;
  if (needHold) holdFail++;
}

void setup() {
  pinMode(DIR_H, OUTPUT); digitalWrite(DIR_H, LOW);
  pinMode(DIR_S, OUTPUT); digitalWrite(DIR_S, LOW);
  pinMode(ESTOP, INPUT_PULLUP);
  pinMode(TRIP, OUTPUT); digitalWrite(TRIP, HIGH);   // energize relay = servo power on

  HostBus.setTX(4); HostBus.setRX(5); HostBus.setFIFOSize(1024); HostBus.begin(BAUD);     // host side
  ServoBus.setTX(0); ServoBus.setRX(1); ServoBus.setFIFOSize(1024); ServoBus.begin(BAUD); // servo side
  Serial.begin(115200);                         // USB status output
}

void loop() {
  bool estop = digitalRead(ESTOP) == LOW;       // NO contact closes to GND when pressed
  digitalWrite(TRIP, estop ? LOW : HIGH);

  while (HostBus.available()) {                 // host -> servos
    uint8_t b = HostBus.read();
    if (echoH > 0) { echoH--; continue; }
    fromHost.push(b);
    while (fromHost.next()) {
      Verdict v = checkPacket(fromHost.buf, fromHost.need);
      const bool isGoal = v == ALLOW && fromHost.buf[4] == 0x83;
      if (isGoal) {
        uint8_t j = unsafeJoint(fromHost.buf, fromHost.need);
        bool was = hold.on;
        if (j) lastJoint = j;
        if (!hold.goal(j == 0, millis())) {
          goalDrop++;
          if (!was) { handled = 0; needHold = true; }
          if (needHold) holdHere();             // also retries joints that did not answer
          fromHost.consume();
          continue;
        }
        clampPacket(fromHost.buf, fromHost.need);
      }
      if (v == ALLOW) {
        sendOn(ServoBus, DIR_S, fromHost.buf, fromHost.need, echoS, tS);
        pktHS++;
      } else {                                  // dropped: host sees a timeout
        blocked++; lastV = v; lastId = fromHost.buf[2]; lastIns = fromHost.buf[4]; lastAddr = fromHost.need > 6 ? fromHost.buf[5] : 0;
      }
      fromHost.consume();
    }
  }
  pumpServo();
  if (echoH && micros() - tH > 500) echoH = 0;  // resync if an echo byte was lost

  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    if (Serial.availableForWrite() >= 230)      // never let an unread USB port stall the bus
      Serial.printf("H->S %lu  S->H %lu  badH %lu  badS %lu  estop=%d  BLOCKED %lu last=%s id=%u ins=0x%02X addr=%u  HOLD %d n=%lu joint=%u drop=%lu fail=%lu\n",
                    pktHS, pktSH, fromHost.bad, fromServo.bad, estop, blocked, VERDICT_NAME[lastV], lastId, lastIns, lastAddr,
                    hold.on, hold.count, lastJoint, goalDrop, holdFail);
  }
}
