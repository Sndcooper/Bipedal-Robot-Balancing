// ============================================================================
// ax12_control — AX-12+ LEG SUBSYSTEM BENCH RIG (wireless)
// STM32F401CD Black Pill | 3DR telemetry USART1 (PA9/PA10 @ 115200)
// AX-12 bus Serial2 (PA2/PA3 @ 1 Mbaud)
// ----------------------------------------------------------------------------
// SERVOS ONLY. There is deliberately no IMU, no balance PID, no encoders and
// no motor drive in this firmware — the drive-motor pins are driven LOW at boot
// so the L298N is definitively dead while hands are in the linkage. The whole
// 100 Hz loop belongs to the AX-12 bus.
//
// WHAT IT EXPOSES
//   * global torque limit / compliance margin / compliance slope / moving speed
//   * a master torque kill (limp legs) with safe re-grip
//   * per-servo raw goal position (bypasses IK)
//   * two mutually exclusive pose modes: CROUCH (one knob) and IK (foot x,y)
//   * present position / speed / load / voltage / temperature readback
//
// TECHNIQUES CARRIED VERBATIM FROM mcu_ik_engine_pretest_wireless
// (kept unchanged on purpose — these were the fixes for the sequential-running
//  bottleneck, and they are protocol-independent):
//   * 256 B UART TX+RX buffers (see platformio.ini — 64 B silently truncates)
//   * 300 us RX drain budget, called EVERY tick
//   * telemetry built into one buffer, written once, guarded by
//     availableForWrite() >= n — never per-field print() (that blocks and
//     stretches the loop's dt when the radio backs up)
//   * ~1 line per 100 ms downlink budget, because the 3DR/SiK air link is
//     half-duplex TDM and a fat downlink starves the uplink for seconds
//   * non-blocking one-servo-per-20 ms poll state machine
//   * every settings write spread one servo per tick, never batched
// ============================================================================

#include <Arduino.h>

// STM32F401 Black Pill: USART3 does not exist; 3DR uses USART1 (PA9/PA10).
#define Serial3 Serial1
extern HardwareSerial Serial6;

// ── DRIVE MOTOR PINS (L298N) — FORCED OFF, NEVER DRIVEN ──────────────────────
// Not used by this firmware. Declared solely so setup() can pin them LOW: a
// floating enable on a powered L298N can latch a wheel on while you have both
// hands in the leg linkage.
#define ENA PA1
#define IN1 PB14
#define IN2 PB15
#define ENB PA0
#define IN3 PB12
#define IN4 PB13

// ── SERIAL PORTS (instantiated via build_flags) ──────────────────────────────
extern HardwareSerial Serial1;   // 3DR radio (PA9 TX / PA10 RX)
extern HardwareSerial Serial2;   // AX-12 bus
extern HardwareSerial Serial3;   // 3DR radio

// ── LOOP TIMING ──────────────────────────────────────────────────────────────
unsigned long lastTime      = 0;
unsigned long lastPrintTime = 0;
const unsigned long TICK_US = 10000;   // 100 Hz

// ── GUARDED RADIO LINE ───────────────────────────────────────────────────────
// NEVER call Serial3.print/println from the 100 Hz path. println BLOCKS once
// the radio's TX ring backs up, at ~87 us per byte at 115200 — a 20-byte ack
// stalls the loop for 1.7 ms, a sixth of the whole tick, and the profiler will
// show it as a body-time spike with no work to account for it. This drops the
// line instead of waiting. A dropped ack is cheap: the GUI's slider guard
// re-requests state (RB) when an ack goes missing, so it costs a round trip,
// not correctness. The telemetry path has always guarded this way; these ack
// sites simply never did.
void radioLine(const char *s) {
  int n = (int)strlen(s);
  if (n <= 0 || n > 190) return;
  char buf[192];
  memcpy(buf, s, (size_t)n);
  buf[n++] = '\n';
  if (Serial3.availableForWrite() >= n) Serial3.write((uint8_t *)buf, n);
}

// ── PROFILING — one raw sample per tick, out on Serial6 / PA11 ───────────────
// The question this answers: how much of the 10 ms is actually being consumed,
// and by what. Every segment of the loop body is timed separately with micros()
// and emitted every single tick, unaggregated.
//
// Cost accounting, because a profiler that perturbs the thing it measures is
// worthless: `B` is captured BEFORE this line is composed or written, so it is
// the honest un-instrumented body time. The instrumentation's own cost is
// reported separately as `X` (previous tick's, since the current one cannot be
// known until after the write). Subtract X to get the true budget on a build
// with profiling compiled out.
//
// Bandwidth: ~38 B/tick x 100 Hz = ~3.8 kB/s, about a third of the 115200 port.
// The TX ring drains 115 B per 10 ms tick and we add 38, so it stays near
// empty and the guarded write below effectively never has to drop.
uint32_t prof_rxReqUs  = 0;    // micros() when a servo read request went out
uint32_t prof_rxLatUs  = 0;    // request -> reply consumed, latched for one tick
uint16_t prof_rxTo     = 0;    // servo read timeouts since the last emitted line
uint16_t prof_dropped  = 0;    // profiler lines dropped (Serial1 ring full)
uint32_t prof_printUs  = 0;    // previous tick's instrumentation cost

// ── AX-12 HELPERS ────────────────────────────────────────────────────────────
void ax12WriteByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t checksum = ~(id + 4 + 3 + addr + val) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x03, addr, val, checksum};
  Serial2.write(packet, 8);
  Serial2.flush(); // half-duplex: drain before the bus can switch direction
}

void ax12WriteWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t checksum = ~(id + 5 + 3 + addr + lo + hi) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x05, 0x03, addr, lo, hi, checksum};
  Serial2.write(packet, 9);
  Serial2.flush();
}

// ── SERVO STATE ──────────────────────────────────────────────────────────────
// goalPos defaults are the calibrated standing pose (818 left / 441 right) —
// see the leg-ik-and-servos skill. Do not "round" these; they are measured.
struct ServoState {
  uint8_t  id;
  uint16_t goalPos;        // commanded position (raw AX-12 units, 0-1023)
  uint16_t presentPos;     // last read back (raw)
  int16_t  presentSpeed;   // signed, raw
  float    loadPct;
  float    volts;
  uint8_t  temp;
};

ServoState legServos[4] = {
  {6,  818, 818, 0, 0.0f, 0.0f, 0},   // Leg1 Left  (818 = straight-down left)
  {0,  818, 818, 0, 0.0f, 0.0f, 0},   // Leg2 Left
  {14, 441, 441, 0, 0.0f, 0.0f, 0},   // Leg1 Right (441 = straight-down right)
  {1,  441, 441, 0, 0.0f, 0.0f, 0},   // Leg2 Right
};

// ── GLOBAL SERVO SETTINGS (applied identically to all four) ──────────────────
// Global rather than per-servo on purpose: the balancing firmware configures
// all four with identical values, so a per-servo split would only invent
// divergence that the robot never actually runs with.
//
// These are the responsive baseline inherited from the pretest firmware.
// Slope 32 / margin 4 (the sluggish combo) gives an 8x wider proportional band
// and a wide deadband: the joint eases in slowly and holds weakly, visibly
// creeping under the robot's own weight. That reads as "slow servos" and has
// nothing to do with serial latency.
uint16_t g_torqueLimit = 1023;   // addr 34, 0-1023  (1023 = full holding torque)
uint8_t  g_compMargin  = 1;      // addr 26/27, 0-254 (narrow deadband)
uint8_t  g_compSlope   = 4;      // addr 28/29, 0-254 (tight proportional band)
uint16_t g_movingSpeed = 0;      // addr 32, 0-1023  (0 = uncapped)
bool     g_torqueOn    = true;   // addr 24 master kill

// Dirty bitmask — one bit per legServos[] index. Set by a settings command,
// drained one servo per 100 Hz tick by applySettingsTask(). Pushing all the
// registers to all four servos in one go is ~28 packets (~2.5 ms of blocking
// flush()), a quarter of the tick; spreading it costs ~630 us instead.
uint8_t settingsDirtyMask = 0x0F;

void markAllSettingsDirty() { settingsDirtyMask = 0x0F; }

// ── POSE MODES ───────────────────────────────────────────────────────────────
// Two INDEPENDENT modes, never combined: whichever the operator selects owns
// the foot target outright. In CROUCH the foot x is pinned to the calibrated
// default and one knob slides it vertically; in IK the dragged (x,y) is used
// as-is and crouchOffset is ignored entirely.
#define MODE_CROUCH 0
#define MODE_IK     1
uint8_t poseMode = MODE_CROUCH;

// ── 5-BAR IK ─────────────────────────────────────────────────────────────────
// Ported verbatim from mcu_ik_engine_wireless / the pretest firmware — same
// mounts, same 818/441 calibration. Proven math, not re-derived.
#define SERVO_L_X -30.0f
#define SERVO_L_Y   0.0f
#define SERVO_R_X  30.0f
#define SERVO_R_Y   0.0f
#define FEMUR_LEN  55.0f
#define TIBIA_LEN 100.0f
#define LEG2_INVERTED_MOUNT true

const float BASE_FX1 =  1.0f, BASE_FY1 = -151.1f;  // Leg1 standing foot target (mm)
const float BASE_FX2 = -6.0f, BASE_FY2 = -149.6f;  // Leg2 standing foot target (mm)
const float IK_DIST  = 180.0f;                      // leg separation (mm)

float crouchOffset = 0.0f;                 // CR<f>: 0 = standing, + = crouched (mm)
float ft_x[2] = {BASE_FX1, BASE_FX2};      // FT<leg> x y: IK-mode foot targets (mm)
float ft_y[2] = {BASE_FY1, BASE_FY2};
bool  ikValid[2] = {true, true};           // last solve reachable?

struct Point2D { float x; float y; };

bool circle_intersections(Point2D p0, float r0, Point2D p1, float r1,
                           Point2D &out1, Point2D &out2) {
  float dx = p1.x - p0.x, dy = p1.y - p0.y;
  float d  = sqrtf(dx * dx + dy * dy);
  if (d > r0 + r1 || d < fabsf(r0 - r1) || d == 0) return false;
  float a  = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  float h  = sqrtf(fmaxf(r0 * r0 - a * a, 0.0f));
  float px = p0.x + a * dx / d;
  float py = p0.y + a * dy / d;
  float rx = -h * dy / d, ry = h * dx / d;
  out1 = {px + rx, py + ry};
  out2 = {px - rx, py - ry};
  return true;
}

struct IK_Result { bool valid; Point2D Knee_L, Knee_R; float Angle_L, Angle_R; };

IK_Result solve_ik(float tx, float ty, float leg_offset_x) {
  IK_Result res = {false};
  Point2D foot = {tx, ty};
  Point2D sl   = {SERVO_L_X + leg_offset_x, SERVO_L_Y};
  Point2D sr   = {SERVO_R_X + leg_offset_x, SERVO_R_Y};
  Point2D li1, li2, ri1, ri2;
  if (!circle_intersections(sl, FEMUR_LEN, foot, TIBIA_LEN, li1, li2)) return res;
  if (!circle_intersections(sr, FEMUR_LEN, foot, TIBIA_LEN, ri1, ri2)) return res;
  res.valid  = true;
  res.Knee_L = (li1.x < li2.x) ? li1 : li2;
  res.Knee_R = (ri1.x > ri2.x) ? ri1 : ri2;
  res.Angle_L = atan2f(res.Knee_L.y - sl.y, res.Knee_L.x - sl.x) * 180.0f / PI;
  res.Angle_R = atan2f(res.Knee_R.y - sr.y, res.Knee_R.x - sr.x) * 180.0f / PI;
  return res;
}

uint16_t map_angle_to_ax12(float ik_angle, bool is_left, bool is_leg2) {
  float base_angle = is_leg2 ? 90.0f : -90.0f;
  float diff_deg   = fmodf(ik_angle - base_angle + 180.0f, 360.0f);
  if (diff_deg < 0) diff_deg += 360.0f;
  diff_deg -= 180.0f;
  float base_pos = is_left ? 818.0f : 441.0f;
  float ax_pos   = base_pos + (diff_deg * 3.413f);
  return (uint16_t)constrain((int)ax_pos, 0, 1023);
}

// ── FORWARD KINEMATICS ───────────────────────────────────────────────────────
// Exact inverse of map_angle_to_ax12(), plus the 5-bar FK. Needed for ONE
// thing: after a limp/re-grip the legs are wherever the operator's hands left
// them, and the interpolator below has to know that in order to move smoothly
// away from it. Without this, the first sync-write of the next pose move would
// jump the servos back to the pre-limp pose — reintroducing exactly the slam
// the re-grip logic exists to prevent.
float ax12_to_angle(uint16_t pos, bool is_left, bool is_leg2) {
  float base_angle = is_leg2 ? 90.0f : -90.0f;
  float base_pos   = is_left ? 818.0f : 441.0f;
  return base_angle + ((float)pos - base_pos) / 3.413f;
}

bool solve_fk(float aL_deg, float aR_deg, float leg_offset_x, Point2D &foot) {
  Point2D sl = {SERVO_L_X + leg_offset_x, SERVO_L_Y};
  Point2D sr = {SERVO_R_X + leg_offset_x, SERVO_R_Y};
  Point2D kL = {sl.x + FEMUR_LEN * cosf(aL_deg * PI / 180.0f),
                sl.y + FEMUR_LEN * sinf(aL_deg * PI / 180.0f)};
  Point2D kR = {sr.x + FEMUR_LEN * cosf(aR_deg * PI / 180.0f),
                sr.y + FEMUR_LEN * sinf(aR_deg * PI / 180.0f)};
  Point2D f1, f2;
  if (!circle_intersections(kL, TIBIA_LEN, kR, TIBIA_LEN, f1, f2)) return false;
  foot = (f1.y < f2.y) ? f1 : f2;   // knee-up branch: the lower intersection
  return true;
}

// ── SYNC_WRITE — all four goal positions in ONE bus packet ───────────────────
// THIS IS THE FIX for "one servo of a leg moves before the other".
//
// The old path wrote goal positions inside pollLegServosTask(), one servo per
// slot. A slot is 40 ms (write+request on one call, read the reply on the next
// 20 ms call, next servo eligible 20 ms after that), so a full round-robin is
// 160 ms. The two servos of ONE leg sit two slots apart in legServos[] — ID 6
// at index 0, ID 14 at index 2 — so they received their new goals 80 ms apart.
// At the AX-12's uncapped ~1208 counts/s that is ~97 counts, 37% of a full
// crouch-to-stand travel, completed by one joint before the other was even
// told to move.
//
// SYNC_WRITE (instruction 0x83, broadcast ID 0xFE) hands every servo its goal
// in a single 20-byte packet — ~200 us at 1 Mbaud, 2% of the tick. Every joint
// latches on the same byte, so the stagger is not reduced, it is structurally
// impossible. Being a broadcast there is no status reply, only the half-duplex
// echo of our own bytes, which is drained here.
void ax12SyncWriteGoals() {
  uint8_t pkt[20];
  pkt[0] = 0xFF; pkt[1] = 0xFF;
  pkt[2] = 0xFE;                 // broadcast
  pkt[3] = (2 + 1) * 4 + 4;      // LENGTH = (data_len+1)*n_servos + 4 = 16
  pkt[4] = 0x83;                 // SYNC_WRITE
  pkt[5] = 30;                   // start address: Goal Position
  pkt[6] = 2;                    // bytes per servo
  uint16_t sum = 0xFE + pkt[3] + 0x83 + 30 + 2;
  uint8_t k = 7;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    uint8_t lo = legServos[i].goalPos & 0xFF;
    uint8_t hi = (legServos[i].goalPos >> 8) & 0xFF;
    pkt[k++] = id; pkt[k++] = lo; pkt[k++] = hi;
    sum += id + lo + hi;
  }
  pkt[k++] = ~(sum & 0xFF) & 0xFF;
  Serial2.write(pkt, k);
  Serial2.flush();
  while (Serial2.available()) Serial2.read();   // drain our own echo
}

// ── POSE TRAJECTORY ──────────────────────────────────────────────────────────
// Sync-write alone makes the joints start together. It does NOT make the FOOT
// travel a straight line: handed only an endpoint, each servo races there at
// its own uncapped speed, so the foot bulges along the way. "Proportional"
// needs the FOOT TARGET stepped through intermediate points with the IK
// re-solved at every step — which is what this does, at the full 100 Hz.
uint16_t moveTimeMs = 800;              // MT<n>, 100-3000 ms
bool     moveActive = false;
unsigned long moveStartMs = 0;
float mv_x0[2], mv_y0[2];               // where the move started
float mv_x1[2], mv_y1[2];               // where it ends
float cur_x[2] = {BASE_FX1, BASE_FX2};  // where the foot is RIGHT NOW —
float cur_y[2] = {BASE_FY1, BASE_FY2};  // the authoritative pose state

// The endpoint implied by the active mode. Crouch and IK stay independent.
void computeDesiredFoot(float *fx, float *fy) {
  if (poseMode == MODE_CROUCH) {
    fx[0] = BASE_FX1;  fy[0] = BASE_FY1 + crouchOffset;
    fx[1] = BASE_FX2;  fy[1] = BASE_FY2 + crouchOffset;
  } else {
    fx[0] = ft_x[0];   fy[0] = ft_y[0];
    fx[1] = ft_x[1];   fy[1] = ft_y[1];
  }
}

// Solve both legs at the given foot targets and cache the counts. Does NOT
// touch the bus — ax12SyncWriteGoals() does that in one shot afterwards.
void solveGoalsFor(const float *fx, const float *fy) {
  IK_Result sol1 = solve_ik(fx[0], fy[0], 0.0f);
  IK_Result sol2 = solve_ik(fx[1] + IK_DIST, fy[1], IK_DIST);
  ikValid[0] = sol1.valid;
  ikValid[1] = sol2.valid;
  if (!sol1.valid || !sol2.valid) return;   // out of reach — hold last good pose

  uint16_t p6  = map_angle_to_ax12(sol1.Angle_L, true,  false);
  uint16_t p14 = map_angle_to_ax12(sol1.Angle_R, false, false);
  float ikL2 = sol2.Angle_L, ikR2 = sol2.Angle_R;
  if (LEG2_INVERTED_MOUNT) { ikL2 = -sol2.Angle_R; ikR2 = -sol2.Angle_L; }
  uint16_t p0 = map_angle_to_ax12(ikL2, true,  true);
  uint16_t p1 = map_angle_to_ax12(ikR2, false, true);

  for (int i = 0; i < 4; i++) {
    if      (legServos[i].id == 6)  legServos[i].goalPos = p6;
    else if (legServos[i].id == 14) legServos[i].goalPos = p14;
    else if (legServos[i].id == 0)  legServos[i].goalPos = p0;
    else if (legServos[i].id == 1)  legServos[i].goalPos = p1;
  }
}

// Begin an interpolated move from wherever the foot currently is to the pose
// the active mode now implies. Re-starting mid-flight is safe and seamless
// because the new move always starts from cur_*, never from the old endpoint.
void startPoseMove() {
  computeDesiredFoot(mv_x1, mv_y1);
  for (int i = 0; i < 2; i++) { mv_x0[i] = cur_x[i]; mv_y0[i] = cur_y[i]; }
  moveStartMs = millis();
  moveActive  = true;
}

// Jump straight to the commanded pose with no interpolation. Boot and SR only,
// where the pose is not actually changing and there is nothing to ease.
void snapPose() {
  computeDesiredFoot(cur_x, cur_y);
  solveGoalsFor(cur_x, cur_y);
  moveActive = false;
}

void updateMoveTask() {
  if (!moveActive) return;
  unsigned long elapsed = millis() - moveStartMs;
  float f = (moveTimeMs == 0) ? 1.0f : (float)elapsed / (float)moveTimeMs;
  if (f >= 1.0f) { f = 1.0f; moveActive = false; }

  // Smoothstep rather than a raw linear ramp. Both keep the joints perfectly
  // coordinated (that is the sync-write's job); smoothstep additionally starts
  // and stops at zero velocity, so the legs do not snap into motion at full
  // speed and slam to a halt at the end. For a strictly linear ramp, delete
  // this line and use f directly.
  float s = f * f * (3.0f - 2.0f * f);

  for (int i = 0; i < 2; i++) {
    cur_x[i] = mv_x0[i] + (mv_x1[i] - mv_x0[i]) * s;
    cur_y[i] = mv_y0[i] + (mv_y1[i] - mv_y0[i]) * s;
  }
  solveGoalsFor(cur_x, cur_y);
  ax12SyncWriteGoals();
}

// ── SETTINGS PUSH — one servo per tick (see settingsDirtyMask note) ──────────
void applyServoSettings(uint8_t idx) {
  uint8_t id = legServos[idx].id;
  ax12WriteWord(id, 34, g_torqueLimit);       // Torque Limit          (RAM)
  ax12WriteByte(id, 26, g_compMargin);        // CW  Compliance Margin (RAM)
  ax12WriteByte(id, 27, g_compMargin);        // CCW Compliance Margin (RAM)
  ax12WriteByte(id, 28, g_compSlope);         // CW  Compliance Slope  (RAM)
  ax12WriteByte(id, 29, g_compSlope);         // CCW Compliance Slope  (RAM)
  ax12WriteWord(id, 32, g_movingSpeed);       // Moving Speed          (RAM)
  ax12WriteByte(id, 24, g_torqueOn ? 1 : 0);  // Torque Enable         (RAM)
}

void applySettingsTask() {
  if (settingsDirtyMask == 0) return;
  for (uint8_t i = 0; i < 4; i++) {
    if (settingsDirtyMask & (1 << i)) {
      applyServoSettings(i);
      settingsDirtyMask &= ~(1 << i);
      return;                      // exactly one servo per tick
    }
  }
}

void initAX12Legs() {
  for (int i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    ax12WriteByte(id, 16, 1);      // Status Return Level (EEPROM — written once)
    ax12WriteByte(id, 5,  0);      // Return Delay Time = 0 (EEPROM — written once)
  }
  markAllSettingsDirty();          // all RAM registers pushed by applySettingsTask()
}

// ── SERVO POLL — non-blocking, one servo per 20 ms ───────────────────────────
// READ addr 36 len 8 -> Pos(2) Speed(2) Load(2) Volt(1) Temp(1).
// The control table is contiguous, so Voltage at addr 42 cannot be skipped on
// the way to Temp at 43 — it costs zero extra bus bytes and is the cleanest way
// to tell a sagging pack (AX-12 torque scales with supply volts) apart from
// badly tuned compliance. Both present as "weak servos".
//
// Reply is 14 B (FF FF ID LEN ERR + 8 params + CHK) and the half-duplex bus
// also echoes our own 8 B instruction, so we wait for 22.
enum PollState { POLL_IDLE, POLL_WAITING };
PollState     pollState       = POLL_IDLE;
unsigned long lastPollTime    = 0;
unsigned long waitStartTime   = 0;
uint8_t       currentServoIdx = 0;
const unsigned long POLL_INTERVAL_MS = 20;

void pollLegServosTask() {
  unsigned long now = millis();

  if (pollState == POLL_IDLE) {
    if (now - lastPollTime < POLL_INTERVAL_MS) return;
    ServoState &s = legServos[currentServoIdx];

    // Re-assert torque only (RAM, no EEPROM wear). Goal position is NOT written
    // here any more — that per-servo write was the source of the 80 ms intra-leg
    // stagger. Goals now belong exclusively to ax12SyncWriteGoals(), which hands
    // all four out in one packet. Skipped entirely while limp: writing to a
    // torque-disabled servo would make it lunge the instant torque came back.
    if (g_torqueOn) {
      ax12WriteByte(s.id, 24, 1);
      ax12WriteWord(s.id, 34, g_torqueLimit);
    }

    while (Serial2.available()) Serial2.read();   // flush write echoes

    uint8_t checksum = ~(s.id + 4 + 2 + 36 + 8) & 0xFF;
    uint8_t packet[] = {0xFF, 0xFF, s.id, 0x04, 0x02, 36, 8, checksum};
    Serial2.write(packet, 8);
    prof_rxReqUs = micros();   // profiler: start of the request->reply window

    pollState     = POLL_WAITING;
    waitStartTime = now;
  }
  else if (pollState == POLL_WAITING) {
    if (Serial2.available() >= 22) {              // 8-byte echo + 14-byte reply
      for (int i = 0; i < 8; i++) Serial2.read();
      uint8_t reply[14];
      for (int i = 0; i < 14; i++) reply[i] = Serial2.read();

      ServoState &s = legServos[currentServoIdx];
      if (reply[0] == 0xFF && reply[1] == 0xFF && reply[2] == s.id) {
        uint16_t posRaw   = (uint16_t)(reply[5]  | (reply[6]  << 8));
        uint16_t speedRaw = (uint16_t)(reply[7]  | (reply[8]  << 8));
        uint16_t loadRaw  = (uint16_t)(reply[9]  | (reply[10] << 8));
        uint8_t  voltRaw  = reply[11];
        uint8_t  tempRaw  = reply[12];

        // Speed and Load are magnitude in bits 0-9 with direction in bit 10.
        s.presentPos   = posRaw & 0x3FF;
        s.presentSpeed = (int16_t)((int)(speedRaw & 0x3FF) * ((speedRaw & 0x400) ? -1 : 1));
        s.loadPct      = ((loadRaw & 0x3FF) / 1023.0f) * 100.0f;
        s.volts        = voltRaw / 10.0f;
        s.temp         = tempRaw;
      }
      // Request -> reply consumed. Spans two ticks, so it is NOT part of any
      // single tick's body time; it measures the servo's own turnaround.
      prof_rxLatUs    = micros() - prof_rxReqUs;
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
    else if (now - waitStartTime > 20) {          // timeout — servo silent
      if (prof_rxTo < 0xFFFF) prof_rxTo++;
      while (Serial2.available()) Serial2.read();
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
  }
}

// ── STATE LINE ───────────────────────────────────────────────────────────────
// Doubles as the command ack: the GUI's sliders clear their pending-echo flag
// off this line, exactly like the pretest GUI does off "Updated ->".
void sendStateLine() {
  char line[192];
  int n = snprintf(line, sizeof(line),
    "STATE:MODE:%u,TQ:%u,TL:%u,CM:%u,CS:%u,MS:%u,MT:%u,CROUCH:%.2f,"
    "FX1:%.2f,FY1:%.2f,FX2:%.2f,FY2:%.2f,IK1:%u,IK2:%u\n",
    (unsigned)poseMode, (unsigned)(g_torqueOn ? 1 : 0),
    (unsigned)g_torqueLimit, (unsigned)g_compMargin,
    (unsigned)g_compSlope, (unsigned)g_movingSpeed,
    (unsigned)moveTimeMs, crouchOffset,
    ft_x[0], ft_y[0], ft_x[1], ft_y[1],
    (unsigned)(ikValid[0] ? 1 : 0), (unsigned)(ikValid[1] ? 1 : 0));
  if (n > 0 && n < (int)sizeof(line) && Serial3.availableForWrite() >= n)
    Serial3.write((uint8_t*)line, n);
}

// ── COMMAND PARSER ───────────────────────────────────────────────────────────
// EVERY command in this variant is a TWO-CHARACTER prefix. The pretest parser
// had to order multi-char cases before single-char ones (or "TE1" parsed as a
// tilt of atof("E1") = 0); here there are no single-char commands at all, so
// cmd[0]+cmd[1] disambiguates completely and that ordering hazard cannot exist.
//
// NOTE: this protocol is NOT interchangeable with the other tuner_legcontrol
// variants. "TE" means auto-trim-enable in mcu_ik_engine_pretest_wireless; here
// the master torque kill is "TQ" specifically so the two cannot be confused.
void parseCommand(char *cmd) {
  char ack[96];

  // PING:<token> -> PONG:<token>  (keeps latency_test.py working)
  if (cmd[0]=='P' && cmd[1]=='I' && cmd[2]=='N' && cmd[3]=='G') {
    snprintf(ack, sizeof(ack), "PONG%s", cmd + 4);
    radioLine(ack);
    return;
  }

  // -- per-servo raw goal position, e.g. "PS6 750" — bypasses IK entirely --
  if (cmd[0]=='P' && cmd[1]=='S') {
    char *sp = strchr(cmd + 2, ' ');
    if (sp) {
      int id  = atoi(cmd + 2);
      int pos = constrain(atoi(sp + 1), 0, 1023);
      bool found = false;
      for (int i = 0; i < 4; i++) {
        if (legServos[i].id == id) { legServos[i].goalPos = (uint16_t)pos; found = true; break; }
      }
      snprintf(ack, sizeof(ack), found ? "PS:OK ID%d POS%d" : "PS:ERR UNKNOWN_ID%d", id, pos);
    } else {
      snprintf(ack, sizeof(ack), "PS:ERR BAD_FORMAT");
    }
    radioLine(ack);
    return;
  }

  // -- global servo settings (all four, identical values) --
  else if (cmd[0]=='T' && cmd[1]=='L') {
    g_torqueLimit = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  else if (cmd[0]=='C' && cmd[1]=='M') {
    g_compMargin = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  else if (cmd[0]=='C' && cmd[1]=='S') {
    g_compSlope = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  else if (cmd[0]=='M' && cmd[1]=='S') {
    g_movingSpeed = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  // Duration of an interpolated pose move. Fixed TIME, not fixed speed: every
  // move takes this long regardless of how far the foot has to travel.
  else if (cmd[0]=='M' && cmd[1]=='T') {
    moveTimeMs = (uint16_t)constrain(atoi(cmd + 2), 100, 3000);
  }

  // -- master torque kill --
  // Re-gripping seeds every goalPos from that servo's last-read PRESENT
  // position. Without this, going limp, repositioning the legs by hand and
  // re-enabling would slam all four back to the stale pose at full torque —
  // with your hands still in the linkage. The operator re-commands a pose
  // deliberately afterwards.
  else if (cmd[0]=='T' && cmd[1]=='Q') {
    bool want = (cmd[2] == '1');
    if (want && !g_torqueOn) {
      for (int i = 0; i < 4; i++) legServos[i].goalPos = legServos[i].presentPos;
      // Re-seed the interpolator from the real pose via FK. cur_* still holds
      // the PRE-limp foot, so without this the next pose move would interpolate
      // from a pose the legs left minutes ago, and its very first sync-write
      // would slam them straight back to it.
      uint16_t p6 = 818, p14 = 441, p0 = 818, p1 = 441;
      for (int i = 0; i < 4; i++) {
        if      (legServos[i].id == 6)  p6  = legServos[i].presentPos;
        else if (legServos[i].id == 14) p14 = legServos[i].presentPos;
        else if (legServos[i].id == 0)  p0  = legServos[i].presentPos;
        else if (legServos[i].id == 1)  p1  = legServos[i].presentPos;
      }
      Point2D f;
      if (solve_fk(ax12_to_angle(p6, true, false),
                   ax12_to_angle(p14, false, false), 0.0f, f)) {
        cur_x[0] = f.x; cur_y[0] = f.y;
      }
      float ikL2 = ax12_to_angle(p0, true,  true);
      float ikR2 = ax12_to_angle(p1, false, true);
      float aL2 = ikL2, aR2 = ikR2;
      if (LEG2_INVERTED_MOUNT) { aL2 = -ikR2; aR2 = -ikL2; }
      if (solve_fk(aL2, aR2, IK_DIST, f)) {
        cur_x[1] = f.x - IK_DIST; cur_y[1] = f.y;
      }
      moveActive = false;
    }
    g_torqueOn = want;
    markAllSettingsDirty();
    snprintf(ack, sizeof(ack), "ACK:TORQUE_%s", g_torqueOn ? "ON" : "LIMP");
    radioLine(ack);
    sendStateLine();
    return;
  }

  // -- pose mode select (the two modes are mutually exclusive) --
  // A mode switch must be a ZERO-MOTION event. The two modes keep separate
  // state, so without seeding, MD1 would re-solve against ft_x/ft_y that were
  // last written while the robot was standing — and the legs would snap out of
  // a crouch the instant the operator flipped the radio button, with no Send
  // Pose click anywhere. So the incoming mode is seeded from the pose the
  // outgoing one is currently holding.
  else if (cmd[0]=='M' && cmd[1]=='D') {
    uint8_t want = (cmd[2] == '1') ? MODE_IK : MODE_CROUCH;
    if (want != poseMode) {
      if (want == MODE_IK) {
        // Crouch -> IK: the crouch pose IS a foot target, so copy it across
        // exactly. The resulting solve is bit-identical, so nothing moves.
        ft_x[0] = BASE_FX1;  ft_y[0] = BASE_FY1 + crouchOffset;
        ft_x[1] = BASE_FX2;  ft_y[1] = BASE_FY2 + crouchOffset;
      } else {
        // IK -> Crouch: crouch has only one DOF, so an arbitrary dragged pose
        // cannot be represented exactly. Take the mean vertical displacement,
        // which preserves body height and only gives up the horizontal offset.
        crouchOffset = 0.5f * ((ft_y[0] - BASE_FY1) + (ft_y[1] - BASE_FY2));
      }
    }
    poseMode = want;
    startPoseMove();
  }

  // -- CROUCH mode: single vertical knob, foot x pinned to calibrated default --
  else if (cmd[0]=='C' && cmd[1]=='R') {
    crouchOffset = atof(cmd + 2);
    if (poseMode == MODE_CROUCH) startPoseMove();
  }

  // -- IK mode: "FT<leg> <x> <y>", leg is 1 or 2, x/y in mm --
  else if (cmd[0]=='F' && cmd[1]=='T') {
    int leg = atoi(cmd + 2);
    char *s1 = strchr(cmd + 2, ' ');
    char *s2 = s1 ? strchr(s1 + 1, ' ') : NULL;
    if (s1 && s2 && (leg == 1 || leg == 2)) {
      ft_x[leg - 1] = atof(s1 + 1);
      ft_y[leg - 1] = atof(s2 + 1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      radioLine("FT:ERR BAD_FORMAT");
      return;
    }
  }

  // -- IK mode, BOTH legs atomically: "FA <x1> <y1> <x2> <y2>" --
  // The GUI's Send Pose uses this rather than two FT commands. At the 300 us RX
  // budget an 18-byte FT takes ~6 ticks to arrive, so two back-to-back would
  // start leg 2 moving ~60 ms after leg 1 AND restart the move timer mid-flight.
  // One command means both legs start on the same tick, exactly as CR does.
  else if (cmd[0]=='F' && cmd[1]=='A') {
    char *a = strchr(cmd + 2, ' ');
    char *b = a ? strchr(a + 1, ' ') : NULL;
    char *c = b ? strchr(b + 1, ' ') : NULL;
    char *d = c ? strchr(c + 1, ' ') : NULL;
    if (a && b && c && d) {
      ft_x[0] = atof(a + 1);  ft_y[0] = atof(b + 1);
      ft_x[1] = atof(c + 1);  ft_y[1] = atof(d + 1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      radioLine("FA:ERR BAD_FORMAT");
      return;
    }
  }

  // -- home: back to the calibrated standing pose in both modes --
  else if (cmd[0]=='H' && cmd[1]=='M') {
    crouchOffset = 0.0f;
    ft_x[0] = BASE_FX1; ft_y[0] = BASE_FY1;
    ft_x[1] = BASE_FX2; ft_y[1] = BASE_FY2;
    startPoseMove();
  }

  // -- re-init the bus (re-writes the two EEPROM regs + all RAM settings) --
  else if (cmd[0]=='S' && cmd[1]=='R') {
    initAX12Legs();
    snapPose();
    radioLine("ACK:SERVOS_RESET");
    sendStateLine();
    return;
  }

  // -- resync request: GUI asks for the full state after a (re)connect --
  else if (cmd[0]=='R' && cmd[1]=='B') { sendStateLine(); return; }

  else return; // unknown command

  sendStateLine();   // ack for every settings/pose command
}

// ── NON-BLOCKING RX — 300 us budget (verbatim from pretest_wireless) ─────────
// A byte at 115200 baud is 87 us, so a 40 us budget drains at most ONE byte per
// call. Combined with a 50 Hz call rate that made an 8-byte command take
// 8 x 20 ms = 160 ms to reach the parser, and let the UART FIFO overflow
// (silently corrupting commands) on any radio burst. 300 us is 3% of the 10 ms
// loop and drains ~3 bytes/tick; handleTelemetryRX() runs EVERY tick.
const unsigned long RX_BUDGET_US = 300;

static char    rxBuf[48];
static uint8_t rxLen = 0;

void handleTelemetryRX() {
  unsigned long rxStart = micros();
  while (Serial3.available() && (micros() - rxStart) < RX_BUDGET_US) {
    char c = (char)Serial3.read();
    if (c >= 'a' && c <= 'z') c -= 32;         // uppercase in place
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; parseCommand(rxBuf); rxLen = 0; }
    } else if (rxLen < 47) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  analogWriteResolution(8);
  delay(2000);                 // let AX-12 servos stabilise before UART traffic

  Serial6.begin(115200);       // wired profiler port, PA11 = TX (output only)
  Serial3.begin(115200);       // 3DR radio
  Serial2.begin(1000000);      // AX-12 bus

  // Drive motors hard off. This firmware never touches them again.
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);

  initAX12Legs();
  snapPose();
  ax12SyncWriteGoals();

  lastTime     = micros();
  lastPollTime = millis();

  radioLine("BOOT:OK AX12_CONTROL");

  // Legend for the raw per-tick rows. Printed once; setup() has no 10 ms
  // deadline so an unguarded blocking write is fine here.
  Serial6.println();
  Serial6.println("# ax12_control tick profiler - one row per 100 Hz tick, all us");
  Serial6.println("#   P  tick period            (target 10000)");
  Serial6.println("#   B  loop body total        <- budget consumed");
  Serial6.println("#   F  free time left of 10000");
  Serial6.println("#   R  handleTelemetryRX + parseCommand");
  Serial6.println("#   S  applySettingsTask      (servo register writes)");
  Serial6.println("#   U  AX-12 bus              (sync-write / poll TX+RX)");
  Serial6.println("#   T  telemetry build+write  (Serial1 / 3DR)");
  Serial6.println("#   L  servo read latency, request->reply (0 = none this tick)");
  Serial6.println("#   X  profiler cost itself, previous tick");
  Serial6.println("#   !OVR body exceeded 10000   !ERR servo timeout or dropped row");
}

// ── MAIN LOOP (100 Hz) ───────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastTime < TICK_US) return;   // enforce 100 Hz
  uint32_t d_period = (uint32_t)(now - lastTime);
  lastTime = now;

  uint32_t t_body = micros();
  uint32_t mark   = t_body;

  // Uplink commands are latency-critical, so RX runs every 10 ms tick.
  handleTelemetryRX();
  uint32_t d_rx = micros() - mark;   // includes parseCommand() and its acks

  mark = micros();
  // At most one servo's settings pushed per tick (~630 us), never all four.
  // Gated on POLL_IDLE for the same reason the hold sync-write is: these are
  // 7 blocking writes, and their ~60 bytes of half-duplex echo landing while a
  // health read is in flight would satisfy the poll's "available() >= 22" with
  // garbage. The poll's own pre-read flush would resync it a slot later, but
  // the read is lost — and dragging a compliance slider would silently punch
  // holes in the health readout exactly when you are watching it.
  if (pollState == POLL_IDLE) applySettingsTask();
  uint32_t d_set = micros() - mark;

  mark = micros();
  // ── BUS ARBITRATION ──────────────────────────────────────────────────────
  // Exactly ONE transaction owns Serial2 per tick. This matters more than it
  // looks: the health poll spans two ticks (request, then reply), and a
  // SYNC_WRITE issued in between would push its own half-duplex echo into the
  // 22-byte reply the poll is counting, desynchronising the read forever.
  if (moveActive) {
    // A move is in flight — it takes the bus outright at the full 100 Hz so
    // the foot follows its interpolated path smoothly. Health readback is
    // suspended for the (sub-3 s) duration; a pending read is abandoned
    // cleanly rather than left half-consumed.
    if (pollState == POLL_WAITING) {
      while (Serial2.available()) Serial2.read();
      pollState    = POLL_IDLE;
      lastPollTime = millis();
    }
    updateMoveTask();
  } else {
    // Idle: re-assert the held pose at 10 Hz with one sync-write. This
    // replaces the per-servo goal write the poll used to do, and is both
    // faster to come round (100 ms vs the 160 ms round-robin) and free of
    // stagger. Gated on POLL_IDLE so it can never land mid-read.
    static unsigned long lastHoldUs = 0;
    if (g_torqueOn && pollState == POLL_IDLE && now - lastHoldUs >= 100000) {
      lastHoldUs = now;
      ax12SyncWriteGoals();
    }

    // Servo poll keeps the pretest's 50 Hz call slot; pollLegServosTask()
    // already rate-limits itself to POLL_INTERVAL_MS internally.
    static bool isReadCycle = false;
    isReadCycle = !isReadCycle;
    if (isReadCycle) pollLegServosTask();
  }
  uint32_t d_bus = micros() - mark;   // ALL Serial2 traffic for this tick

  mark = micros();
  // ── TELEMETRY @ 10 Hz — ONE short line per frame ────────────────────────
  // Same downlink budget as the balancing firmware. The 3DR/SiK link is
  // half-duplex TDM: each end only gets ~half the airtime, so a fat downlink
  // leaves no window for the ground unit to transmit and commands sit in the
  // radio's buffer for seconds. One servo per frame round-robin => each servo
  // refreshes at 2.5 Hz, which is plenty for temp/load/voltage.
  if (now - lastPrintTime >= 100000) {
    lastPrintTime = now;

    static uint8_t txIdx = 0;
    ServoState &s = legServos[txIdx];
    char line[160];
    int n = snprintf(line, sizeof(line),
      "SRV:%u,POS:%u,GOAL:%u,SPD:%d,LOAD:%.1f,TEMP:%u,VOLT:%.1f\n",
      (unsigned)s.id, (unsigned)s.presentPos, (unsigned)s.goalPos,
      (int)s.presentSpeed, s.loadPct, (unsigned)s.temp, s.volts);
    if (n > 0 && n < (int)sizeof(line) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)line, n);
    txIdx = (txIdx + 1) % 4;

    // Full state at 1 Hz as a keepalive resync, on top of the per-command acks.
    static unsigned long lastState = 0;
    if (now - lastState >= 1000000) { lastState = now; sendStateLine(); }
  }
  uint32_t d_tel = micros() - mark;

  // ── PROFILER EMIT — every tick, raw, on Serial1 ──────────────────────────
  // B is sampled HERE, before anything below runs, so it is the true body cost
  // with the instrumentation excluded. F is what is left of the 10 ms.
  uint32_t d_body = micros() - t_body;
  int32_t  d_free = (int32_t)TICK_US - (int32_t)d_body;

  uint32_t t_prof = micros();
  {
    char pl[96];
    int n = snprintf(pl, sizeof(pl),
      "P%lu B%lu F%ld R%lu S%lu U%lu T%lu L%lu X%lu%s%s\n",
      (unsigned long)d_period, (unsigned long)d_body, (long)d_free,
      (unsigned long)d_rx,  (unsigned long)d_set,
      (unsigned long)d_bus, (unsigned long)d_tel,
      (unsigned long)prof_rxLatUs, (unsigned long)prof_printUs,
      (d_body > TICK_US) ? " !OVR" : "",
      (prof_rxTo || prof_dropped) ? " !ERR" : "");

    // Guarded exactly like the radio path — the profiler must never be the
    // thing that stalls the loop it is measuring.
    if (n > 0 && n < (int)sizeof(pl) && Serial6.availableForWrite() >= n) {
      Serial6.write((uint8_t *)pl, n);
      prof_rxTo = 0; prof_dropped = 0;
    } else {
      if (prof_dropped < 0xFFFF) prof_dropped++;
    }
    prof_rxLatUs = 0;   // latency is reported on the tick the reply landed
  }
  prof_printUs = micros() - t_prof;
}
