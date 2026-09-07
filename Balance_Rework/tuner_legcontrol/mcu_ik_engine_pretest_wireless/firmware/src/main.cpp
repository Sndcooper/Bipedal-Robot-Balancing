// ============================================================================
// mcu_ik_engine_pretest_wireless — MINIMAL SINGLE-LOOP WIRELESS BALANCER
// STM32 Bluepill F103C8 | 3DR telemetry Serial3 (PB10/PB11 @ 115200)
// AX-12 bus Serial2 (PA2/PA3 @ 1 Mbaud) | MPU6050 I2C1 (PB6/PB7)
// ----------------------------------------------------------------------------
// Deliberately ONE control loop only: a single balance PID (Kp,Ki,Kd) acting on
// complementary-filtered pitch. No cascade (velocity/position), no IK, no RC,
// no steering. Legs are held in the calibrated standing pose and polled for
// health only. Encoder velocity is computed for DISPLAY telemetry and is not
// fed into control EXCEPT by the opt-in auto-trim add-on below (off by
// default), which nudges the trim bias to null steady drift instead of
// requiring a hand re-trim after every gain change. A tiny PING->PONG
// responder is kept so latency_test.py works.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>

// ── ENCODER PINS (velocity telemetry only — not used for control) ────────────
#define ENC_L_A PA6
#define ENC_L_B PA7
#define ENC_R_A PB0
#define ENC_R_B PB1

volatile long encoderLeft  = 0;
volatile long encoderRight = 0;
long prevEncoderLeft  = 0;
long prevEncoderRight = 0;

void countLeft()  { if (digitalRead(ENC_L_B)) encoderLeft--;  else encoderLeft++;  }
void countRight() { if (digitalRead(ENC_R_B)) encoderRight--; else encoderRight++; }

// ── MOTOR PINS (L298N) ───────────────────────────────────────────────────────
#define ENA PA1
#define IN1 PB14
#define IN2 PB15
#define ENB PA0
#define IN3 PB12
#define IN4 PB13

// ── SERIAL PORTS (instantiated via build_flags) ──────────────────────────────
extern HardwareSerial Serial1;   // tick profiler out (PA9 TX) — OUTPUT ONLY
extern HardwareSerial Serial2;   // AX-12 bus
extern HardwareSerial Serial3;   // 3DR radio

// ============================================================================
// TICK PROFILER — how much of the 10 ms is consumed, and by what
// ----------------------------------------------------------------------------
// INSTRUMENTATION ONLY. Not one line of control logic below is altered by this
// block: no stage is reordered, no timing is changed, nothing is optimised. The
// numbers therefore describe the firmware you already trust, which is the whole
// point of measuring it rather than rewriting it.
//
// Output is Serial1 (PA9 = TX) at 115200 — a plain wired UART, deliberately NOT
// the 3DR radio. Sending profiling data over the link whose airtime starvation
// you are trying to characterise would perturb the very thing being measured.
//
// TWO STREAMS
//   1. one raw row per 100 Hz tick, so every stage's cost is visible per-tick
//   2. a 1 Hz report: worst 5 ticks, best tick, mean, and modal bucket, ending
//      in the WORST-CASE FREE microseconds — the honest budget for adding RC
//
// The report is ~11 lines but the UART TX ring is only 256 B, so it can never
// be written in one tick. It is emitted ONE LINE PER TICK across the following
// ticks instead, with the raw row suppressed while that runs (those ticks are
// still counted in the statistics — only their printing is skipped). This is
// why nothing here can ever block: every write is a single line, guarded.
// ============================================================================
#define PROF_STAGES 8
// Column letters, in loop order. Keep in sync with PROF_KEYS below.
//   I readIMU (I2C)   K calibrationTask   Y safety cutoff   E encoder->velocity
//   C balance PID + setMotors             R telemetryRX+parse
//   V pollLegServosTask (AX-12 bus)       T telemetry block
static const char PROF_KEYS[PROF_STAGES + 1] = "IKYECRVT";

uint16_t prof_stage[PROF_STAGES];       // this tick's per-stage microseconds

// --- 1 s accumulators -------------------------------------------------------
uint32_t prof_n       = 0;              // ticks this window
uint32_t prof_sumBody = 0;
uint16_t prof_best    = 0xFFFF;
uint16_t prof_bestS[PROF_STAGES];
uint16_t prof_worst [5];                // body us, sorted descending
uint16_t prof_worstS[5][PROF_STAGES];
uint16_t prof_ovr = 0, prof_rxTo = 0, prof_drop = 0;

// Modal bucket: 100 us bins across 0..6.3 ms, last bin catches everything above.
#define PROF_BINS 64
uint16_t prof_hist[PROF_BINS];

uint32_t prof_printUs = 0;              // previous tick's instrumentation cost
int8_t   prof_report  = -1;             // >=0 while a report is being emitted

// Snapshot the window so the report can be emitted over the following ticks
// while a fresh window is already accumulating.
uint32_t rep_n, rep_sum;
uint16_t rep_best, rep_bestS[PROF_STAGES];
uint16_t rep_worst[5], rep_worstS[5][PROF_STAGES];
uint16_t rep_modeBin, rep_modeCount, rep_ovr, rep_rxTo, rep_drop;

void profResetWindow() {
  prof_n = 0; prof_sumBody = 0; prof_best = 0xFFFF;
  for (int i = 0; i < 5; i++) prof_worst[i] = 0;
  for (int i = 0; i < PROF_BINS; i++) prof_hist[i] = 0;
  prof_ovr = 0; prof_rxTo = 0; prof_drop = 0;
}

void profAccumulate(uint16_t body) {
  prof_n++;
  prof_sumBody += body;
  if (body > 10000 && prof_ovr < 0xFFFF) prof_ovr++;

  uint16_t bin = body / 100;
  if (bin >= PROF_BINS) bin = PROF_BINS - 1;
  if (prof_hist[bin] < 0xFFFF) prof_hist[bin]++;

  if (body < prof_best) {
    prof_best = body;
    for (int i = 0; i < PROF_STAGES; i++) prof_bestS[i] = prof_stage[i];
  }
  // Top-5 insertion sort, descending. Five compares worst case, ~1 us.
  for (int i = 0; i < 5; i++) {
    if (body > prof_worst[i]) {
      for (int j = 4; j > i; j--) {
        prof_worst[j] = prof_worst[j - 1];
        for (int k = 0; k < PROF_STAGES; k++)
          prof_worstS[j][k] = prof_worstS[j - 1][k];
      }
      prof_worst[i] = body;
      for (int k = 0; k < PROF_STAGES; k++) prof_worstS[i][k] = prof_stage[k];
      break;
    }
  }
}

void profSnapshotReport() {
  rep_n   = prof_n ? prof_n : 1;
  rep_sum = prof_sumBody;
  rep_best = (prof_best == 0xFFFF) ? 0 : prof_best;
  for (int i = 0; i < PROF_STAGES; i++) rep_bestS[i] = prof_bestS[i];
  for (int i = 0; i < 5; i++) {
    rep_worst[i] = prof_worst[i];
    for (int k = 0; k < PROF_STAGES; k++) rep_worstS[i][k] = prof_worstS[i][k];
  }
  rep_modeBin = 0; rep_modeCount = 0;
  for (int i = 0; i < PROF_BINS; i++)
    if (prof_hist[i] > rep_modeCount) { rep_modeCount = prof_hist[i]; rep_modeBin = i; }
  rep_ovr = prof_ovr; rep_rxTo = prof_rxTo; rep_drop = prof_drop;
  prof_report = 0;
  profResetWindow();
}

// Append " I1340 K0 Y0 ..." for one stage vector.
int profStages(char *buf, int cap, const uint16_t *s) {
  int n = 0;
  for (int i = 0; i < PROF_STAGES && n < cap - 12; i++)
    n += snprintf(buf + n, cap - n, " %c%u", PROF_KEYS[i], (unsigned)s[i]);
  return n;
}

// One report line per call. Returns false when the report is finished.
bool profReportLine(char *b, int cap) {
  int n = 0;
  uint32_t mean = rep_sum / rep_n;
  switch (prof_report) {
    case 0:
      snprintf(b, cap, "=== BUDGET 1s: %lu ticks x 10000us ===",
               (unsigned long)rep_n);
      break;
    case 1:
      snprintf(b, cap, "  mean B%luus %lu.%lu%% free %luus",
               (unsigned long)mean, (unsigned long)(mean / 100),
               (unsigned long)((mean / 10) % 10), (unsigned long)(10000 - mean));
      break;
    case 2:
      n = snprintf(b, cap, "  best B%uus free %uus",
                   (unsigned)rep_best, (unsigned)(10000 - rep_best));
      profStages(b + n, cap - n, rep_bestS);
      break;
    case 3:
      snprintf(b, cap, "  mode B%u-%uus (%u of %lu ticks)",
               (unsigned)(rep_modeBin * 100), (unsigned)(rep_modeBin * 100 + 99),
               (unsigned)rep_modeCount, (unsigned long)rep_n);
      break;
    case 4: case 5: case 6: case 7: case 8: {
      int i = prof_report - 4;
      n = snprintf(b, cap, "  w%d B%uus %lu.%lu%%", i + 1, (unsigned)rep_worst[i],
                   (unsigned long)(rep_worst[i] / 100),
                   (unsigned long)((rep_worst[i] / 10) % 10));
      profStages(b + n, cap - n, rep_worstS[i]);
      break;
    }
    case 9:
      // The number that answers "how much room is left for RC": not the mean,
      // the WORST tick. A stage that fits on average but not on the worst tick
      // is a stage that overruns the loop under load.
      snprintf(b, cap, "  WORST-CASE FREE %uus (%lu.%lu%%) <- RC budget",
               (unsigned)(10000 - rep_worst[0]),
               (unsigned long)((10000 - rep_worst[0]) / 100),
               (unsigned long)(((10000 - rep_worst[0]) / 10) % 10));
      break;
    case 10:
      snprintf(b, cap, "  ovr %u  servo_timeout %u  rows_dropped %u",
               (unsigned)rep_ovr, (unsigned)rep_rxTo, (unsigned)rep_drop);
      break;
    default:
      return false;
  }
  return true;
}

// ── MPU6050 ───────────────────────────────────────────────────────────────────
const int MPU_ADDR = 0x68;
float pitch = 0.0f, pitchOffset = 0.0f;
float accelPitchRaw = 0.0f;
float gyroRate = 0.0f;
#define GYRO_PITCH_SIGN 1.0f     // flip to -1.0f if pitch runs the wrong way

// ── THE SINGLE BALANCE PID (the ONLY control loop) ───────────────────────────
float Kp = 78.0f, Ki = 0.0f, Kd = 0.0f;   // neutral bench-tuning start
float integral = 0.0f;
float alpha = 0.96f;                        // complementary-filter coefficient
float targetAngle = 0.0f;                   // balance setpoint (operator trim)
float maxSafeTilt = 25.0f;                  // safety cutoff threshold (deg)
const float MAX_INTEGRAL_PWM = 1200.0f;     // anti-windup: cap Ki term's PWM (10x, per request)

// ── ENCODER VELOCITY (telemetry display; also feeds auto-trim below) ────────
float vel_current = 0.0f;
float vel_alpha   = 0.85f;

// ── AUTO-TRIM (opt-in drift-cancelling bias) ─────────────────────────────────
// Ports the RC_mcu_IK_wireless velocity-integral idea (same sign convention
// and same order-of-magnitude gain as its Ki_vel) into this single-loop
// firmware: while enabled, any steady encL/encR drift means targetAngle is
// off from the true balance point, so integrate vel_current toward zero and
// add the result on top of targetAngle. "TC" then bakes the converged bias
// into targetAngle permanently, replacing a manual re-trim.
bool  autoTrimEnabled = false;          // off by default — opt in with TE1
float Ki_trim         = 0.001f;         // deg of bias per (count/s) per second — TG cmd
float trim_bias       = 0.0f;           // current auto-trim contribution (deg)
const float MAX_TRIM_BIAS = 15.0f;      // anti-windup clamp (deg) — widened 3x, capped
                                         // well under maxSafeTilt (25°) on purpose: see note below.

bool motorsEnabled = false;
bool safetyLatched = false;

// ── LOOP TIMING ───────────────────────────────────────────────────────────────
unsigned long lastTime      = 0;
unsigned long lastPrintTime = 0;

// ── AX-12 HELPERS ─────────────────────────────────────────────────────────────
// Shared by every write/read path below — one place to discard whatever our own
// half-duplex echo (or a timed-out reply) left sitting in Serial2's RX buffer.
void ax12DrainRx() { while (Serial2.available()) Serial2.read(); }

// Status Return Level is set to 1 (reply to READ only) in initAX12Legs(), so a
// WRITE never gets a status packet back — only the half-duplex echo of the
// bytes we just sent. Draining it HERE, once, means every caller (initAX12Legs,
// applyServoSettings, ...) automatically leaves Serial2's RX buffer clean for
// whatever runs next in the tick, instead of each call site having to remember
// to do it (applyServoSettings() didn't, which desynced pollLegServosTask()'s
// byte-counted READ parser whenever a settings push and a health poll landed
// in the same or adjacent ticks).
void ax12WriteByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t checksum = ~(id + 4 + 3 + addr + val) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x03, addr, val, checksum};
  Serial2.write(packet, 8);
  Serial2.flush();       // TX drained: bus can turn around
  ax12DrainRx();          // RX drained: our own echo, discarded
}

void ax12WriteWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t checksum = ~(id + 5 + 3 + addr + lo + hi) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x05, 0x03, addr, lo, hi, checksum};
  Serial2.write(packet, 9);
  Serial2.flush();
  ax12DrainRx();
}

// ── LEG SERVO STATE (held pose + health only, no IK) ─────────────────────────
struct ServoState {
  uint8_t  id;
  uint16_t goalPos;      // calibrated standing pose
  uint16_t torqueLimit;
  uint8_t  compMargin;
  uint8_t  compSlope;
  uint8_t  temp;
  float    loadPct;
};

// Torque/compliance restored to the responsive baseline (was 511 / margin 4 /
// slope 32). Slope 32 is an 8x wider proportional band and margin 4 a wide
// deadband, which together make the servo ease in slowly and hold weakly — the
// joint visibly creeps toward its target under the robot's own weight, which
// reads as "slow servos" independently of any serial latency.
//   torqueLimit 511 -> 1023 : full holding torque (see note: doubles jam force)
//   compMargin     4 -> 1   : narrow deadband, reacts to small errors
//   compSlope     32 -> 4   : tight proportional band, no sluggish ease-in
ServoState legServos[4] = {
  {6,  818, 1023, 1, 4, 0, 0.0f},   // Leg1 Left  (818 = straight-down left)
  {0,  818, 1023, 1, 4, 0, 0.0f},   // Leg2 Left
  {14, 441, 1023, 1, 4, 0, 0.0f},   // Leg1 Right (441 = straight-down right)
  {1,  441, 1023, 1, 4, 0, 0.0f},   // Leg2 Right
};

// ── GLOBAL SERVO SETTINGS (ported from ax12_control — applies to all four) ───
// Identical to what the balancing firmware hard-codes in initAX12Legs() but now
// live-tunable from the GUI without reflashing. markAllSettingsDirty() queues
// an applySettingsTask() spread — one servo per 100 Hz tick — so a compliance
// slider drag costs ~630 µs (4 ticks) instead of 2.5 ms in one shot.
uint16_t g_torqueLimit = 1023;   // addr 34, 0-1023  (1023 = full holding torque)
uint8_t  g_compMargin  = 1;      // addr 26/27, 0-254 (narrow deadband)
uint8_t  g_compSlope   = 4;      // addr 28/29, 0-254 (tight proportional band)
uint16_t g_movingSpeed = 0;      // addr 32, 0-1023  (0 = uncapped, servos race at max speed)
bool     g_torqueOn    = true;   // addr 24 master kill: false = limp legs

// Dirty bitmask — one bit per legServos[] index. Set by any settings command,
// drained one servo per tick by applySettingsTask(). Prevents the 2.5 ms
// blocking bath that would quarter the loop's free time.
uint8_t settingsDirtyMask = 0x0F;  // push all registers at first boot tick
void markAllSettingsDirty() { settingsDirtyMask = 0x0F; }

// ── POSE TRAJECTORY ENGINE (ported verbatim from ax12_control) ───────────────
// Two mutually exclusive pose modes. In CROUCH the foot x is pinned to the
// calibrated default and one knob slides it vertically; in IK the dragged (x,y)
// is used as-is. Only CROUCH is available when the balance PID is active (it
// maps 1-to-1 with the existing crouchOffset path). IK is for bench posing.
#define MODE_CROUCH 0
#define MODE_IK     1
uint8_t  poseMode   = MODE_CROUCH;

uint16_t moveTimeMs = 800;              // MT<n>, 100-3000 ms per move
bool     moveActive = false;
unsigned long moveStartMs = 0;
float mv_x0[2], mv_y0[2];               // where the move started
float mv_x1[2], mv_y1[2];               // where it ends
float cur_x[2] = {1.0f, -6.0f};         // authoritative current foot position (mm)
float cur_y[2] = {-151.1f, -149.6f};
float ft_x[2]  = {1.0f, -6.0f};         // IK-mode commanded foot targets (mm)
float ft_y[2]  = {-151.1f, -149.6f};
bool  ikValid[2] = {true, true};         // last solve was in-reach?

// ── CROUCH IK (opt-in, one degree of freedom: stand tall <-> crouch) ────────
// Ports the 5-bar solve_ik/map_angle_to_ax12 pair from mcu_ik_engine_wireless
// verbatim (same mounts, same 818/441 calibration) so the math is proven, not
// re-derived. No lean/turn here on purpose — this variant only needs a single
// "how low is the body over the wheels" knob, driven by the CR<f> command.
#define SERVO_L_X -30.0f
#define SERVO_L_Y   0.0f
#define SERVO_R_X  30.0f
#define SERVO_R_Y   0.0f
#define FEMUR_LEN  55.0f
#define TIBIA_LEN 100.0f
#define LEG2_INVERTED_MOUNT true

const float ik_fx1 = 1.0f,  ik_fy1 = -151.1f;   // Leg1 standing foot target (mm)
const float ik_fx2 = -6.0f, ik_fy2 = -149.6f;   // Leg2 standing foot target (mm)
const float ik_dist = 180.0f;                    // leg separation (mm)
float crouchOffset = 0.0f;   // CR command: 0 = standing (default), + = crouched (mm)

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

void initAX12Legs() {
  for (int i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    ax12WriteByte(id, 16, 1);                         // Status Return Level (EEPROM, once)
    ax12WriteByte(id, 5,  0);                         // Return Delay Time = 0 (EEPROM, once)
    ax12WriteByte(id, 24, 1);                         // Torque Enable
    ax12WriteWord(id, 34, legServos[i].torqueLimit);  // Torque Limit
    ax12WriteByte(id, 26, legServos[i].compMargin);   // CW  Compliance Margin
    ax12WriteByte(id, 27, legServos[i].compMargin);   // CCW Compliance Margin
    ax12WriteByte(id, 28, legServos[i].compSlope);    // CW  Compliance Slope
    ax12WriteByte(id, 29, legServos[i].compSlope);    // CCW Compliance Slope
    ax12WriteWord(id, 30, legServos[i].goalPos);      // Goal Position (standing pose)
  }
}

// ── PRESENT POSITION READ (blocking) — TQ1 re-grip only, never the hot path ──
// Reads addr 36 len 2 (Present Position). Blocking is fine here: this runs
// synchronously in response to one deliberate operator command (TQ1), not on
// every 100 Hz tick like pollLegServosTask() — a 20 ms/servo worst case is
// nothing next to the human reaction time behind the command that triggered it.
bool ax12ReadPresentPos(uint8_t id, uint16_t &posOut) {
  // No pre-clear needed: ax12WriteByte/Word now drain their own echo (see
  // above), so Serial2's RX buffer is already quiet by the time this runs.
  uint8_t checksum = ~(id + 4 + 2 + 36 + 2) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x02, 36, 2, checksum};
  Serial2.write(packet, 8);
  Serial2.flush();

  unsigned long start = millis();                // 8-byte echo + 8-byte reply
  while (Serial2.available() < 16) {
    if (millis() - start >= 20) {                 // servo silent — give up
      ax12DrainRx();                              // caller keeps last goalPos
      return false;
    }
  }

  uint8_t buf[16];
  for (uint8_t i = 0; i < 16; i++) buf[i] = Serial2.read();
  if (buf[8] != 0xFF || buf[9] != 0xFF || buf[10] != id) return false;
  posOut = buf[13] | ((uint16_t)buf[14] << 8);
  return true;
}

// ── FORWARD KINEMATICS — for safe re-grip after limp ─────────────────────────
// Exact inverse of map_angle_to_ax12(). Only needed when the operator has
// moved the legs by hand (TQ0 limp -> reposition -> TQ1 re-grip). Without FK,
// the first sync-write of the next move would jump back to the pre-limp pose.
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

// ── SYNC_WRITE — all four goal positions in ONE 20-byte bus packet ────────────
// THE FIX for the 80 ms intra-leg stagger: one broadcast hands every servo its
// goal simultaneously. Being broadcast (ID 0xFE) there is no status reply —
// only the half-duplex echo of our own bytes, drained here.
void ax12SyncWriteGoals() {
  uint8_t pkt[20];
  pkt[0] = 0xFF; pkt[1] = 0xFF;
  pkt[2] = 0xFE;                      // broadcast ID
  pkt[3] = (2 + 1) * 4 + 4;          // LENGTH = (data_len+1)*n_servos + 4 = 16
  pkt[4] = 0x83;                      // SYNC_WRITE opcode
  pkt[5] = 30;                        // start address: Goal Position (RAM)
  pkt[6] = 2;                         // bytes per servo
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
  ax12DrainRx();                                 // drain our own half-duplex echo
}

// ── POSE TRAJECTORY HELPERS ──────────────────────────────────────────────────
void computeDesiredFoot(float *fx, float *fy) {
  if (poseMode == MODE_CROUCH) {
    fx[0] = ik_fx1;  fy[0] = ik_fy1 + crouchOffset;
    fx[1] = ik_fx2;  fy[1] = ik_fy2 + crouchOffset;
  } else {
    fx[0] = ft_x[0]; fy[0] = ft_y[0];
    fx[1] = ft_x[1]; fy[1] = ft_y[1];
  }
}

// Solve IK for both legs and cache counts into legServos[].goalPos.
// Does NOT touch the bus — ax12SyncWriteGoals() does that atomically.
void solveGoalsFor(const float *fx, const float *fy) {
  IK_Result sol1 = solve_ik(fx[0], fy[0], 0.0f);
  IK_Result sol2 = solve_ik(fx[1] + ik_dist, fy[1], ik_dist);
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

// Begin an interpolated move from current foot to new target. Safe to restart
// mid-flight: always departs from cur_*, never from the stale old endpoint.
void startPoseMove() {
  computeDesiredFoot(mv_x1, mv_y1);
  for (int i = 0; i < 2; i++) { mv_x0[i] = cur_x[i]; mv_y0[i] = cur_y[i]; }
  moveStartMs = millis();
  moveActive  = true;
}

// Snap to target immediately — no interpolation. Used at boot and on SR.
void snapPose() {
  computeDesiredFoot(cur_x, cur_y);
  solveGoalsFor(cur_x, cur_y);
  moveActive = false;
}

// 100 Hz interpolation tick: advance the foot along a smoothstep curve and
// broadcast the new IK solution to all four servos simultaneously.
void updateMoveTask() {
  if (!moveActive) return;
  unsigned long elapsed = millis() - moveStartMs;
  float f = (moveTimeMs == 0) ? 1.0f : (float)elapsed / (float)moveTimeMs;
  if (f >= 1.0f) { f = 1.0f; moveActive = false; }

  // Smoothstep: starts/ends at zero velocity so legs never snap into or slam
  // out of motion even at full torque. For a linear ramp use f directly.
  float s = f * f * (3.0f - 2.0f * f);

  for (int i = 0; i < 2; i++) {
    cur_x[i] = mv_x0[i] + (mv_x1[i] - mv_x0[i]) * s;
    cur_y[i] = mv_y0[i] + (mv_y1[i] - mv_y0[i]) * s;
  }
  solveGoalsFor(cur_x, cur_y);
  ax12SyncWriteGoals();
}

// ── SETTINGS PUSH — one servo per tick, never batched ─────────────────────────
// Applies g_torqueLimit/compMargin/compSlope/movingSpeed/torqueOn to one servo
// per tick (identified by settingsDirtyMask). 7 register writes ≈ 630 µs — a
// quarter of the tick — which is why they are spread rather than sent all at once.
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
      return;                    // exactly one servo per tick
    }
  }
}

// ── SERVO HEALTH POLL — non-blocking, one servo per 20 ms ─────────────────────
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

    // ── REMOVED: per-servo register writes from the poll slot ──────────────
    // Goal position:  owned by ax12SyncWriteGoals() — one SYNC_WRITE packet
    //                 broadcasts all 4 goals simultaneously. Writing goalPos
    //                 here individually reintroduces the 80 ms stagger.
    // Torque/settings: owned by applySettingsTask() via settingsDirtyMask.
    //                  Writing s.torqueLimit / s.compMargin here (hardcoded
    //                  struct fields) overrides g_torqueLimit / g_compMargin
    //                  set by the GUI slider, and forces torque ON even after
    //                  a TQ0 (limp) command — the exact bugs reported.
    // The poll is now READ-ONLY: fire the request, parse the reply, nothing more.



    // Every write path already drains its own echo (see ax12DrainRx()), so
    // this is normally a no-op — but it's cheap insurance against a stray
    // byte from EMI/noise on the bus landing here and shifting the fixed
    // 8-echo/10-reply byte count this state machine relies on below.
    ax12DrainRx();

    // READ addr 40 len 4 → Load(2B) Volt(1B) Temp(1B)
    uint8_t checksum = ~(s.id + 4 + 2 + 40 + 4) & 0xFF;
    uint8_t packet[] = {0xFF, 0xFF, s.id, 0x04, 0x02, 40, 4, checksum};
    Serial2.write(packet, 8);

    pollState     = POLL_WAITING;
    waitStartTime = now;
  }
  else if (pollState == POLL_WAITING) {
    if (Serial2.available() >= 18) {              // 8-byte echo + 10-byte reply
      for (int i = 0; i < 8; i++) Serial2.read();
      uint8_t reply[10];
      for (int i = 0; i < 10; i++) reply[i] = Serial2.read();

      if (reply[0] == 0xFF && reply[1] == 0xFF &&
          reply[2] == legServos[currentServoIdx].id) {
        uint16_t loadRaw = reply[5] | (reply[6] << 8);
        uint8_t  temp    = reply[8];
        legServos[currentServoIdx].temp    = temp;
        legServos[currentServoIdx].loadPct = ((loadRaw & 0x3FF) / 1023.0f) * 100.0f;
      }
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
    else if (now - waitStartTime > 20) {          // timeout — servo silent
      ax12DrainRx();
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
  }
}

// ── IMU ───────────────────────────────────────────────────────────────────────
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readIMU(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)12, (uint8_t)true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  (void)(Wire.read() << 8 | Wire.read()); // temp
  (void)(Wire.read() << 8 | Wire.read()); // gyroX
  int16_t gy = Wire.read() << 8 | Wire.read();

  accelPitchRaw = atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / PI;
  float accelPitch = accelPitchRaw - pitchOffset;
  gyroRate = GYRO_PITCH_SIGN * (float)gy / 131.0f;
  pitch = alpha * (pitch + gyroRate * dt) + (1.0f - alpha) * accelPitch;
}

// ── IMU CALIBRATION — non-blocking state machine ─────────────────────────────
// Was a blocking `for (100) { readIMU(); delay(10); }` — a full 1000 ms with
// the loop dead: no RX (so commands sent during/just after a calibration were
// delayed a whole second and often lost to UART FIFO overflow), no motor
// update, no telemetry. Same 100 samples over the same ~1 s, but now one
// sample per 100 Hz tick with the loop still servicing everything else.
// readIMU() is already called once per tick by loop(), so this only
// accumulates — it must run AFTER readIMU() in the tick.
const int CAL_SAMPLES = 100;
bool  calibrating   = false;
int   calSampleIdx  = 0;
float calSum        = 0.0f;

void startCalibration() {
  calibrating  = true;
  calSampleIdx = 0;
  calSum       = 0.0f;
  Serial3.println("CAL:START");
}

void calibrationTask() {
  if (!calibrating) return;

  calSum += accelPitchRaw;          // this tick's fresh reading, from readIMU()
  calSampleIdx++;

  if (calSampleIdx >= CAL_SAMPLES) {
    pitchOffset = calSum / (float)CAL_SAMPLES;
    pitch       = 0.0f;
    integral    = 0.0f;             // offset moved; a stale integral would kick
    calibrating = false;
    char buf[48];
    int n = snprintf(buf, sizeof(buf), "CAL:DONE,OFFSET:%.4f\n", pitchOffset);
    if (n > 0 && n < (int)sizeof(buf) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)buf, n);
  }
}

// ── MOTOR DRIVER ──────────────────────────────────────────────────────────────
void setMotors(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM  >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(ENA, abs(leftPWM));

  if (rightPWM >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else               { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(ENB, abs(rightPWM));
}

// ── COMMAND PARSER — single-loop balance + calibration only ──────────────────
void parseCommand(char *cmd) {
  char ack[160];

  // PING:<token> → PONG:<token>  (keeps latency_test.py working)
  if (cmd[0]=='P' && cmd[1]=='I' && cmd[2]=='N' && cmd[3]=='G') {
    Serial3.print("PONG");
    Serial3.println(cmd + 4);
    return;
  }

  // ── NEW TWO-CHAR COMMANDS (ax12_control ported) — must precede single-char ──
  // All TQ/TL/CM/CS/MS/MT/FT/FA/HM/SR/MD/RB checked here so they can never
  // collide with the single-letter TE/TG/TC/T/C/M/R/S/P cases below.

  // TQ0 / TQ1 — limp / re-grip
  // Re-gripping seeds goalPos and cur_* from the last-read presentPos so the
  // legs move smoothly from wherever the operator's hands left them.
  if (cmd[0]=='T' && cmd[1]=='Q') {
    bool want = (cmd[2] == '1');
    if (want && !g_torqueOn) {
      // Re-seed goalPos from a live present-position read of each servo.
      // (Was `legServos[i].goalPos = legServos[i].torqueLimit` — copied the
      // torque-limit register, ~1023, into the goal position, which slammed
      // every leg toward its travel limit on every re-grip.)
      for (int i = 0; i < 4; i++) {
        uint16_t pos;
        if (ax12ReadPresentPos(legServos[i].id, pos)) legServos[i].goalPos = pos;
        // else: servo didn't answer in time — keep the last known goalPos
        // rather than guess.
      }
      // --- FK re-seed of cur_* so the first move departs from the right foot ---
      uint16_t p6=818, p14=441, p0=818, p1=441;
      for (int i = 0; i < 4; i++) {
        if      (legServos[i].id == 6)  p6  = legServos[i].goalPos;
        else if (legServos[i].id == 14) p14 = legServos[i].goalPos;
        else if (legServos[i].id == 0)  p0  = legServos[i].goalPos;
        else if (legServos[i].id == 1)  p1  = legServos[i].goalPos;
      }
      Point2D f;
      if (solve_fk(ax12_to_angle(p6,  true,  false),
                   ax12_to_angle(p14, false, false), 0.0f, f)) {
        cur_x[0] = f.x; cur_y[0] = f.y;
      }
      float ikL2 = ax12_to_angle(p0, true,  true);
      float ikR2 = ax12_to_angle(p1, false, true);
      float aL2 = ikL2, aR2 = ikR2;
      if (LEG2_INVERTED_MOUNT) { aL2 = -ikR2; aR2 = -ikL2; }
      if (solve_fk(aL2, aR2, ik_dist, f)) {
        cur_x[1] = f.x - ik_dist; cur_y[1] = f.y;
      }
      moveActive = false;
    }
    g_torqueOn = want;
    markAllSettingsDirty();
    snprintf(ack, sizeof(ack), "ACK:TORQUE_%s", g_torqueOn ? "ON" : "LIMP");
    Serial3.println(ack);
    return;
  }

  // TL<n> — torque limit 0-1023
  else if (cmd[0]=='T' && cmd[1]=='L') {
    g_torqueLimit = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  // CM<n> — compliance margin 0-254
  else if (cmd[0]=='C' && cmd[1]=='M') {
    g_compMargin = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  // CS<n> — compliance slope 0-254
  else if (cmd[0]=='C' && cmd[1]=='S') {
    g_compSlope = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  // MS<n> — moving speed 0-1023 (0 = uncapped)
  else if (cmd[0]=='M' && cmd[1]=='S') {
    g_movingSpeed = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  // MT<n> — move time ms 100-3000
  else if (cmd[0]=='M' && cmd[1]=='T') {
    moveTimeMs = (uint16_t)constrain(atoi(cmd + 2), 100, 3000);
  }
  // FT<leg> <x> <y> — IK foot target for leg 1 or 2 (mm)
  else if (cmd[0]=='F' && cmd[1]=='T') {
    int leg = atoi(cmd + 2);
    char *s1 = strchr(cmd + 2, ' ');
    char *s2 = s1 ? strchr(s1 + 1, ' ') : NULL;
    if (s1 && s2 && (leg == 1 || leg == 2)) {
      ft_x[leg-1] = atof(s1 + 1);
      ft_y[leg-1] = atof(s2 + 1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      Serial3.println("FT:ERR BAD_FORMAT");
      return;
    }
  }
  // FA <x1> <y1> <x2> <y2> — BOTH legs atomically (avoids 2-FT stagger)
  else if (cmd[0]=='F' && cmd[1]=='A') {
    char *a = strchr(cmd + 2, ' ');
    char *b = a ? strchr(a + 1, ' ') : NULL;
    char *c = b ? strchr(b + 1, ' ') : NULL;
    char *d = c ? strchr(c + 1, ' ') : NULL;
    if (a && b && c && d) {
      ft_x[0]=atof(a+1); ft_y[0]=atof(b+1);
      ft_x[1]=atof(c+1); ft_y[1]=atof(d+1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      Serial3.println("FA:ERR BAD_FORMAT");
      return;
    }
  }
  // HM — home: standing pose in both modes
  else if (cmd[0]=='H' && cmd[1]=='M') {
    crouchOffset = 0.0f;
    ft_x[0]=ik_fx1; ft_y[0]=ik_fy1;
    ft_x[1]=ik_fx2; ft_y[1]=ik_fy2;
    startPoseMove();
  }
  // SR — servo reset (re-init bus registers)
  else if (cmd[0]=='S' && cmd[1]=='R') {
    initAX12Legs();
    snapPose();
    Serial3.println("ACK:SERVOS_RESET");
    return;
  }
  // MD0 / MD1 — pose mode: 0=CROUCH, 1=IK
  else if (cmd[0]=='M' && cmd[1]=='D') {
    uint8_t want = (cmd[2] == '1') ? MODE_IK : MODE_CROUCH;
    if (want != poseMode) {
      if (want == MODE_IK) {
        ft_x[0]=ik_fx1; ft_y[0]=ik_fy1+crouchOffset;
        ft_x[1]=ik_fx2; ft_y[1]=ik_fy2+crouchOffset;
      } else {
        crouchOffset = 0.5f * ((ft_y[0]-ik_fy1) + (ft_y[1]-ik_fy2));
      }
    }
    poseMode = want;
    startPoseMove();
  }
  // RB — request state broadcast (GUI resync after connect)
  else if (cmd[0]=='R' && cmd[1]=='B') {
    // Build and send an AX12 state line immediately
    char sl[192];
    int n = snprintf(sl, sizeof(sl),
      "AX12:MODE:%u,TQ:%u,TL:%u,CM:%u,CS:%u,MS:%u,MT:%u,CR:%.1f,"
      "FX1:%.2f,FY1:%.2f,FX2:%.2f,FY2:%.2f,IK1:%u,IK2:%u,MOVE:%d\n",
      (unsigned)poseMode,(unsigned)(g_torqueOn?1:0),
      (unsigned)g_torqueLimit,(unsigned)g_compMargin,
      (unsigned)g_compSlope,(unsigned)g_movingSpeed,
      (unsigned)moveTimeMs, crouchOffset,
      ft_x[0],ft_y[0],ft_x[1],ft_y[1],
      (unsigned)(ikValid[0]?1:0),(unsigned)(ikValid[1]?1:0),(int)moveActive);
    if (n > 0 && n < (int)sizeof(sl) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)sl, n);
    return;
  }

  // Auto-trim controls — multi-char, must be checked before the single-letter
  // 'T' (max safe tilt) case below or "TE1"/"TG.05" would parse as garbage tilt.
  else if (cmd[0]=='T' && cmd[1]=='E') {
    autoTrimEnabled = (cmd[2] == '1');
    if (!autoTrimEnabled) trim_bias = 0.0f;   // don't leave a half-converged bias live
    snprintf(ack, sizeof(ack), "ACK:AUTOTRIM_%s", autoTrimEnabled ? "ON" : "OFF");
    Serial3.println(ack);
    return;
  }
  else if (cmd[0]=='T' && cmd[1]=='C') {
    targetAngle += trim_bias;
    float committed = trim_bias;
    trim_bias = 0.0f;
    integral  = 0.0f;                         // stale balance integral would double-kick
    snprintf(ack, sizeof(ack), "TRIM:DONE COMMITTED:%.3f TARGET:%.3f", committed, targetAngle);
    Serial3.println(ack);
    return;
  }
  else if (cmd[0]=='T' && cmd[1]=='G') Ki_trim = atof(cmd + 2);
  else if (cmd[0]=='P' && cmd[1]=='S') {
    // Raw per-servo position, e.g. "PS6 750" — bypasses the crouch IK entirely,
    // for moving/testing exactly one joint. Checked before the single-letter
    // 'P' (Kp) case below, same ordering rule as every other multi-char prefix
    // in this parser (else "PS6 750" would parse as Kp = atof("S6 750") = 0).
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
    Serial3.println(ack);
    return;
  }
  // Balance / tuning commands
  else if (cmd[0] == 'P' && cmd[1] != '\0') Kp = atof(cmd + 1);
  else if (cmd[0] == 'I' && cmd[1] != '\0') Ki = atof(cmd + 1);
  else if (cmd[0] == 'D' && cmd[1] != '\0') Kd = atof(cmd + 1);
  else if (cmd[0] == 'A' && cmd[1] != '\0') alpha = atof(cmd + 1);
  else if (cmd[0] == 'T' && cmd[1] != '\0') maxSafeTilt = atof(cmd + 1);
  else if (cmd[0] == 'O' && cmd[1] != '\0') {
    // Manual pitch offset — re-seed pitch against the new reference.
    pitchOffset = atof(cmd + 1);
    pitch       = accelPitchRaw - pitchOffset;
    integral    = 0.0f;
  }
  else if (cmd[0] == 'S' && cmd[1] != '\0') targetAngle = atof(cmd + 1);
  else if (cmd[0] == 'S' && cmd[1] == '\0') {
    initAX12Legs();
    Serial3.println("ACK:SERVOS_RESET");
    return;
  }
  else if (cmd[0] == 'C' && cmd[1] == 'R') {
    // Crouch bar — checked before the bare 'C' (calibrate) case, or "CR40"
    // would trigger an IMU calibration instead of setting crouch depth.
    crouchOffset = atof(cmd + 2);
    // legs move (and hold, torque-independent of motorsEnabled) — same solver
    // solveGoalsFor() uses for FT/FA/HM moves, just applied immediately instead
    // of through the interpolated trajectory engine.
    float fx[2] = {ik_fx1, ik_fx2};
    float fy[2] = {ik_fy1 + crouchOffset, ik_fy2 + crouchOffset};
    solveGoalsFor(fx, fy);
  }
  else if (cmd[0] == 'C') { startCalibration(); return; }
  else if (cmd[0] == 'R') {
    integral  = 0.0f;
    trim_bias = 0.0f;
    Serial3.println("ACK:INT_RESET");
    return;
  }
  else if (cmd[0] == 'M') {
    motorsEnabled = !motorsEnabled;
    if (motorsEnabled) {
      safetyLatched = false;
      integral      = 0.0f;
      trim_bias     = 0.0f;
      vel_current   = 0.0f;
      encoderLeft = 0; encoderRight = 0;
      prevEncoderLeft = 0; prevEncoderRight = 0;
    }
    snprintf(ack, sizeof(ack), "Motors %s", motorsEnabled ? "ENABLED" : "DISABLED");
    Serial3.println(ack);
    return;
  }
  else return; // unknown command

  // Ack for tuning commands — parsed by _parse_fw_update() in the GUI.
  snprintf(ack, sizeof(ack),
    "Updated -> P:%.3f I:%.3f D:%.3f Offset:%.4f Target:%.3f Alpha:%.4f Tilt:%.2f TrimGain:%.4f Crouch:%.2f",
    Kp, Ki, Kd, pitchOffset, targetAngle, alpha, maxSafeTilt, Ki_trim, crouchOffset);
  uint8_t len = (uint8_t)strlen(ack);
  ack[len] = '\n'; len++;
  if (Serial3.availableForWrite() >= len)
    Serial3.write((uint8_t*)ack, len);
}

// ── NON-BLOCKING RX — 300 µs budget ──────────────────────────────────────────
// A byte at 115200 baud is 87 µs, so the old 40 µs budget drained at most ONE
// byte per call. Combined with the old 50 Hz call rate that made an 8-byte
// command ("CR40.0\n") take 8 x 20 ms = 160 ms to even reach the parser, and
// let the 64-byte UART FIFO overflow (silently corrupting commands) whenever
// the radio delivered a burst. 300 µs is 3% of the 10 ms loop and drains ~3
// bytes/tick; handleTelemetryRX() is now also called EVERY tick (see loop()).
const unsigned long RX_BUDGET_US = 300;

static char  rxBuf[48];
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

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
  delay(2000);                 // let AX-12 servos stabilise before UART traffic

  Serial1.begin(115200);       // tick profiler out (PA9 = TX)
  Serial3.begin(115200);       // 3DR radio
  Serial2.begin(1000000);      // AX-12 bus

  initAX12Legs();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  setupMPU();
  // Snap legs to standing pose and broadcast it via SYNC_WRITE before the
  // balance loop starts. This means cur_* is valid from the very first tick.
  snapPose();
  ax12SyncWriteGoals();

  lastTime     = micros();
  lastPollTime = millis();

  Serial3.println("BOOT:OK");

  profResetWindow();
  Serial1.println();
  Serial1.println("# profiler: P period B body F free X overhead");
}

// ── MAIN LOOP (100 Hz) ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastTime < 10000) return;   // enforce 100 Hz
  float dt = (now - lastTime) * 1.0e-6f;
  uint32_t prof_period = (uint32_t)(now - lastTime);
  lastTime = now;

  uint32_t prof_t0 = micros();      // profiler: start of the loop body
  uint32_t prof_m  = prof_t0;       // profiler: start of the current stage

  // ── IMU ────────────────────────────────────────────────────────────────
  readIMU(dt);
  prof_stage[0] = (uint16_t)(micros() - prof_m); prof_m = micros();   // I
  calibrationTask();   // accumulates this tick's sample when a cal is running
  prof_stage[1] = (uint16_t)(micros() - prof_m); prof_m = micros();   // K

  // ── SAFETY CUTOFF ───────────────────────────────────────────────────────
  if (fabsf(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral      = 0.0f;
    setMotors(0, 0);
    Serial3.println("SAFETY:CUTOFF");
  }
  prof_stage[2] = (uint16_t)(micros() - prof_m); prof_m = micros();   // Y

  // ── ENCODER → VELOCITY (telemetry display only) ─────────────────────────
  long encL = encoderLeft;
  long encR = encoderRight;
  float deltaL =  (float)(encL - prevEncoderLeft);
  float deltaR = -(float)(encR - prevEncoderRight);   // mirror-mount normalise
  float vel_raw = ((deltaL + deltaR) * 0.5f) / dt;
  vel_current = vel_alpha * vel_current + (1.0f - vel_alpha) * vel_raw;
  prevEncoderLeft  = encL;
  prevEncoderRight = encR;
  prof_stage[3] = (uint16_t)(micros() - prof_m); prof_m = micros();   // E

  // Leg torque is intentionally independent of motorsEnabled (the drive-wheel
  // arm state): pollLegServosTask() below keeps torque enabled and re-asserts
  // legServos[].goalPos every 20 ms/servo regardless, so the CR<f> crouch bar
  // can stand/crouch the legs on the bench with the wheel motors disarmed.

  // ══════════════════════════════════════════════════════════════════════════
  // THE SINGLE BALANCE PID — the only control loop
  // ══════════════════════════════════════════════════════════════════════════
  // `calibrating` holds the motors off for the ~1 s sampling window. The old
  // blocking calibrateIMU() froze the whole loop, so the motors could not act
  // on a half-settled pitch; now that the loop keeps running, say so explicitly.
  float output = 0.0f;
  if (!motorsEnabled || calibrating) {
    integral  = 0.0f;
    trim_bias = 0.0f;
    setMotors(0, 0);
  } else {
    if (autoTrimEnabled) {
      // Same sign convention as RC_mcu_IK_wireless's velocity-integral tilt
      // bias: there's no drive command in this variant, so the implied
      // target velocity is always 0 — any steady vel_current is drift from
      // a wrong targetAngle, integrated out here instead of by hand.
      trim_bias += Ki_trim * (0.0f - vel_current) * dt;
      trim_bias  = constrain(trim_bias, -MAX_TRIM_BIAS, MAX_TRIM_BIAS);
    }

    float error = (targetAngle + trim_bias) - pitch;

    integral += error * dt;
    if (Ki > 1e-6f) {                       // anti-windup: clamp integrator STATE
      float intLimit = MAX_INTEGRAL_PWM / Ki;
      integral = constrain(integral, -intLimit, intLimit);
    } else {
      integral = 0.0f;                       // Ki off: never bank a latent kick
    }

    float derivative = -gyroRate;            // derivative on measurement
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int pwm = (int)constrain(-output, -255.0f, 255.0f);
    setMotors(pwm, pwm);                      // no steering — pure balance
  }

  prof_stage[4] = (uint16_t)(micros() - prof_m); prof_m = micros();   // C

  // ── RX EVERY TICK, SERVO POLL AT 50 Hz ──────────────────────────────────
  // Uplink commands are latency-critical and the downlink was starving them,
  // so RX now runs on every 10 ms tick instead of every other one. The servo
  // health poll keeps its old 50 Hz slot — it is a slow, purely cosmetic read
  // and pollLegServosTask() already rate-limits itself to POLL_INTERVAL_MS.
  handleTelemetryRX();
  prof_stage[5] = (uint16_t)(micros() - prof_m); prof_m = micros();   // R

  // ── SETTINGS PUSH (gated on POLL_IDLE to avoid collision with reply bytes) ──
  if (pollState == POLL_IDLE) applySettingsTask();

  // ── 3-MODE BUS ARBITRATION — exactly one Serial2 transaction per tick ────────
  // Mode A: move active → SYNC_WRITE at full 100 Hz, health suspended.
  // Mode B: idle, settings dirty → applySettingsTask() (already ran above).
  // Mode C: idle, no settings → hold pose at 10 Hz + health poll at 50 Hz.
  if (moveActive) {
    // Abandon any in-flight health read so it does not collide with move writes.
    if (pollState == POLL_WAITING) {
      ax12DrainRx();
      pollState    = POLL_IDLE;
      lastPollTime = millis();
    }
    updateMoveTask();   // solveGoalsFor() + ax12SyncWriteGoals() inside
  } else {
    // Hold pose at 10 Hz — re-asserts goals to fight servo drift, but at a
    // cadence slow enough that the health poll can run in the gaps.
    static unsigned long lastHoldUs = 0;
    if (g_torqueOn && pollState == POLL_IDLE && now - lastHoldUs >= 100000) {
      lastHoldUs = now;
      ax12SyncWriteGoals();
    }
    static bool isReadCycle = false;
    isReadCycle = !isReadCycle;
    if (isReadCycle) pollLegServosTask();
  }
  prof_stage[6] = (uint16_t)(micros() - prof_m); prof_m = micros();   // V

  // ── TELEMETRY @ 10 Hz ────────────────────────────────────────────────────
  // Halved from 20 Hz. The 3DR/SiK link is half-duplex with a TDM air protocol:
  // each end only gets ~half the airtime, so the old ~2.7 kB/s downlink left no
  // window for the ground unit to transmit, and commands sat in the radio's
  // buffer for seconds. (latency_test.py measured 97% downlink loss from a
  // single PING every 2 s — proof the air link had zero headroom.)
  if (now - lastPrintTime >= 100000) {
    lastPrintTime = now;

    // ── Balance telemetry (10 Hz) ────────────────────────────────────────────
    // Extended with leg subsystem state fields so the GUI can display both
    // the balance loop AND the AX-12 trajectory/mode in one pass.
    char line[256];
    int n = snprintf(line, sizeof(line),
      "PITCH:%.2f,PID_OUT:%.2f,INT:%.4f,EL:%ld,ER:%ld,VEL:%.1f,"
      "MOT:%d,TILT:%.1f,TRIM:%.3f,ATE:%d,LATCH:%d,"
      "TORQ:%d,CR:%.1f,FX1:%.2f,FY1:%.2f,FX2:%.2f,FY2:%.2f,IK1:%d,IK2:%d,MOVE:%d\n",
      pitch, output, integral, encL, encR, vel_current,
      (int)motorsEnabled, maxSafeTilt, trim_bias,
      (int)autoTrimEnabled, (int)safetyLatched,
      (int)g_torqueOn, crouchOffset,
      cur_x[0], cur_y[0], cur_x[1], cur_y[1],
      (int)ikValid[0], (int)ikValid[1], (int)moveActive);
    if (n > 0 && n < (int)sizeof(line) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)line, n);

    // ── AX-12 state line (every 5 ticks = 500 ms) — compliance, speed, mode ─
    // Emitted on alternating frames so it never competes with the PITCH line.
    static uint8_t ax12StateDiv = 0;
    if (++ax12StateDiv >= 5) {
      ax12StateDiv = 0;
      int m = snprintf(line, sizeof(line),
        "AX12:MODE:%u,TQ:%u,TL:%u,CM:%u,CS:%u,MS:%u,MT:%u\n",
        (unsigned)poseMode, (unsigned)(g_torqueOn?1:0),
        (unsigned)g_torqueLimit, (unsigned)g_compMargin,
        (unsigned)g_compSlope,  (unsigned)g_movingSpeed,
        (unsigned)moveTimeMs);
      if (m > 0 && m < (int)sizeof(line) && Serial3.availableForWrite() >= m)
        Serial3.write((uint8_t*)line, m);
    }

    // ── Servo health at 1 Hz (round-robin one servo per second) ─────────────
    static uint8_t healthIdx   = 0;
    static unsigned long lastHealth = 0;
    if (now - lastHealth >= 1000000) {
      lastHealth = now;
      ServoState &h = legServos[healthIdx];
      int k = snprintf(line, sizeof(line), "SRV:%u,%u,%.1f\n",
                       (unsigned)h.id, (unsigned)h.temp, h.loadPct);
      if (k > 0 && k < (int)sizeof(line) && Serial3.availableForWrite() >= k)
        Serial3.write((uint8_t*)line, k);
      healthIdx = (healthIdx + 1) % 4;
    }
  }
  prof_stage[7] = (uint16_t)(micros() - prof_m);                       // T

  // ══════════════════════════════════════════════════════════════════════════
  // PROFILER EMIT — everything below is instrumentation, no control logic
  // ══════════════════════════════════════════════════════════════════════════
  // Body is sampled HERE, before anything below runs, so it excludes the
  // profiler's own cost. That tax is reported separately as X (previous tick's,
  // since this tick's is not knowable until after the write). Subtract X to get
  // what the loop costs with profiling compiled out.
  uint32_t prof_bodyU = micros() - prof_t0;
  uint16_t prof_body  = (prof_bodyU > 65535) ? 65535 : (uint16_t)prof_bodyU;
  profAccumulate(prof_body);

  uint32_t prof_tp = micros();
  {
    char pb[128];
    if (prof_report >= 0) {
      // A 1 Hz report is in flight: one line per tick, raw row suppressed.
      // Those ticks are still counted above — only their printing is skipped.
      if (profReportLine(pb, sizeof(pb))) {
        int n = (int)strlen(pb);
        pb[n++] = '\n';
        if (Serial1.availableForWrite() >= n) { Serial1.write((uint8_t*)pb, n); prof_report++; }
      } else {
        prof_report = -1;
      }
    } else {
      int n = snprintf(pb, sizeof(pb), "P%lu B%u F%ld",
                       (unsigned long)prof_period, (unsigned)prof_body,
                       (long)(10000 - (int32_t)prof_body));
      n += profStages(pb + n, (int)sizeof(pb) - n, prof_stage);
      n += snprintf(pb + n, sizeof(pb) - n, " X%lu%s\n",
                    (unsigned long)prof_printUs,
                    (prof_body > 10000) ? " !OVR" : "");
      if (n > 0 && n < (int)sizeof(pb) && Serial1.availableForWrite() >= n)
        Serial1.write((uint8_t*)pb, n);
      else if (prof_drop < 0xFFFF) prof_drop++;
    }

    // Roll the window once a second. Snapshot first so the report can be
    // emitted over the following ticks while a fresh window accumulates.
    static unsigned long profLastReport = 0;
    if (prof_report < 0 && now - profLastReport >= 1000000) {
      profLastReport = now;
      profSnapshotReport();
    }
  }
  prof_printUs = micros() - prof_tp;
}
