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
extern HardwareSerial Serial2;   // AX-12 bus
extern HardwareSerial Serial3;   // 3DR radio

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

// Recomputes goalPos for all 4 legs at the current crouchOffset and caches it
// into legServos[]; pollLegServosTask() (20ms/servo) re-asserts it to hardware
// on its normal cadence — no need to touch the 100 Hz hot path for this.
void updateLegPose() {
  float footY1 = ik_fy1 + crouchOffset;
  float footY2 = ik_fy2 + crouchOffset;
  IK_Result sol1 = solve_ik(ik_fx1, footY1, 0.0f);
  IK_Result sol2 = solve_ik(ik_fx2 + ik_dist, footY2, ik_dist);
  if (!sol1.valid || !sol2.valid) return;   // out of reach — leave last good pose

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

    // Re-assert held pose (RAM only, no EEPROM wear) so the legs stay put.
    ax12WriteByte(s.id, 24, 1);
    ax12WriteWord(s.id, 34, s.torqueLimit);
    ax12WriteWord(s.id, 30, s.goalPos);

    while (Serial2.available()) Serial2.read();   // flush write echoes

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
      while (Serial2.available()) Serial2.read();
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

  // Auto-trim controls — multi-char, must be checked before the single-letter
  // 'T' (max safe tilt) case below or "TE1"/"TG.05" would parse as garbage tilt.
  if (cmd[0]=='T' && cmd[1]=='E') {
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
    updateLegPose();   // legs move (and hold, torque-independent of motorsEnabled)
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
  lastTime     = micros();
  lastPollTime = millis();

  Serial3.println("BOOT:OK");
}

// ── MAIN LOOP (100 Hz) ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastTime < 10000) return;   // enforce 100 Hz
  float dt = (now - lastTime) * 1.0e-6f;
  lastTime = now;

  // ── IMU ────────────────────────────────────────────────────────────────
  readIMU(dt);
  calibrationTask();   // accumulates this tick's sample when a cal is running

  // ── SAFETY CUTOFF ───────────────────────────────────────────────────────
  if (fabsf(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral      = 0.0f;
    setMotors(0, 0);
    Serial3.println("SAFETY:CUTOFF");
  }

  // ── ENCODER → VELOCITY (telemetry display only) ─────────────────────────
  long encL = encoderLeft;
  long encR = encoderRight;
  float deltaL =  (float)(encL - prevEncoderLeft);
  float deltaR = -(float)(encR - prevEncoderRight);   // mirror-mount normalise
  float vel_raw = ((deltaL + deltaR) * 0.5f) / dt;
  vel_current = vel_alpha * vel_current + (1.0f - vel_alpha) * vel_raw;
  prevEncoderLeft  = encL;
  prevEncoderRight = encR;

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

  // ── RX EVERY TICK, SERVO POLL AT 50 Hz ──────────────────────────────────
  // Uplink commands are latency-critical and the downlink was starving them,
  // so RX now runs on every 10 ms tick instead of every other one. The servo
  // health poll keeps its old 50 Hz slot — it is a slow, purely cosmetic read
  // and pollLegServosTask() already rate-limits itself to POLL_INTERVAL_MS.
  handleTelemetryRX();

  static bool isReadCycle = false;
  isReadCycle = !isReadCycle;
  if (isReadCycle) pollLegServosTask();

  // ── TELEMETRY @ 10 Hz ────────────────────────────────────────────────────
  // Halved from 20 Hz. The 3DR/SiK link is half-duplex with a TDM air protocol:
  // each end only gets ~half the airtime, so the old ~2.7 kB/s downlink left no
  // window for the ground unit to transmit, and commands sat in the radio's
  // buffer for seconds. (latency_test.py measured 97% downlink loss from a
  // single PING every 2 s — proof the air link had zero headroom.)
  if (now - lastPrintTime >= 100000) {
    lastPrintTime = now;

    // Build the balance line into one buffer and hand it to the UART in a
    // single guarded write. The old per-field Serial3.print() calls BLOCK once
    // the radio backs up, stretching the 100 Hz loop's dt and delaying RX
    // further — the ack path in parseCommand() already guards this way.
    char line[192];
    int n = snprintf(line, sizeof(line),
      "PITCH:%.2f,PID_OUT:%.2f,INT:%.4f,EL:%ld,ER:%ld,VEL:%.1f,"
      "MOT:%d,TILT:%.1f,TRIM:%.3f,ATE:%d,LATCH:%d\n",
      pitch, output, integral, encL, encR, vel_current,
      (int)motorsEnabled, maxSafeTilt, trim_bias,
      (int)autoTrimEnabled, (int)safetyLatched);

    if (n > 0 && n < (int)sizeof(line) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)line, n);

    // Servo health at 1 Hz (round-robin one servo per second) instead of one
    // line per telemetry frame. Temperature and load are slow-moving; sending
    // them 20x/s was pure air-time waste competing with the uplink.
    static uint8_t healthIdx   = 0;
    static unsigned long lastHealth = 0;
    if (now - lastHealth >= 1000000) {
      lastHealth = now;
      ServoState &h = legServos[healthIdx];
      int m = snprintf(line, sizeof(line), "SRV:%u,%u,%.1f\n",
                       (unsigned)h.id, (unsigned)h.temp, h.loadPct);
      if (m > 0 && m < (int)sizeof(line) && Serial3.availableForWrite() >= m)
        Serial3.write((uint8_t*)line, m);
      healthIdx = (healthIdx + 1) % 4;
    }
  }
}
