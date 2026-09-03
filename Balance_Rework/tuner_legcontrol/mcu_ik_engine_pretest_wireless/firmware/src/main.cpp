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
const float MAX_INTEGRAL_PWM = 120.0f;      // anti-windup: cap Ki term's PWM

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
const float MAX_TRIM_BIAS = 5.0f;       // anti-windup clamp (deg), matches sibling variant

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

ServoState legServos[4] = {
  {6,  818, 511, 4, 32, 0, 0.0f},   // Leg1 Left  (818 = straight-down left)
  {0,  818, 511, 4, 32, 0, 0.0f},   // Leg2 Left
  {14, 441, 511, 4, 32, 0, 0.0f},   // Leg1 Right (441 = straight-down right)
  {1,  441, 511, 4, 32, 0, 0.0f},   // Leg2 Right
};

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

void calibrateIMU() {
  Serial3.println("CAL:START");
  long double sum = 0;
  for (int i = 0; i < 100; i++) {
    readIMU(0.01f);
    sum += accelPitchRaw;
    delay(10);
  }
  pitchOffset = (float)(sum / 100.0);
  pitch = 0.0f;
  char buf[48];
  snprintf(buf, sizeof(buf), "CAL:DONE,OFFSET:%.4f", pitchOffset);
  Serial3.println(buf);
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
  else if (cmd[0] == 'C') { calibrateIMU(); return; }
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
    "Updated -> P:%.3f I:%.3f D:%.3f Offset:%.4f Target:%.3f Alpha:%.4f Tilt:%.2f TrimGain:%.4f",
    Kp, Ki, Kd, pitchOffset, targetAngle, alpha, maxSafeTilt, Ki_trim);
  uint8_t len = (uint8_t)strlen(ack);
  ack[len] = '\n'; len++;
  if (Serial3.availableForWrite() >= len)
    Serial3.write((uint8_t*)ack, len);
}

// ── NON-BLOCKING RX — 40 µs budget ────────────────────────────────────────────
static char  rxBuf[48];
static uint8_t rxLen = 0;

void handleTelemetryRX() {
  unsigned long rxStart = micros();
  while (Serial3.available() && (micros() - rxStart) < 40) {
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

  // ── AX-12 torque follows arm state ──────────────────────────────────────
  static bool prevMotorsEnabled = false;
  if (motorsEnabled != prevMotorsEnabled) {
    prevMotorsEnabled = motorsEnabled;
    for (int i = 0; i < 4; i++) ax12WriteByte(legServos[i].id, 24, motorsEnabled ? 1 : 0);
  }

  // ══════════════════════════════════════════════════════════════════════════
  // THE SINGLE BALANCE PID — the only control loop
  // ══════════════════════════════════════════════════════════════════════════
  float output = 0.0f;
  if (!motorsEnabled) {
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

  // ── 50 Hz READ/WRITE TOGGLE ─────────────────────────────────────────────
  // READ tick: poll one servo's health.  WRITE tick: process GUI commands.
  static bool isReadCycle = false;
  isReadCycle = !isReadCycle;
  if (isReadCycle) pollLegServosTask();
  else             handleTelemetryRX();

  // ── TELEMETRY @ 20 Hz ────────────────────────────────────────────────────
  if (now - lastPrintTime >= 50000) {
    lastPrintTime = now;

    // Balance line (parsed by _parse_telemetry in the GUI)
    Serial3.print("PITCH:");   Serial3.print(pitch, 2);
    Serial3.print(",PID_OUT:");Serial3.print(output, 2);
    Serial3.print(",INT:");    Serial3.print(integral, 4);
    Serial3.print(",EL:");     Serial3.print(encL);
    Serial3.print(",ER:");     Serial3.print(encR);
    Serial3.print(",VEL:");    Serial3.print(vel_current, 1);
    Serial3.print(",MOT:");    Serial3.print((int)motorsEnabled);
    Serial3.print(",TILT:");   Serial3.print(maxSafeTilt, 1);
    Serial3.print(",TRIM:");   Serial3.print(trim_bias, 3);
    Serial3.print(",ATE:");    Serial3.print((int)autoTrimEnabled);
    Serial3.print(",LATCH:");  Serial3.println((int)safetyLatched);

    // One servo-health line per telemetry frame (round-robin), SRV:id,temp,load
    static uint8_t healthIdx = 0;
    ServoState &h = legServos[healthIdx];
    Serial3.print("SRV:");  Serial3.print(h.id);
    Serial3.print(",");     Serial3.print(h.temp);
    Serial3.print(",");     Serial3.println(h.loadPct, 1);
    healthIdx = (healthIdx + 1) % 4;
  }
}
