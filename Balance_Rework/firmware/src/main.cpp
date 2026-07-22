#include <Arduino.h>
#include <Wire.h>
 
// ============================================================================
//  BALANCE REWORK FIRMWARE  —  STM32 Bluepill (F103C8) self-balancing biped
// ----------------------------------------------------------------------------
//  Changes vs. the original PlatformIO_Firmware/src/main.cpp:
//    1. SAFETY: automatic tilt cutoff (MAX_SAFE_TILT) that latches motors OFF.
//    2. IMU:    complementary-filter fusion of accelerometer + gyro Y rate.
//    3. CTRL:   encoder-derived wheel-velocity damping (Kd_vel).
//    4. SERIAL: new live-tune commands (V/A/T) + extended telemetry line.
//    5. AX-12:  staggered torque/position heartbeat (healLegServos) so a leg
//               servo that silently drops Torque Enable (overload trip, brownout,
//               EEPROM Max Torque mismatch, etc.) gets re-asserted automatically
//               instead of staying limp until the next manual "S" reset.
//  Pin layout is IDENTICAL to the original (see Hardware_Connections.md).
// ============================================================================
 
// --- ENCODER SETTINGS ---
// Left
#define ENC_L_A PA6
#define ENC_L_B PA7
// Right
#define ENC_R_A PB0
#define ENC_R_B PB1
 
volatile long encoderLeft = 0;
volatile long encoderRight = 0;
 
void countLeft() { if (digitalRead(ENC_L_B)) encoderLeft++; else encoderLeft--; }
void countRight() { if (digitalRead(ENC_R_B)) encoderRight++; else encoderRight--; }
 
// --- MOTOR PINS (from your main.cpp) ---
HardwareSerial Serial2(USART2);
 
#define ENA PA1
#define IN1 PB14
#define IN2 PB15
 
#define ENB PA0
#define IN3 PB12
#define IN4 PB13
 
// --- MPU6050 & PID ---
const int MPU_ADDR = 0x68;
float pitch = 0.0, pitchOffset = 0.0;
float Kp = 11.21, Ki = 0.0, Kd = 0.715;   // starting point from the old "tuned" run
float targetAngle = 0.0;
float integral = 0.0, prevError = 0.0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
 
// --- NEW tunable globals (all live-tunable over serial, see handleSerialTuning) ---
// Complementary filter blend factor: pitch = alpha*(gyro-integrated) + (1-alpha)*accel.
// Higher alpha => trust the gyro more (smoother, but drifts); lower => trust accel more.
float alpha = 0.96;
 
// Encoder velocity damping gain. Subtracts Kd_vel * wheelVelocity from the PID output
// to fight the "run away then lurch back" wobble. Default 0.0 => feature off until swept.
float Kd_vel = 0.0;
 
// Safety tilt limit (degrees). Default seeded from the #define below; live-tunable via "T".
#define MAX_SAFE_TILT_DEFAULT 35.0f
float maxSafeTilt = MAX_SAFE_TILT_DEFAULT;
 
// Sign of the gyro-Y contribution in the complementary filter. Depends on how the MPU
// is physically mounted. If, when you slowly tilt the robot by hand on the harness, the
// fused PITCH lags or runs AWAY from the raw accelerometer tilt instead of tracking it,
// flip this to -1.0f and reflash. (See Balance_Rework/README.md, "First-run checklist".)
#define GYRO_PITCH_SIGN 1.0f
 
// Raw accelerometer-only pitch (before offset subtraction), captured each readIMU().
// Used by calibrateIMU() so calibration never depends on the fused estimate.
float accelPitchRaw = 0.0;
 
bool motorsEnabled = false; // safe toggle
bool safetyLatched = false; // set true when a safety cutoff has latched motors off
 
// --- AX-12 Servo Writer Helpers ---
void ax12WriteByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t checksum = ~(id + 4 + 3 + addr + val) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x03, addr, val, checksum};
  Serial2.write(packet, 8);
  Serial2.flush();
}
 
void ax12WriteWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t checksum = ~(id + 5 + 3 + addr + lo + hi) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x05, 0x03, addr, lo, hi, checksum};
  Serial2.write(packet, 9);
  Serial2.flush();
}
 
void initAX12Legs() {
  Serial1.println("Locking AX-12 Legs to home position...");
  uint8_t left_servos[] = {6, 0};
  uint8_t right_servos[] = {14, 1};
  // Calibrated lowered/squat pose (foot x,y = 0, -130mm).
  // Upright standing pose was: left_positions[] = {818, 818}, right_positions[] = {441, 441}
  // left_servos = {6, 0}; right_servos = {14, 1}.
  uint16_t left_positions[] = {717, 717};   // id6, id0
  uint16_t right_positions[] = {541, 541};  // id14, id1
 
  for(int i = 0; i < 2; i++) {
    for (uint8_t id : {left_servos[i], right_servos[i]}) {
      ax12WriteByte(id, 24, 1);    // Torque Enable
      ax12WriteWord(id, 34, 1023); // Torque Limit (maximum holding torque)
      ax12WriteByte(id, 26, 1);    // CW Compliance Margin (tight inner deadzone)
      ax12WriteByte(id, 27, 1);    // CCW Compliance Margin
      ax12WriteByte(id, 28, 4);    // CW Compliance Slope (stiffer holding, was 16)
      ax12WriteByte(id, 29, 4);    // CCW Compliance Slope (stiffer holding, was 16)
    }
    ax12WriteWord(left_servos[i], 30, left_positions[i]);
    ax12WriteWord(right_servos[i], 30, right_positions[i]);
  }
}
 
// ── AX-12 position-hold heartbeat ───────────────────────────────────────────
// Periodically re-asserts Torque Enable, Torque Limit, and Goal Position for one
// leg servo at a time. Staggered (one servo per HEAL_INTERVAL_MS) rather than all
// four at once, so we're not hammering the power rail with 4 simultaneous torque
// re-enables every cycle. This does NOT read anything back from the servos (your
// half-duplex wiring hasn't been confirmed to support clean reads yet) — it just
// blindly re-asserts state, which is enough to recover from a silent Torque
// Enable drop (overload trip / brownout / EEPROM Max Torque mismatch) without
// needing to know why it happened.
//
// NOTE: goalPos values here MUST be kept in sync with initAX12Legs()'s
// left_positions/right_positions if you change the standing/squat pose there.
struct LegServo {
  uint8_t id;
  uint16_t goalPos;
};
 
LegServo legServos[4] = {
  {6,  717},  // left
  {0,  717},  // left
  {14, 541},  // right
  {1,  541},  // right
};
 
unsigned long lastHealTime = 0;
uint8_t healIndex = 0;
const unsigned long HEAL_INTERVAL_MS = 1500; // one servo every 1500ms -> full sweep every 6s
 
void healLegServos() {
  unsigned long now = millis();
  if (now - lastHealTime < HEAL_INTERVAL_MS) return;
  lastHealTime = now;
 
  LegServo &s = legServos[healIndex];
  ax12WriteByte(s.id, 24, 1);         // re-assert Torque Enable
  ax12WriteWord(s.id, 34, 1023);      // re-assert Torque Limit
  ax12WriteWord(s.id, 30, s.goalPos); // re-assert Goal Position
 
  healIndex = (healIndex + 1) % 4;
}
 
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
 
// Reads the MPU6050 and updates the fused `pitch` via a complementary filter.
// dt is the loop period in seconds (needed for gyro integration).
void readIMU(float dt) {
  // Burst-read 12 bytes starting at ACCEL_XOUT_H (0x3B):
  //   0x3B/3C ax, 0x3D/3E ay, 0x3F/40 az, 0x41/42 temp,
  //   0x43/44 gyroX, 0x45/46 gyroY  <- pitch rotation is about Y.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)12, (uint8_t)true);
 
  int16_t ax   = Wire.read() << 8 | Wire.read();
  int16_t ay   = Wire.read() << 8 | Wire.read();
  int16_t az   = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();  (void)temp; // discarded
  int16_t gx   = Wire.read() << 8 | Wire.read();  (void)gx;   // discarded
  int16_t gy   = Wire.read() << 8 | Wire.read();               // GYRO_YOUT
 
  // Accelerometer-only pitch (same convention as the original firmware).
  accelPitchRaw = atan2((float)-ax, sqrt((float)ay*(float)ay + (float)az*(float)az)) * 180.0 / PI;
  float accelPitch = accelPitchRaw - pitchOffset;
 
  // Gyro Y rate in deg/s (MPU6050 default full-scale ±250 dps => 131 LSB/(deg/s)).
  float gyroRate = GYRO_PITCH_SIGN * (float)gy / 131.0;
 
  // Complementary filter: integrate the gyro for the fast/smooth term, correct the
  // slow drift with the accelerometer.
  pitch = alpha * (pitch + gyroRate * dt) + (1.0 - alpha) * accelPitch;
}
 
void calibrateIMU() {
  Serial1.println("Calibrating IMU... Keep robot still and upright.");
  long double sum = 0;
  for(int i = 0; i < 100; i++) {
    readIMU(0.01);                // dt nominal; robot must be still during calibration
    sum += accelPitchRaw;         // use raw accel only, never the fused estimate
    delay(10);
  }
  pitchOffset = (float)(sum / 100.0);
  pitch = 0.0;                     // reset fused estimate to the new zero
  Serial1.print("Calibration complete. Offset: ");
  Serial1.println(pitchOffset);
}
 
void setMotors(int leftPWM, int rightPWM) {
  // Constrain limits
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);
 
  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(leftPWM));
 
  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(rightPWM));
}
 
void handleSerialTuning() {
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    input.toUpperCase(); // Make it case-insensitive so 'm' or 'M' both work
 
    if (input.startsWith("P")) Kp = input.substring(1).toFloat();
    else if (input.startsWith("I")) Ki = input.substring(1).toFloat();
    else if (input.startsWith("D")) Kd = input.substring(1).toFloat();
    else if (input == "S") {
      initAX12Legs();
      Serial1.println("Servos reset to home position!");
    }
    else if (input.startsWith("S") && input.length() > 1) targetAngle = input.substring(1).toFloat();
    else if (input.startsWith("V")) Kd_vel = input.substring(1).toFloat();          // NEW: velocity damping gain
    else if (input.startsWith("A")) alpha = input.substring(1).toFloat();           // NEW: complementary filter blend
    else if (input.startsWith("T")) maxSafeTilt = input.substring(1).toFloat();     // NEW: safety tilt limit
    else if (input.startsWith("C")) calibrateIMU();
    else if (input.startsWith("M")) {
      motorsEnabled = !motorsEnabled;
      if (motorsEnabled) {
        // Explicit re-enable clears the safety latch and resets controller state so a
        // prior fall doesn't kick the motors on re-enable.
        safetyLatched = false;
        integral = 0.0;
      }
      Serial1.print("Motors "); Serial1.println(motorsEnabled ? "ENABLED" : "DISABLED");
    }
 
    // Extended status print — original P/I/D/Offset/Target fields preserved,
    // new Vel/Alpha/Tilt fields appended.
    Serial1.print("Updated -> P:"); Serial1.print(Kp);
    Serial1.print(" I:"); Serial1.print(Ki);
    Serial1.print(" D:"); Serial1.print(Kd);
    Serial1.print(" Offset:"); Serial1.print(pitchOffset);
    Serial1.print(" Target:"); Serial1.print(targetAngle);
    Serial1.print(" Vel:"); Serial1.print(Kd_vel, 4);
    Serial1.print(" Alpha:"); Serial1.print(alpha, 4);
    Serial1.print(" Tilt:"); Serial1.println(maxSafeTilt);
  }
}
 
// Previous encoder counts, used to derive wheel velocity each loop.
long prevEncoderLeft = 0;
long prevEncoderRight = 0;
 
void setup() {
  Serial1.begin(115200);
 
  // 2-second delay to allow AX-12+ servos to power up and stabilize before initializing Serial2.
  // This leaves the UART pins floating/high-impedance during the servo boot sequence to prevent noise.
  delay(2000);
 
  Serial2.begin(1000000); // AX-12s
  initAX12Legs(); // Automatically lock the legs at startup
 
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
 
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);
 
  setupMPU();
  lastTime = micros(); // Fixed frequency timing
  lastHealTime = millis(); // start the leg heartbeat timer fresh so it doesn't fire immediately
}
 
void loop() {
  unsigned long now = micros();
  // 100Hz fixed loop (10000 microseconds)
  if (now - lastTime < 10000) return;
 
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
 
  readIMU(dt);
 
  // ── SAFETY NET ─────────────────────────────────────────────────────────
  // If we exceed the tilt limit while motors are live, latch them OFF immediately.
  // This requires an explicit "M" re-enable to recover (no auto-recovery), so a fall
  // can't keep driving the wheels into the ground or a person's hand.
  if (fabs(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral = 0.0;
    setMotors(0, 0);
    Serial1.println("SAFETY CUTOFF TRIGGERED");
  }
 
  // ── Encoder-derived wheel velocity (ticks/sec & linear velocity) ────────
  // 32-bit aligned reads are atomic on Cortex-M3, so no need to disable interrupts.
  long encL = encoderLeft;
  long encR = encoderRight;
  float wheelVelocity = ((float)((encL - prevEncoderLeft) + (encR - prevEncoderRight)) * 0.5) / dt;
  prevEncoderLeft = encL;
  prevEncoderRight = encR;
 
  // 330 ticks per rotation, 67mm wheel diameter -> circumference = PI * 67mm
  float linearVelMmS = wheelVelocity * (3.14159265f * 67.0f / 330.0f); // mm/s
  float linearVelMS  = linearVelMmS / 1000.0f; // m/s
 
  // ── Balancing PID ───────────────────────────────────────────────────────
  float error = targetAngle - pitch;
 
  // Prevent Integral Windup: only build integral while motors are actually running.
  if (!motorsEnabled) {
    integral = 0.0;
    prevError = error; // Prevents massive derivative spike when turning motors back on
  } else {
    integral += error * dt;
  }
 
  float derivative = (error - prevError) / dt;
  prevError = error;
 
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
 
  // Velocity damping: subtract Kd_vel * wheelVelocity before it reaches the motors.
  output -= Kd_vel * wheelVelocity;
 
  // INVERTED output: if it's falling forward, drive forward to catch it.
  if (motorsEnabled) {
    setMotors(-output, -output);
  } else {
    setMotors(0, 0); // Safety disable
  }
  handleSerialTuning();
 
  // Leg servo torque/position heartbeat — non-blocking, self-timed off millis(),
  // fires at most once per 1500ms per servo regardless of the 100Hz loop rate above.
  healLegServos();
 
  // Data log (Print at 20Hz so we don't saturate the serial bus).
  // ORIGINAL fields (PITCH/PID_OUT/ENC_L/ENC_R) kept in order; new fields appended.
  if (now - lastPrintTime >= 50000) {
    lastPrintTime = now;
    Serial1.print("PITCH:"); Serial1.print(pitch); Serial1.print(", ");
    Serial1.print("PID_OUT:"); Serial1.print(output); Serial1.print(", ");
    Serial1.print("ENC_L:"); Serial1.print(encoderLeft); Serial1.print(", ");
    Serial1.print("ENC_R:"); Serial1.print(encoderRight); Serial1.print(", ");
    Serial1.print("VEL:"); Serial1.print(wheelVelocity); Serial1.print(", ");
    Serial1.print("VEL_MMS:"); Serial1.print(linearVelMmS, 2); Serial1.print(", ");
    Serial1.print("VEL_MS:"); Serial1.print(linearVelMS, 4); Serial1.print(", ");
    Serial1.print("KDVEL:"); Serial1.print(Kd_vel, 4); Serial1.print(", ");
    Serial1.print("ALPHA:"); Serial1.print(alpha, 4); Serial1.print(", ");
    Serial1.print("TILT:"); Serial1.println(maxSafeTilt);
  }
}
