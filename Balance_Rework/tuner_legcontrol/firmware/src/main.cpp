#include <Arduino.h>
#include <Wire.h>

// --- ENCODER SETTINGS ---
#define ENC_L_A PA6
#define ENC_L_B PA7
#define ENC_R_A PB0
#define ENC_R_B PB1

volatile long encoderLeft = 0;
volatile long encoderRight = 0;
long prevEncoderLeft = 0;
long prevEncoderRight = 0;

void countLeft() { if (digitalRead(ENC_L_B)) encoderLeft--; else encoderLeft++; }
void countRight() { if (digitalRead(ENC_R_B)) encoderRight--; else encoderRight++; }

// --- MOTOR PINS ---
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
float Kp = 78.0, Ki = 0.0, Kd = 0.0;
float targetAngle = 0.0;
float integral = 0.0, prevError = 0.0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

float alpha = 0.96;
float Kd_vel = 0.0;
float smoothedVelocity = 0.0f;
float velFilterAlpha = 0.15f;
#define MAX_SAFE_TILT_DEFAULT 25.0f
float maxSafeTilt = MAX_SAFE_TILT_DEFAULT;
#define GYRO_PITCH_SIGN 1.0f

float accelPitchRaw = 0.0;
bool motorsEnabled = false; 
bool safetyLatched = false; 

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

// ── NEW: State-Aware Leg Management ──────────────────────────────────────────
// Tracks the latest GUI-commanded values so the heartbeat doesn't fight the GUI.
struct ServoState {
  uint8_t id;
  uint16_t goalPos;
  uint16_t torqueLimit;
  uint8_t compMargin;
  uint8_t compSlope;
};

ServoState legServos[4] = {
  {6,  818, 511, 4, 32},
  {0,  818, 511, 4, 32},
  {14, 441, 511, 4, 32},
  {1,  441, 511, 4, 32},
};

void initAX12Legs() {
  Serial1.println("Locking AX-12 Legs to home position...");
  for(int i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    ax12WriteByte(id, 24, 1);                           // Torque Enable
    ax12WriteWord(id, 34, legServos[i].torqueLimit);    // Torque Limit
    ax12WriteByte(id, 26, legServos[i].compMargin);     // CW Compliance Margin
    ax12WriteByte(id, 27, legServos[i].compMargin);     // CCW Compliance Margin
    ax12WriteByte(id, 28, legServos[i].compSlope);      // CW Compliance Slope
    ax12WriteByte(id, 29, legServos[i].compSlope);      // CCW Compliance Slope
    ax12WriteWord(id, 30, legServos[i].goalPos);        // Goal Position
  }
}

// ── NEW: Non-Blocking Polling & Healing State Machine ─────────────────────────
enum PollState { POLL_IDLE, POLL_WAITING };
PollState pollState = POLL_IDLE;
unsigned long lastPollTime = 0;
unsigned long waitStartTime = 0;
uint8_t currentServoIdx = 0;
const unsigned long POLL_INTERVAL_MS = 2500; // 1 servo per 2.5s -> All 4 every 10s

void pollLegServosTask() {
  unsigned long now = millis();

  if (pollState == POLL_IDLE) {
    if (now - lastPollTime >= POLL_INTERVAL_MS) {
      ServoState &s = legServos[currentServoIdx];
      
      // 1. Heal the servo state silently (in case of a brownout/drop)
      ax12WriteByte(s.id, 24, 1);
      ax12WriteWord(s.id, 34, s.torqueLimit);
      ax12WriteWord(s.id, 30, s.goalPos);

      // 2. Clear RX buffer to drop all ECHOES from the heal commands above
      while(Serial2.available()) Serial2.read();

      // 3. Send READ command for Address 40 (Load, 2b), 42 (Volt, 1b), 43 (Temp, 1b). Total 4 bytes.
      uint8_t checksum = ~(s.id + 4 + 2 + 40 + 4) & 0xFF;
      uint8_t packet[] = {0xFF, 0xFF, s.id, 0x04, 0x02, 40, 4, checksum};
      Serial2.write(packet, 8); // This generates exactly an 8-byte echo!

      pollState = POLL_WAITING;
      waitStartTime = now;
    }
  } 
  else if (pollState == POLL_WAITING) {
    // We expect 8 bytes of ECHO + 10 bytes of REPLY = 18 bytes.
    if (Serial2.available() >= 18) {
      // Step A: Read and discard the 10k Resistor Hack Echo (8 bytes)
      for (int i = 0; i < 8; i++) Serial2.read();

      // Step B: Read the actual servo reply (10 bytes)
      uint8_t reply[10];
      for (int i = 0; i < 10; i++) reply[i] = Serial2.read();

      // Step C: Verify Header and ID
      if (reply[0] == 0xFF && reply[1] == 0xFF && reply[2] == legServos[currentServoIdx].id) {
        uint16_t loadRaw = reply[5] | (reply[6] << 8);
        uint8_t temp = reply[8];

        // Convert Load to Percentage (Bits 0-9 are magnitude 0-1023)
        uint16_t loadMag = loadRaw & 0x3FF;
        float loadPct = (loadMag / 1023.0f) * 100.0f;

        // Broadcast to Python GUI: SRV:<id>,<temp>,<load%>
        Serial1.print("SRV:");
        Serial1.print(legServos[currentServoIdx].id);
        Serial1.print(",");
        Serial1.print(temp);
        Serial1.print(",");
        Serial1.println(loadPct, 1);
      }

      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime = millis();
      pollState = POLL_IDLE;
    } 
    else if (now - waitStartTime > 20) {
      // TIMEOUT (20ms) - Servo disconnected or dead. 
      // Do not block! Flush buffer, give up, and move to next servo.
      while(Serial2.available()) Serial2.read();
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime = millis();
      pollState = POLL_IDLE;
    }
  }
}

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

  int16_t ax   = Wire.read() << 8 | Wire.read();
  int16_t ay   = Wire.read() << 8 | Wire.read();
  int16_t az   = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();  (void)temp;
  int16_t gx   = Wire.read() << 8 | Wire.read();  (void)gx;
  int16_t gy   = Wire.read() << 8 | Wire.read();

  accelPitchRaw = atan2((float)-ax, sqrt((float)ay*(float)ay + (float)az*(float)az)) * 180.0 / PI;
  float accelPitch = accelPitchRaw - pitchOffset;
  float gyroRate = GYRO_PITCH_SIGN * (float)gy / 131.0;
  pitch = alpha * (pitch + gyroRate * dt) + (1.0 - alpha) * accelPitch;
}

void calibrateIMU() {
  Serial1.println("Calibrating IMU... Keep robot still and upright.");
  long double sum = 0;
  for(int i = 0; i < 100; i++) {
    readIMU(0.01);
    sum += accelPitchRaw;
    delay(10);
  }
  pitchOffset = (float)(sum / 100.0);
  pitch = 0.0;
  Serial1.print("Calibration complete. Offset: ");
  Serial1.println(pitchOffset);
}

void setMotors(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); } 
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  analogWrite(ENA, abs(leftPWM));

  if (rightPWM >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); } 
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  analogWrite(ENB, abs(rightPWM));
}

// ── NEW: GUI Command Parser ──────────────────────────────────────────────────
void handleSerialTuning() {
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    // NEW: Leg Tab Commands (Format: POS,id,val / TRQ,id,limit / CMP,id,margin,slope)
    if (input.startsWith("POS,") || input.startsWith("TRQ,") || input.startsWith("CMP,")) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int id = input.substring(firstComma + 1, secondComma).toInt();
      
      if (input.startsWith("POS,")) {
        int val = input.substring(secondComma + 1).toInt();
        ax12WriteWord(id, 30, val);
        for(int i=0; i<4; i++) if(legServos[i].id == id) legServos[i].goalPos = val;
      } 
      else if (input.startsWith("TRQ,")) {
        int val = input.substring(secondComma + 1).toInt();
        ax12WriteWord(id, 34, val);
        for(int i=0; i<4; i++) if(legServos[i].id == id) legServos[i].torqueLimit = val;
      }
      else if (input.startsWith("CMP,")) {
        int thirdComma = input.indexOf(',', secondComma + 1);
        int margin = input.substring(secondComma + 1, thirdComma).toInt();
        int slope = input.substring(thirdComma + 1).toInt();
        
        ax12WriteByte(id, 26, margin); // CW Margin
        ax12WriteByte(id, 27, margin); // CCW Margin
        ax12WriteByte(id, 28, slope);  // CW Slope
        ax12WriteByte(id, 29, slope);  // CCW Slope
        for(int i=0; i<4; i++) {
          if(legServos[i].id == id) {
            legServos[i].compMargin = margin;
            legServos[i].compSlope = slope;
          }
        }
      }
      return; // Skip the noisy Serial1.print status dump below for high-freq Leg commands
    }
    
    // Legacy PID Tuning Commands
    else if (input.startsWith("P")) Kp = input.substring(1).toFloat();
    else if (input.startsWith("I")) Ki = input.substring(1).toFloat();
    else if (input.startsWith("D")) Kd = input.substring(1).toFloat();
    else if (input == "S") { initAX12Legs(); Serial1.println("Servos reset to home!"); }
    else if (input.startsWith("S") && input.length() > 1) targetAngle = input.substring(1).toFloat();
    else if (input.startsWith("V")) Kd_vel = input.substring(1).toFloat();
    else if (input.startsWith("A")) alpha = input.substring(1).toFloat();
    else if (input.startsWith("T") && !input.startsWith("TRQ,")) maxSafeTilt = input.substring(1).toFloat();
    else if (input.startsWith("C")) calibrateIMU();
    else if (input == "R") { integral = 0.0; Serial1.println("Integral Reset!"); }
    else if (input.startsWith("M")) {
      motorsEnabled = !motorsEnabled;
      if (motorsEnabled) { safetyLatched = false; integral = 0.0; }
      Serial1.print("Motors "); Serial1.println(motorsEnabled ? "ENABLED" : "DISABLED");
    }

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

void setup() {
  Serial1.begin(115200);
  delay(2000); 
  Serial2.begin(1000000);
  initAX12Legs();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  setupMPU();
  lastTime = micros(); 
  lastPollTime = millis();
}

void loop() {
  unsigned long now = micros();
  if (now - lastTime < 10000) return; // 100Hz loop

  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  readIMU(dt);

  if (fabs(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral = 0.0;
    setMotors(0, 0);
    Serial1.println("SAFETY CUTOFF TRIGGERED");
  }

  long encL = encoderLeft;
  long encR = encoderRight;
  float rawVelocity = ((float)((encL - prevEncoderLeft) + (encR - prevEncoderRight)) * 0.5) / dt;
  smoothedVelocity = (velFilterAlpha * rawVelocity) + ((1.0f - velFilterAlpha) * smoothedVelocity);
  prevEncoderLeft = encL;
  prevEncoderRight = encR;

  float linearVelMmS = smoothedVelocity * (3.14159265f * 67.0f / 330.0f);
  float linearVelMS  = linearVelMmS / 1000.0f;

  float error = targetAngle - pitch;

  if (!motorsEnabled) {
    integral = 0.0;
    prevError = error; 
  } else {
    integral += error * dt;
  }

  float derivative = (error - prevError) / dt;
  prevError = error;

  float activeKi = motorsEnabled ? Ki : 0.0;
  float output = (Kp * error) + (activeKi * integral) + (Kd * derivative);
  output -= Kd_vel * smoothedVelocity;

  if (motorsEnabled) { setMotors(-output, -output); } 
  else { setMotors(0, 0); }
  
  handleSerialTuning();

  // ── Call the new non-blocking state machine ──
  pollLegServosTask();

  if (now - lastPrintTime >= 50000) {
    lastPrintTime = now;
    Serial1.print("PITCH:"); Serial1.print(pitch); Serial1.print(", ");
    Serial1.print("PID_OUT:"); Serial1.print(output); Serial1.print(", ");
    Serial1.print("ENC_L:"); Serial1.print(encoderLeft); Serial1.print(", ");
    Serial1.print("ENC_R:"); Serial1.print(encoderRight); Serial1.print(", ");
    Serial1.print("VEL:"); Serial1.print(smoothedVelocity); Serial1.print(", ");
    Serial1.print("VEL_MMS:"); Serial1.print(linearVelMmS, 2); Serial1.print(", ");
    Serial1.print("VEL_MS:"); Serial1.print(linearVelMS, 4); Serial1.print(", ");
    Serial1.print("KDVEL:"); Serial1.print(Kd_vel, 4); Serial1.print(", ");
    Serial1.print("ALPHA:"); Serial1.print(alpha, 4); Serial1.print(", ");
    Serial1.print("TILT:"); Serial1.println(maxSafeTilt);
  }
}