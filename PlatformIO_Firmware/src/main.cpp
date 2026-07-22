#include <Arduino.h>
#include <Wire.h>

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
float Kp = 20.0, Ki = 0.5, Kd = 1.0;
float targetAngle = 0.0;
float integral = 0.0, prevError = 0.0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

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
  Serial1.println("Locking AX-12 Legs to 90 degrees...");
  uint8_t left_servos[] = {6, 0};
  uint8_t right_servos[] = {14, 1};
  uint16_t left_positions[] = {821, 825};
  uint16_t right_positions[] = {448, 455};

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

void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  
  pitch = atan2((float)-ax, sqrt((float)ay*(float)ay + (float)az*(float)az)) * 180.0 / PI - pitchOffset;
}

bool motorsEnabled = false; // Add safe toggle

void calibrateIMU() {
  Serial1.println("Calibrating IMU... Keep robot still and upright.");
  long sum = 0;
  for(int i = 0; i < 100; i++) {
    readIMU();
    sum += pitch + pitchOffset; // Get raw pitch
    delay(10);
  }
  pitchOffset = sum / 100.0;
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
    else if (input.startsWith("O")) pitchOffset = input.substring(1).toFloat();
    else if (input == "S") {
      initAX12Legs();
      Serial1.println("Servos reset to home position!");
    }
    else if (input.startsWith("S") && input.length() > 1) targetAngle = input.substring(1).toFloat();
    else if (input.startsWith("C")) calibrateIMU();
    else if (input.startsWith("M")) { 
      motorsEnabled = !motorsEnabled; 
      Serial1.print("Motors "); Serial1.println(motorsEnabled ? "ENABLED" : "DISABLED"); 
    }
    
    Serial1.print("Updated -> P:"); Serial1.print(Kp);
    Serial1.print(" I:"); Serial1.print(Ki);
    Serial1.print(" D:"); Serial1.print(Kd);
    Serial1.print(" Offset:"); Serial1.print(pitchOffset);
    Serial1.print(" Target:"); Serial1.println(targetAngle);
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
}

void loop() {
  unsigned long now = micros();
  // 100Hz fixed loop (10000 microseconds)
  if (now - lastTime < 10000) return; 
  
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  
  readIMU();
  
  // ── Encoder-derived wheel velocity (ticks/sec & linear velocity) ────────
  long encL = encoderLeft;
  long encR = encoderRight;
  float wheelVelocity = ((float)((encL - prevEncoderLeft) + (encR - prevEncoderRight)) * 0.5) / dt;
  prevEncoderLeft = encL;
  prevEncoderRight = encR;

  // 330 ticks per rotation, 67mm wheel diameter -> circumference = PI * 67mm
  float linearVelMmS = wheelVelocity * (3.14159265f * 67.0f / 330.0f); // mm/s
  float linearVelMS  = linearVelMmS / 1000.0f; // m/s

  // Read Encoders to factor into PID (e.g. cascaded position/speed PID)
  // For now, simple standard balancing PID:
  float error = targetAngle - pitch;
  
  // Prevent Integral Windup: Only build integral if motors are actually running
  if (!motorsEnabled) {
    integral = 0.0;
    prevError = error; // Prevents massive derivative spike when turning motors back on
  } else {
    integral += error * dt;
  }
  
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Basic balancing approach: feed output directly to motor PWMs
  // INVERTED output: If it's falling forward, drive forward to catch it.
  if (motorsEnabled) {
    setMotors(-output, -output); 
  } else {
    setMotors(0, 0); // Safety disable
  }
  handleSerialTuning();
  
  // Data log (Print at 20Hz so we don't saturate the serial bus)
  if (now - lastPrintTime >= 50000) { 
    lastPrintTime = now;
    Serial1.print("PITCH:"); Serial1.print(pitch); Serial1.print(", ");
    Serial1.print("PID_OUT:"); Serial1.print(output); Serial1.print(", ");
    Serial1.print("ENC_L:"); Serial1.print(encoderLeft); Serial1.print(", ");
    Serial1.print("ENC_R:"); Serial1.print(encoderRight); Serial1.print(", ");
    Serial1.print("VEL:"); Serial1.print(wheelVelocity); Serial1.print(", ");
    Serial1.print("VEL_MMS:"); Serial1.print(linearVelMmS, 2); Serial1.print(", ");
    Serial1.print("VEL_MS:"); Serial1.println(linearVelMS, 4);
  }
}
