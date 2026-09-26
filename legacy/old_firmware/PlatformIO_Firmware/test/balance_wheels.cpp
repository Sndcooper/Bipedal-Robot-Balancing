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
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  
  pitch = atan2(-ax, sqrt((long)ay*ay + (long)az*az)) * 180.0 / PI - pitchOffset;
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

void setup() {
  Serial1.begin(115200);
  Serial2.begin(1000000); // AX-12s
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);
  
  setupMPU();
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0.005) return; // 200Hz max
  lastTime = now;
  
  readIMU();
  
  // Read Encoders to factor into PID (e.g. cascaded position/speed PID)
  // For now, simple standard balancing PID:
  float error = targetAngle - pitch;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Basic balancing approach: feed output directly to motor PWMs
  setMotors(output, output); 
  
  // Data log
  Serial1.print(pitch); Serial1.print(",");
  Serial1.print(output); Serial1.print(",");
  Serial1.print(encoderLeft); Serial1.print(",");
  Serial1.println(encoderRight);
}
