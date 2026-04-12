#include <Arduino.h>

// Initialize HardwareSerial for Serial3 (USART3) for Nano IMU
// RX = PB11, TX = PB10
HardwareSerial Serial3(USART3);

// --- LEFT MOTOR (Motor 1) ---
#define ENA  PA0  // Speed (PWM)
#define IN1  PB12 // Direction 1
#define IN2  PB13 // Direction 2

// --- RIGHT MOTOR (Motor 2) ---
#define ENB  PA1  // Speed (PWM)
#define IN3  PB14 // Direction 1
#define IN4  PB15 // Direction 2

// --- SENSOR DATA STRUCT ---
struct SensorData {
  float ax, ay, az; // Accelerometer
  float gx, gy, gz; // Gyroscope
  float mx, my, mz; // Magnetometer
};
SensorData receivedData;

// State machine for reading IMU
enum ReadState { WAIT_HEADER_1, WAIT_HEADER_2, READ_PAYLOAD };
ReadState currentState = WAIT_HEADER_1;
uint8_t payloadBuffer[sizeof(SensorData)];
int bytesRead = 0;

// --- PID & BALANCING VARIABLES ---
float Kp = 15.0;
float Ki = 0.0;
float Kd = 2.0;

float roll = 0.0;
float setpoint = 0.0; // Target angle (Offset calibrated automatically)
float integral = 0.0;

// Movement & Steering
float turnOffset = 0.0; 
float speedOffset = 0.0;
float leanAngle = 2.0; // The target angle to lean to move forward/backward
bool calibratingCOM = false;
int calibCount = 0;
float calibSum = 0.0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

// --- MOTOR FUNCTIONS ---
void setupMotors() {
  pinMode(ENA, OUTPUT);  pinMode(IN1, OUTPUT);  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);  pinMode(IN3, OUTPUT);  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}

void setMotors(float leftOut, float rightOut) {
  // Constrain limits (Drastically reduced to keep motors slow and safe!)
  leftOut = constrain(leftOut, -120, 120);
  rightOut = constrain(rightOut, -120, 120);

  // Deadband compensation (overcome physical motor friction)
  int deadband = 30; 
  if (leftOut > 1) leftOut += deadband;
  else if (leftOut < -1) leftOut -= deadband;
  
  if (rightOut > 1) rightOut += deadband;
  else if (rightOut < -1) rightOut -= deadband;

  // Left Motor
  if (leftOut >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, leftOut);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftOut);
  }

  // Right Motor
  if (rightOut >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, rightOut);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightOut);
  }
}

// --- SETUP ---
void setup() {
  Serial1.begin(250000); // User interface / Tuning
  Serial3.begin(250000); // IMU from Nano
  
  setupMotors();

  // Flush Serial3
  delay(1000);
  while(Serial3.available()) Serial3.read();
  
  Serial1.println("Balancing Robot Ready.");
  Serial1.println("Send: P<val>, I<val>, D<val> to tune.");
  Serial1.println("Send: W, A, S, D, X to drive/stop.");
  
  lastTime = micros();
}

// --- MAIN LOOP ---
void loop() {
  
  // 1. CHECK FOR IMU PACKETS ON SERIAL3
  while (Serial3.available() > 0) {
    byte incomingByte = Serial3.read();

    switch (currentState) {
      case WAIT_HEADER_1:
        if (incomingByte == 0xAA) currentState = WAIT_HEADER_2;
        break;
      case WAIT_HEADER_2:
        if (incomingByte == 0xBB) { currentState = READ_PAYLOAD; bytesRead = 0; } 
        else if (incomingByte != 0xAA) currentState = WAIT_HEADER_1;
        break;
      case READ_PAYLOAD:
        payloadBuffer[bytesRead++] = incomingByte;
        if (bytesRead >= sizeof(SensorData)) {
          memcpy(&receivedData, payloadBuffer, sizeof(SensorData));
          currentState = WAIT_HEADER_1;
          
          // --- 2. CALCULATE ROLL & PID ---
          unsigned long currentTime = micros();
          float dt = (currentTime - lastTime) / 1000000.0;
          lastTime = currentTime;
          if (dt > 0.1) dt = 0.01; // Avoid huge spikes on startup

          // Calculate Accel Roll. Assuming AY is forward, AZ is up. 
          float accRoll = atan2(receivedData.ay, receivedData.az) * 180.0 / PI;
          
          // Complementary Filter combining Gyro and Accel
          float gyroRate = receivedData.gx; // Rate of roll change (deg/s)
          roll = 0.98 * (roll + gyroRate * dt) + 0.02 * accRoll;

          // Safety check: if fallen over, stop motors to prevent crazy spinning
          if (roll > 45.0 || roll < -45.0) {
             setMotors(0, 0);
             integral = 0;
             break; 
          }

          // --- AUTO-CALIBRATE COM MODE ---
          if (calibratingCOM) {
             calibSum += roll;
             calibCount++;
             if (calibCount >= 100) {
                setpoint = calibSum / 100.0;
                calibratingCOM = false;
                Serial1.print("\n=== CALIBRATION COMPLETE! True Center is: ");
                Serial1.print(setpoint);
                Serial1.println(" ===");
             }
             setMotors(0, 0); // Keep motors off during calib
             integral = 0;
             break;
          }

          // Compute PID
          float error = roll - (setpoint + speedOffset);
          integral += error * dt;
          integral = constrain(integral, -200, 200); // Anti-windup
          
          // Use Gyro rate directly for Derivative term
          // INVERTED OUT: If the wheels drive the wrong way, we multiply by -1.0 
          float output = -1.0 * ((Kp * error) + (Ki * integral) + (Kd * gyroRate));

          // Apply limits and steering offsets
          setMotors(output + turnOffset, output - turnOffset);
        }
        break;
    }
  }

  // 3. READ SERIAL FOR TUNING & COMMANDS
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    float val = Serial1.parseFloat();
    
    cmd = toupper(cmd);
    if (cmd == 'C') { // Auto-Calibrate COM
      calibratingCOM = true; 
      calibCount = 0; 
      calibSum = 0.0;
      Serial1.println("\n>>> CALIBRATING COM... Hold robot PERFECTLY STILL safely upright for 1 second! <<<");
    }
    else if (cmd == 'P') { Kp = val; Serial1.print("Kp set to: "); Serial1.println(Kp); }
    else if (cmd == 'I') { Ki = val; Serial1.print("Ki set to: "); Serial1.println(Ki); }
    else if (cmd == 'D') { Kd = val; Serial1.print("Kd set to: "); Serial1.println(Kd); }
    else if (cmd == 'O') { setpoint = val; Serial1.print("Setpoint set to: "); Serial1.println(setpoint); }
    else if (cmd == 'W') { speedOffset = -leanAngle; Serial1.println("Moving Forward"); } // Lean forward
    else if (cmd == 'S') { speedOffset = leanAngle; Serial1.println("Moving Backward"); }  // Lean back
    else if (cmd == 'A') { turnOffset = -20.0; Serial1.println("Turning Left"); }
    else if (cmd == 'D') { turnOffset = 20.0; Serial1.println("Turning Right"); }
    else if (cmd == 'X') { speedOffset = 0; turnOffset = 0; Serial1.println("Stopped"); }
    else if (cmd == '+') { leanAngle += 0.5; Serial1.print("Speed (Lean Angle) increased to: "); Serial1.println(leanAngle); }
    else if (cmd == '-') { 
      leanAngle -= 0.5; 
      if (leanAngle < 0.0) leanAngle = 0.0; 
      Serial1.print("Speed (Lean Angle) decreased to: "); Serial1.println(leanAngle); 
    }
  }

  // 4. TELEMETRY OUTPUT (4 times a second)
  if (millis() - lastPrintTime > 250) {
    Serial1.print("Roll: "); Serial1.print(roll);
    Serial1.print(" | Kp: "); Serial1.print(Kp);
    Serial1.print(" | Ki: "); Serial1.print(Ki);
    Serial1.print(" | Kd: "); Serial1.println(Kd);
    lastPrintTime = millis();
  }
}
