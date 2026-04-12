#include <Arduino.h>

// Initialize HardwareSerial for Serial3 (USART3) for Nano IMU
// RX = PB11, TX = PB10
HardwareSerial Serial3(USART3);

// --- LEFT MOTOR (Motor 1) ---
#define ENA PA0  // Speed (PWM)
#define IN1 PB12 // Direction 1
#define IN2 PB13 // Direction 2

// --- RIGHT MOTOR (Motor 2) ---
#define ENB PA1  // Speed (PWM)
#define IN3 PB14 // Direction 1
#define IN4 PB15 // Direction 2

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
float Kp = 4.0;
float Ki = 0.0;
float Kd = 2.0;

float roll = 0.0;
float pitch = 0.0;
float setpoint = 0.0;
float integral = 0.0;

// Movement & Steering
float turnOffset = 0.0;
float speedOffset = 0.0;
float leanAngle = 2.0;
bool calibratingCOM = false;
int calibCount = 0;
float calibSum = 0.0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

// ─────────────────────────────────────────────────
//  RELAY AUTO-TUNE STATE
// ─────────────────────────────────────────────────
bool relayMode = false;    // true while relay tuning
float relayAmp = 60.0;     // relay output amplitude (0-120)
int relayCycles = 0;       // full zero-crossing cycles counted
int relayTargetCycles = 8; // how many cycles to average over
float relayPrevError = 0.0;
unsigned long relayLastCross = 0;
float relayPeriodSum = 0.0;
float relayPeakPos = 0.0;   // max positive error seen
float relayPeakNeg = 0.0;   // min negative error seen
bool relayHighSide = false; // current relay side
bool relayReady = false;    // tuning done

// --- MOTOR FUNCTIONS ---
void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void setMotors(float leftOut, float rightOut) {
  leftOut = constrain(leftOut, -120, 120);
  rightOut = constrain(rightOut, -120, 120);

  // Deadband compensation
  int deadband = 30;
  if (leftOut > 1)
    leftOut += deadband;
  else if (leftOut < -1)
    leftOut -= deadband;
  if (rightOut > 1)
    rightOut += deadband;
  else if (rightOut < -1)
    rightOut -= deadband;

  // Left Motor
  if (leftOut >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftOut);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftOut);
  }

  // Right Motor
  if (rightOut >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightOut);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightOut);
  }
}

// --- SETUP ---
void setup() {
  Serial1.begin(250000);
  Serial3.begin(250000);

  setupMotors();

  delay(1000);
  while (Serial3.available())
    Serial3.read();

  Serial1.println("Balancing Robot Ready.");
  Serial1.println("Commands: P<val> I<val> D<val> O<val>");
  Serial1.println("         C=CalibCOM  R<amp>=RelayTune  W/A/S/D/X=Drive");

  lastTime = micros();
}

// --- MAIN LOOP ---
void loop() {

  // 1. CHECK FOR IMU PACKETS ON SERIAL3
  while (Serial3.available() > 0) {
    byte incomingByte = Serial3.read();

    switch (currentState) {
    case WAIT_HEADER_1:
      if (incomingByte == 0xAA)
        currentState = WAIT_HEADER_2;
      break;
    case WAIT_HEADER_2:
      if (incomingByte == 0xBB) {
        currentState = READ_PAYLOAD;
        bytesRead = 0;
      } else if (incomingByte != 0xAA)
        currentState = WAIT_HEADER_1;
      break;
    case READ_PAYLOAD:
      payloadBuffer[bytesRead++] = incomingByte;
      if (bytesRead >= sizeof(SensorData)) {
        memcpy(&receivedData, payloadBuffer, sizeof(SensorData));
        currentState = WAIT_HEADER_1;

        // --- 2. CALCULATE ANGLES ---
        // NOTE — Nano physical orientation:
        //   ROLL  (gx, atan2(ay,az)) = FORWARD / BACKWARD tilt  → drives motors
        //   PITCH (gy, atan2(-ax,…)) = SIDEWAYS lean            → left leg higher than right
        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;
        if (dt > 0.1)
          dt = 0.01;

        // Forward/backward tilt from accelerometer (used for balancing PID)
        float accRoll = atan2(receivedData.ay, receivedData.az) * 180.0 / PI;

        // Sideways lean from accelerometer (left leg higher = positive pitch)
        float accPitch =
            atan2(-receivedData.ax, sqrt(receivedData.ay * receivedData.ay +
                                         receivedData.az * receivedData.az)) *
            180.0 / PI;

        float gyroRollRate  = receivedData.gx; // forward/backward angular rate
        float gyroPitchRate = receivedData.gy; // sideways angular rate

        // Complementary filter — forward/backward (roll)
        roll  = 0.98 * (roll  + gyroRollRate  * dt) + 0.02 * accRoll;
        // Complementary filter — sideways lean (pitch)
        pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accPitch;

        // --- SAFETY CUTOFF ---
        // Stop motors if fallen too far forward/backward OR tipped sideways
        if (roll > 45.0 || roll < -45.0 || pitch > 40.0 || pitch < -40.0) {
          setMotors(0, 0);
          integral = 0;
          break;
        }

        // --- CALIBRATE COM ---
        if (calibratingCOM) {
          calibSum += roll;
          calibCount++;
          if (calibCount >= 100) {
            setpoint = calibSum / 100.0;
            calibratingCOM = false;
            Serial1.print("\n=== CALIBRATION COMPLETE! Center: ");
            Serial1.print(setpoint);
            Serial1.println(" ===");
          }
          setMotors(0, 0);
          integral = 0;
          break;
        }

        // ─────────────────────────────────────────────
        //  RELAY AUTO-TUNE MODE
        // ─────────────────────────────────────────────
        if (relayMode) {
          float error = roll - setpoint;

          // Bang-bang relay output
          float relayOut;
          if (error < 0) {
            relayOut = relayAmp; // lean one way
            relayHighSide = true;
          } else {
            relayOut = -relayAmp; // lean other way
            relayHighSide = false;
          }

          // Detect zero-crossings (sign change)
          if (relayPrevError != 0 && ((relayPrevError < 0 && error >= 0) ||
                                      (relayPrevError >= 0 && error < 0))) {

            unsigned long now = millis();
            if (relayLastCross > 0 && relayCycles > 0) {
              // Count half-periods; two half-periods = one full period
              float halfPeriod = (now - relayLastCross) / 1000.0;
              relayPeriodSum += halfPeriod * 2.0; // add full period estimate
            }
            relayLastCross = now;
            relayCycles++;

            // Track amplitude
            if (relayPrevError > 0 && relayPrevError > relayPeakPos)
              relayPeakPos = relayPrevError;
            if (relayPrevError < 0 && relayPrevError < relayPeakNeg)
              relayPeakNeg = relayPrevError;
          }
          relayPrevError = error;

          // Enough cycles to compute gains?
          if (relayCycles >= relayTargetCycles * 2 && !relayReady) {
            float Tu = relayPeriodSum / (relayCycles / 2.0); // avg period
            float Au = (relayPeakPos - relayPeakNeg) / 2.0;  // avg amplitude
            if (Au > 0.5 && Tu > 0.1) {
              float Ku = (4.0 * relayAmp) / (PI * Au);
              // Ziegler-Nichols PID formulas
              Kp = 0.6 * Ku;
              Ki = 1.2 * Ku / Tu;
              Kd = 0.075 * Ku * Tu;

              relayReady = true;
              relayMode = false;

              // Report to Python
              Serial1.println("TUNE:DONE");
              Serial1.print("TUNE:Tu=");
              Serial1.println(Tu, 4);
              Serial1.print("TUNE:Au=");
              Serial1.println(Au, 4);
              Serial1.print("TUNE:Ku=");
              Serial1.println(Ku, 4);
              Serial1.print("TUNE:Kp=");
              Serial1.println(Kp, 4);
              Serial1.print("TUNE:Ki=");
              Serial1.println(Ki, 4);
              Serial1.print("TUNE:Kd=");
              Serial1.println(Kd, 4);
            }
            // If amplitude too small: keep going
          }

          // Send relay output (no steering during tuning)
          setMotors(relayOut, relayOut);
          integral = 0;
          break;
        }

        // --- NORMAL PID ---
        float error = roll - (setpoint + speedOffset);
        integral += error * dt;
        integral = constrain(integral, -200, 200);
        float output =
            -1.0 * ((Kp * error) + (Ki * integral) + (Kd * gyroRollRate));
        setMotors(output + turnOffset, output - turnOffset);
      }
      break;
    }
  }

  // 3. READ SERIAL COMMANDS
  if (Serial1.available() > 0) {
    char cmd = Serial1.read();
    float val = Serial1.parseFloat();
    cmd = toupper(cmd);

    if (cmd == 'C') {
      calibratingCOM = true;
      calibCount = 0;
      calibSum = 0.0;
      Serial1.println(">>> CALIBRATING COM — Hold still! <<<");
    } else if (cmd == 'R') {
      // Start relay auto-tune: R<amplitude>  e.g. R60
      relayMode = true;
      relayReady = false;
      if (val > 5.0)
        relayAmp = val;
      relayCycles = 0;
      relayPeriodSum = 0;
      relayPrevError = 0;
      relayLastCross = 0;
      relayPeakPos = 0;
      relayPeakNeg = 0;
      integral = 0;
      Serial1.print("TUNE:START amp=");
      Serial1.println(relayAmp);
    } else if (cmd == 'Q') {
      // Abort relay tune
      relayMode = false;
      setMotors(0, 0);
      Serial1.println("TUNE:ABORTED");
    } else if (cmd == 'P') {
      Kp = val;
      Serial1.print("Kp=");
      Serial1.println(Kp);
    } else if (cmd == 'I') {
      Ki = val;
      Serial1.print("Ki=");
      Serial1.println(Ki);
    } else if (cmd == 'D') {
      Kd = val;
      Serial1.print("Kd=");
      Serial1.println(Kd);
    } else if (cmd == 'O') {
      setpoint = val;
      Serial1.print("Setpoint=");
      Serial1.println(setpoint);
    } else if (cmd == 'W') {
      speedOffset = -leanAngle;
      Serial1.println("Forward");
    } else if (cmd == 'S') {
      speedOffset = leanAngle;
      Serial1.println("Backward");
    } else if (cmd == 'A') {
      turnOffset = -20.0;
      Serial1.println("Left");
    } else if (cmd == 'D') {
      turnOffset = 20.0;
      Serial1.println("Right");
    } else if (cmd == 'X') {
      speedOffset = 0;
      turnOffset = 0;
      Serial1.println("Stopped");
    } else if (cmd == '+') {
      leanAngle += 0.5;
      Serial1.print("LeanAngle=");
      Serial1.println(leanAngle);
    } else if (cmd == '-') {
      leanAngle -= 0.5;
      if (leanAngle < 0.0)
        leanAngle = 0.0;
      Serial1.print("LeanAngle=");
      Serial1.println(leanAngle);
    }
  }

  // 4. STRUCTURED TELEMETRY (10 Hz) — parseable by Python
  if (millis() - lastPrintTime > 100) {
    Serial1.print("TELEM:");
    Serial1.print(roll, 3);
    Serial1.print(",");
    Serial1.print(pitch, 3);
    Serial1.print(",");
    Serial1.print(Kp, 3);
    Serial1.print(",");
    Serial1.print(Ki, 3);
    Serial1.print(",");
    Serial1.print(Kd, 3);
    Serial1.print(",");
    Serial1.print(setpoint, 3);
    Serial1.print(",");
    Serial1.println(relayMode ? 1 : 0);
    lastPrintTime = millis();
  }
}
