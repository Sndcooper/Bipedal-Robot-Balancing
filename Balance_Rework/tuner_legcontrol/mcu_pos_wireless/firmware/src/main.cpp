// mcu_pos_wireless firmware — STM32 Bluepill (F103C8)
// Dedicated single-position balancing & encoder target position tuner.
// 3DR telemetry on Serial3 (PB10 TX / PB11 RX) @ 115200
// AX-12 servo bus on Serial2 (USART2) @ 1 Mbaud

#include <Arduino.h>
#include <Wire.h>

// ── ENCODER PINS ─────────────────────────────────────────────────────────────
#define ENC_L_A PA6
#define ENC_L_B PA7
#define ENC_R_A PB0
#define ENC_R_B PB1

volatile long encoderLeft = 0;
volatile long encoderRight = 0;
long prevEncoderLeft = 0;
long prevEncoderRight = 0;

void countLeft() {
  if (digitalRead(ENC_L_B))
    encoderLeft--;
  else
    encoderLeft++;
}
void countRight() {
  if (digitalRead(ENC_R_B))
    encoderRight--;
  else
    encoderRight++;
}

// ── MOTOR PINS ───────────────────────────────────────────────────────────────
#define ENA PA1
#define IN1 PB14
#define IN2 PB15
#define ENB PA0
#define IN3 PB12
#define IN4 PB13

// ── SERIAL PORTS ─────────────────────────────────────────────────────────────
//   -DENABLE_HWSERIAL2  →  Serial2 on USART2 (AX-12 servos)
//   -DENABLE_HWSERIAL3  →  Serial3 on USART3 (PB10 TX, PB11 RX) = 3DR radio
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;

// ── MPU6050 ──────────────────────────────────────────────────────────────────
const int MPU_ADDR = 0x68;
float pitch = 0.0f, pitchOffset = 0.0f;
float accelPitchRaw = 0.0f;
float gyroRate = 0.0f;
#define GYRO_PITCH_SIGN 1.0f

// ── PID & TUNING ─────────────────────────────────────────────────────────────
// ── LAYER 1: BALANCE PID (inner) ────────────────────────────────────────────
float Kp = 80.0f, Ki = 0.0f, Kd = 0.0f;
float gui_base_angle = 0.0f;
float integral = 0.0f;
float alpha = 0.96f;
float maxSafeTilt = 25.0f;

// ── LAYER 2: VELOCITY → TILT PID (middle) ───────────────────────────────────
float Kp_vel = 0.02f;             // velocity error → tilt bias gain
float Ki_vel = 0.001f;            // velocity integrator gain
float integral_vel = 0.0f;        // velocity loop integrator
float tilt_bias = 0.0f;           // output of velocity loop (degrees)
const float MAX_TILT_BIAS = 5.0f; // anti-windup hard clamp on tilt command

// ── LAYER 3: POSITION HOLD (outer, P-only) ──────────────────────────────────
float Kp_pos = 0.5f;       // position error → velocity command gain
float home_enc_pos = 0.0f; // baseline home position latched on enable/calib
float target_pos_offset =
    0.0f;                    // commanded target offset in ticks (±3000 ticks)
float target_enc_pos = 0.0f; // target encoder position = home + offset
const float MAX_VEL_CMD = 800.0f; // max velocity target (counts/sec)

float turn_bias = 0.0f; // steering bias commanded from GUI

float target_velocity = 0.0f; // commanded velocity (counts/sec)
float pos_error = 0.0f;       // outer loop error (counts)
float vel_error = 0.0f;       // middle loop error (counts/sec)

float vel_current = 0.0f; // EMA-filtered wheel velocity (counts/sec)
float vel_alpha = 0.85f;  // EMA filter coefficient for vel_current (0..0.99)

bool motorsEnabled = false;
bool safetyLatched = false;

// ── LOOP TIMING ──────────────────────────────────────────────────────────────
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;

// ── AX-12 HELPERS ────────────────────────────────────────────────────────────
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

void ax12SyncWritePositions(uint16_t pos6, uint16_t pos14, uint16_t pos0,
                            uint16_t pos1) {
  uint8_t packet[20];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFE; // Broadcast ID
  packet[3] = 16;   // Length
  packet[4] = 0x83; // SYNC_WRITE
  packet[5] = 30;   // Address: Goal Position
  packet[6] = 2;    // Data length per servo
  packet[7] = 6;
  packet[8] = pos6 & 0xFF;
  packet[9] = (pos6 >> 8) & 0xFF;
  packet[10] = 14;
  packet[11] = pos14 & 0xFF;
  packet[12] = (pos14 >> 8) & 0xFF;
  packet[13] = 0;
  packet[14] = pos0 & 0xFF;
  packet[15] = (pos0 >> 8) & 0xFF;
  packet[16] = 1;
  packet[17] = pos1 & 0xFF;
  packet[18] = (pos1 >> 8) & 0xFF;
  uint32_t sum = 0;
  for (int i = 2; i < 19; i++)
    sum += packet[i];
  packet[19] = ~(sum & 0xFF);
  Serial2.write(packet, 20);
  Serial2.flush();
}

// ── LEG SERVO STATE ──────────────────────────────────────────────────────────
struct ServoState {
  uint8_t id;
  uint16_t goalPos;
  uint16_t torqueLimit;
  uint8_t compMargin;
  uint8_t compSlope;
  uint8_t temp;
  float loadPct;
};

ServoState legServos[4] = {
    {6, 818, 511, 4, 32, 0, 0.0f},
    {0, 818, 511, 4, 32, 0, 0.0f},
    {14, 441, 511, 4, 32, 0, 0.0f},
    {1, 441, 511, 4, 32, 0, 0.0f},
};

void initAX12Legs() {
  for (int i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    ax12WriteByte(id, 16, 1);                        // Status Return Level
    ax12WriteByte(id, 5, 0);                         // Return Delay Time = 0
    ax12WriteByte(id, 24, 1);                        // Torque Enable
    ax12WriteWord(id, 34, legServos[i].torqueLimit); // Torque Limit
    ax12WriteByte(id, 26, legServos[i].compMargin);  // CW Compliance Margin
    ax12WriteByte(id, 27, legServos[i].compMargin);  // CCW Compliance Margin
    ax12WriteByte(id, 28, legServos[i].compSlope);   // CW Compliance Slope
    ax12WriteByte(id, 29, legServos[i].compSlope);   // CCW Compliance Slope
    ax12WriteWord(id, 30, legServos[i].goalPos);     // Goal Position
  }
}

// ── IK ENGINE ────────────────────────────────────────────────────────────────
#define SERVO_L_X -30.0f
#define SERVO_L_Y 0.0f
#define SERVO_R_X 30.0f
#define SERVO_R_Y 0.0f
#define FEMUR_LEN 55.0f
#define TIBIA_LEN 100.0f
#define LEG2_INVERTED_MOUNT true

float ik_fx1 = 1.0f, ik_fy1 = -151.1f;
float ik_fx2 = -6.0f, ik_fy2 = -149.6f;
float ik_dist = 180.0f;
float ik_lean = 0.0f;

struct Point2D {
  float x;
  float y;
};

bool circle_intersections(Point2D p0, float r0, Point2D p1, float r1,
                          Point2D &out1, Point2D &out2) {
  float dx = p1.x - p0.x, dy = p1.y - p0.y;
  float d = sqrtf(dx * dx + dy * dy);
  if (d > r0 + r1 || d < fabsf(r0 - r1) || d == 0)
    return false;
  float a = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  float h = sqrtf(fmaxf(r0 * r0 - a * a, 0.0f));
  float px = p0.x + a * dx / d;
  float py = p0.y + a * dy / d;
  float rx = -h * dy / d, ry = h * dx / d;
  out1 = {px + rx, py + ry};
  out2 = {px - rx, py - ry};
  return true;
}

struct IK_Result {
  bool valid;
  Point2D Knee_L, Knee_R;
  float Angle_L, Angle_R;
};

IK_Result solve_ik(float tx, float ty, float leg_offset_x) {
  IK_Result res = {false};
  Point2D foot = {tx, ty};
  Point2D sl = {SERVO_L_X + leg_offset_x, SERVO_L_Y};
  Point2D sr = {SERVO_R_X + leg_offset_x, SERVO_R_Y};
  Point2D li1, li2, ri1, ri2;
  if (!circle_intersections(sl, FEMUR_LEN, foot, TIBIA_LEN, li1, li2))
    return res;
  if (!circle_intersections(sr, FEMUR_LEN, foot, TIBIA_LEN, ri1, ri2))
    return res;
  res.valid = true;
  res.Knee_L = (li1.x < li2.x) ? li1 : li2;
  res.Knee_R = (ri1.x > ri2.x) ? ri1 : ri2;
  res.Angle_L = atan2f(res.Knee_L.y - sl.y, res.Knee_L.x - sl.x) * 180.0f / PI;
  res.Angle_R = atan2f(res.Knee_R.y - sr.y, res.Knee_R.x - sr.x) * 180.0f / PI;
  return res;
}

uint16_t map_angle_to_ax12(float ik_angle, bool is_left, bool is_leg2) {
  float base_angle = is_leg2 ? 90.0f : -90.0f;
  float diff_deg = fmodf(ik_angle - base_angle + 180.0f, 360.0f);
  if (diff_deg < 0)
    diff_deg += 360.0f;
  diff_deg -= 180.0f;
  float base_pos = is_left ? 818.0f : 441.0f;
  float ax_pos = base_pos + (diff_deg * 3.413f);
  return (uint16_t)constrain((int)ax_pos, 0, 1023);
}

void updateIK() {
  float rad = -ik_lean * PI / 180.0f;
  float c = cosf(rad), s = sinf(rad);
  float rx1 = ik_fx1 * c - ik_fy1 * s, ry1 = ik_fx1 * s + ik_fy1 * c;
  float rx2 = ik_fx2 * c - ik_fy2 * s, ry2 = ik_fx2 * s + ik_fy2 * c;

  IK_Result sol1 = solve_ik(rx1, ry1, 0.0f);
  IK_Result sol2 = solve_ik(rx2 + ik_dist, ry2, ik_dist);

  if (sol1.valid && sol2.valid) {
    uint16_t p6 = map_angle_to_ax12(sol1.Angle_L, true, false);
    uint16_t p14 = map_angle_to_ax12(sol1.Angle_R, false, false);
    float ikL2 = sol2.Angle_L, ikR2 = sol2.Angle_R;
    if (LEG2_INVERTED_MOUNT) {
      ikL2 = -sol2.Angle_R;
      ikR2 = -sol2.Angle_L;
    }
    uint16_t p0 = map_angle_to_ax12(ikL2, true, true);
    uint16_t p1 = map_angle_to_ax12(ikR2, false, true);
    ax12SyncWritePositions(p6, p14, p0, p1);
    for (int i = 0; i < 4; i++) {
      if (legServos[i].id == 6)
        legServos[i].goalPos = p6;
      else if (legServos[i].id == 14)
        legServos[i].goalPos = p14;
      else if (legServos[i].id == 0)
        legServos[i].goalPos = p0;
      else if (legServos[i].id == 1)
        legServos[i].goalPos = p1;
    }
  }
}

// ── SERVO POLL STATE MACHINE ─────────────────────────────────────────────────
enum PollState { POLL_IDLE, POLL_WAITING };
PollState pollState = POLL_IDLE;
unsigned long lastPollTime = 0;
unsigned long waitStartTime = 0;
uint8_t currentServoIdx = 0;
const unsigned long POLL_INTERVAL_MS =
    20; // 1 servo per 20ms → all 4 every 80ms

void pollLegServosTask() {
  unsigned long now = millis();

  if (pollState == POLL_IDLE) {
    if (now - lastPollTime < POLL_INTERVAL_MS)
      return;
    ServoState &s = legServos[currentServoIdx];

    ax12WriteByte(s.id, 24, 1);
    ax12WriteWord(s.id, 34, s.torqueLimit);
    ax12WriteWord(s.id, 30, s.goalPos);

    while (Serial2.available())
      Serial2.read();

    uint8_t checksum = ~(s.id + 4 + 2 + 40 + 4) & 0xFF;
    uint8_t packet[] = {0xFF, 0xFF, s.id, 0x04, 0x02, 40, 4, checksum};
    Serial2.write(packet, 8);

    pollState = POLL_WAITING;
    waitStartTime = now;
  } else if (pollState == POLL_WAITING) {
    if (Serial2.available() >= 18) {
      for (int i = 0; i < 8; i++)
        Serial2.read();

      uint8_t reply[10];
      for (int i = 0; i < 10; i++)
        reply[i] = Serial2.read();

      if (reply[0] == 0xFF && reply[1] == 0xFF &&
          reply[2] == legServos[currentServoIdx].id) {
        uint16_t loadRaw = reply[5] | (reply[6] << 8);
        uint8_t temp = reply[8];
        float loadPct = ((loadRaw & 0x3FF) / 1023.0f) * 100.0f;

        legServos[currentServoIdx].temp = temp;
        legServos[currentServoIdx].loadPct = loadPct;
      }

      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime = millis();
      pollState = POLL_IDLE;
    } else if (now - waitStartTime > 20) {
      while (Serial2.available())
        Serial2.read();
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime = millis();
      pollState = POLL_IDLE;
    }
  }
}

// ── IMU ──────────────────────────────────────────────────────────────────────
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

  accelPitchRaw =
      atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / PI;
  float accelPitch = accelPitchRaw - pitchOffset;
  gyroRate = GYRO_PITCH_SIGN * (float)gy / 131.0f;
  pitch = alpha * (pitch + gyroRate * dt) + (1.0f - alpha) * accelPitch;
}

void calibrateIMU() {
  Serial3.print("CAL START|");
  long double sum = 0;
  for (int i = 0; i < 100; i++) {
    readIMU(0.01f);
    sum += accelPitchRaw;
    delay(10);
  }
  pitchOffset = (float)(sum / 100.0);
  pitch = 0.0f;
  home_enc_pos = (float)(encoderLeft + encoderRight) * 0.5f;
  target_pos_offset = 0.0f;
  char buf[64];
  snprintf(buf, sizeof(buf), "CAL DONE OFFSET%.4f HOME%.1f|", pitchOffset,
           home_enc_pos);
  Serial3.print(buf);
}

// ── MOTOR DRIVER ─────────────────────────────────────────────────────────────
void setMotors(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(leftPWM));

  if (rightPWM >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(rightPWM));
}

// ── COMMAND PARSER ───────────────────────────────────────────────────────────
void parseCommand(char *cmd) {
  char ack[128];

  // ── IK commands ──────────────────────────────────────────────────────────
  if (cmd[0] == 'I' && cmd[1] == 'K') {
    char type = cmd[2];
    char *payload = cmd + 4;
    if (type == '1') {
      char *sp = strchr(payload, ' ');
      if (sp) {
        *sp = '\0';
        ik_fx1 = atof(payload);
        ik_fy1 = atof(sp + 1);
      }
    } else if (type == '2') {
      char *sp = strchr(payload, ' ');
      if (sp) {
        *sp = '\0';
        ik_fx2 = atof(payload);
        ik_fy2 = atof(sp + 1);
      }
    } else if (type == 'D') {
      ik_dist = atof(payload);
    } else if (type == 'L') {
      ik_lean = atof(payload);
    }
    updateIK();
    return;
  }

  // ── Leg tab commands ─────────────────────────────────────────────────────
  if (cmd[0] == 'T' && cmd[1] == 'R' && cmd[2] == 'Q' && cmd[3] == 'E') {
    char *p = strchr(cmd, ' ');
    if (p) {
      int state = atoi(p + 1);
      for (int i = 0; i < 4; i++)
        ax12WriteByte(legServos[i].id, 24, state);
    }
    return;
  }

  if ((cmd[0] == 'P' && cmd[1] == 'O' && cmd[2] == 'S') ||
      (cmd[0] == 'T' && cmd[1] == 'R' && cmd[2] == 'Q') ||
      (cmd[0] == 'C' && cmd[1] == 'M' && cmd[2] == 'P')) {
    char *p1 = strchr(cmd, ' ');
    if (!p1)
      return;
    char *p2 = strchr(p1 + 1, ' ');
    if (!p2)
      return;
    *p1 = '\0';
    *p2 = '\0';
    int id = atoi(p1 + 1);

    if (cmd[0] == 'P') { // POS
      int val = atoi(p2 + 1);
      ax12WriteWord(id, 30, val);
      for (int i = 0; i < 4; i++)
        if (legServos[i].id == id)
          legServos[i].goalPos = val;
    } else if (cmd[0] == 'T') { // TRQ
      int val = atoi(p2 + 1);
      ax12WriteWord(id, 34, val);
      for (int i = 0; i < 4; i++)
        if (legServos[i].id == id)
          legServos[i].torqueLimit = val;
    } else { // CMP
      char *p3 = strchr(p2 + 1, ' ');
      if (!p3)
        return;
      *p3 = '\0';
      int margin = atoi(p2 + 1), slope = atoi(p3 + 1);
      ax12WriteByte(id, 26, margin);
      ax12WriteByte(id, 27, margin);
      ax12WriteByte(id, 28, slope);
      ax12WriteByte(id, 29, slope);
      for (int i = 0; i < 4; i++) {
        if (legServos[i].id == id) {
          legServos[i].compMargin = margin;
          legServos[i].compSlope = slope;
        }
      }
    }
    return;
  }

  // ── Position & Motion Target commands ────────────────────────────────────
  if (cmd[0] == 'P' && cmd[1] == 'O') { // PO <ticks> Position target offset
    target_pos_offset = atof(cmd + 2);
  } else if (cmd[0] == 'T' && cmd[1] == 'R') { // TR <bias> Turn bias
    turn_bias = atof(cmd + 2);
  } else if (cmd[0] == 'H' &&
             (cmd[1] == '\0' || cmd[1] == 'O')) { // Home position latch
    home_enc_pos = (float)(encoderLeft + encoderRight) * 0.5f;
    target_pos_offset = 0.0f;
    Serial3.print("ACK HOME LATCHED|");
    return;
  }
  // ── Cascaded loop gains ──────────────────────────────────────────────────
  else if (cmd[0] == 'V' && cmd[1] == 'P') {
    Kp_vel = atof(cmd + 2);
  } else if (cmd[0] == 'V' && cmd[1] == 'I') {
    Ki_vel = atof(cmd + 2);
  } else if (cmd[0] == 'V' && cmd[1] == 'A') {
    vel_alpha = constrain(atof(cmd + 2), 0.0f, 0.99f);
  } else if (cmd[0] == 'P' && cmd[1] == 'P') {
    Kp_pos = atof(cmd + 2);
  }
  // ── Balance PID & System commands ─────────────────────────────────────────
  else if (cmd[0] == 'P' && cmd[1] != '\0')
    Kp = atof(cmd + 1);
  else if (cmd[0] == 'I' && cmd[1] != '\0')
    Ki = atof(cmd + 1);
  else if (cmd[0] == 'D' && cmd[1] != '\0')
    Kd = atof(cmd + 1);
  else if (cmd[0] == 'A' && cmd[1] != '\0')
    alpha = atof(cmd + 1);
  else if (cmd[0] == 'T' && cmd[1] != '\0')
    maxSafeTilt = atof(cmd + 1);
  else if (cmd[0] == 'S' && cmd[1] != '\0')
    gui_base_angle = atof(cmd + 1);
  else if (cmd[0] == 'S' && cmd[1] == '\0') {
    initAX12Legs();
    Serial3.print("ACK SERVOS RESET|");
    return;
  } else if (cmd[0] == 'C') {
    calibrateIMU();
    return;
  } else if (cmd[0] == 'R') {
    integral = 0.0f;
    integral_vel = 0.0f;
    tilt_bias = 0.0f;
    Serial3.print("ACK INT RESET|");
    return;
  } else if (cmd[0] == 'M') {
    motorsEnabled = !motorsEnabled;
    if (motorsEnabled) {
      safetyLatched = false;
      integral = 0.0f;
      integral_vel = 0.0f;
      tilt_bias = 0.0f;
      home_enc_pos = (float)(encoderLeft + encoderRight) * 0.5f;
      target_pos_offset = 0.0f;
    }
    snprintf(ack, sizeof(ack), "Motors %s|",
             motorsEnabled ? "ENABLED" : "DISABLED");
    Serial3.print(ack);
    return;
  } else
    return;

  // Ack for tuning commands
  snprintf(ack, sizeof(ack),
           "Updated P%.3f I%.3f D%.3f Offset%.4f Target%.3f Alpha%.4f "
           "VP%.5f VI%.5f VA%.4f PP%.4f PO%.1f TR%.1f Tilt%.2f|",
           Kp, Ki, Kd, pitchOffset, gui_base_angle, alpha, Kp_vel, Ki_vel,
           vel_alpha, Kp_pos, target_pos_offset, turn_bias, maxSafeTilt);
  uint8_t len = (uint8_t)strlen(ack);
  if (Serial3.availableForWrite() >= len)
    Serial3.write((uint8_t *)ack, len);
}

// ── TELEMETRY RX ─────────────────────────────────────────────────────────────
static char rxBuf[80];
static uint8_t rxLen = 0;

void handleTelemetryRX() {
  int avail = Serial3.available();
  if (avail <= 0)
    return;

  int chunk = avail / 5;
  if (chunk < 1)
    chunk = 1;
  if (chunk > 20)
    chunk = 20;

  for (int i = 0; i < chunk; i++) {
    char c = (char)Serial3.read();
    if (c >= 'a' && c <= 'z')
      c -= 32;
    if (c == '|' || c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        parseCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < 79) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  delay(2000); // Allow AX-12 servos to power up

  Serial3.begin(115200);  // 3DR radio
  Serial2.begin(1000000); // AX-12 bus

  initAX12Legs();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  setupMPU();
  lastTime = micros();
  lastPollTime = millis();

  Serial3.print("BOOT OK|");
}

// ── MAIN LOOP (100 Hz) ───────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastTime < 10000)
    return; // enforce 100 Hz
  static unsigned long dt_us = 10000;
  dt_us = now - lastTime;
  float dt = dt_us * 1.0e-6f;
  lastTime = now;

  // ── IMU ──────────────────────────────────────────────────────────────────
  readIMU(dt);

  // ── SAFETY CUTOFF ─────────────────────────────────────────────────────────
  if (fabsf(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral = 0.0f;
    setMotors(0, 0);
    Serial3.print("SAFETY CUTOFF|");
  }

  // ── ENCODER SNAPSHOT ──────────────────────────────────────────────────────
  long encL = encoderLeft;
  long encR = encoderRight;

  // EL is RIGHT, ER is LEFT. Forward is + for EL, - for ER.
  float deltaEL_norm = (float)(encL - prevEncoderLeft);
  float deltaER_norm = -(float)(encR - prevEncoderRight);
  float vel_raw = ((deltaEL_norm + deltaER_norm) / 2.0f) / dt; // counts/sec
  vel_current = vel_alpha * vel_current + (1.0f - vel_alpha) * vel_raw;

  prevEncoderLeft = encL;
  prevEncoderRight = encR;

  // ── SMART AX-12 TORQUE CUTOFF ─────────────────────────────────────────────
  static bool prevMotorsEnabled = motorsEnabled;
  if (motorsEnabled != prevMotorsEnabled) {
    prevMotorsEnabled = motorsEnabled;
    uint8_t torqueVal = motorsEnabled ? 1 : 0;
    for (int i = 0; i < 4; i++) {
      ax12WriteByte(legServos[i].id, 24, torqueVal);
    }
  }

  // ══════════════════════════════════════════════════════════════════════════
  // CASCADED PID — SINGLE POSITION BALANCING
  // ══════════════════════════════════════════════════════════════════════════

  if (!motorsEnabled) {
    integral = 0.0f;
    integral_vel = 0.0f;
    tilt_bias = 0.0f;
    target_velocity = 0.0f;
    pos_error = 0.0f;
    vel_error = 0.0f;
    home_enc_pos = (float)(encL + encR) * 0.5f;
    setMotors(0, 0);
  } else {
    // ── LAYER 3 (OUTER): Target Position Loop ─────────────────────────────
    target_enc_pos = home_enc_pos + target_pos_offset;
    pos_error = target_enc_pos - (float)(encL + encR) * 0.5f;

    // Apply deadband to prevent micro-jitters
    if (fabsf(pos_error) < 4.0f) {
      pos_error = 0.0f;
    }

    target_velocity = constrain(Kp_pos * pos_error, -MAX_VEL_CMD, MAX_VEL_CMD);

    // ── LAYER 2 (MIDDLE): Velocity PID → Tilt Bias ─────────────────────────
    vel_error = target_velocity - vel_current;
    integral_vel += Ki_vel * vel_error * dt;
    integral_vel = constrain(integral_vel, -MAX_TILT_BIAS, MAX_TILT_BIAS);

    tilt_bias = (Kp_vel * vel_error) + integral_vel;
    tilt_bias = constrain(tilt_bias, -MAX_TILT_BIAS, MAX_TILT_BIAS);

    // ── LAYER 1 (INNER): Balance PID ────────────────────────────────────────
    float targetAngle = gui_base_angle + tilt_bias;
    float error = targetAngle - pitch;

    integral += error * dt;

    // Anti-windup for inner balance loop
    const float MAX_BALANCE_INTEGRAL = 2.0f;
    integral = constrain(integral, -MAX_BALANCE_INTEGRAL, MAX_BALANCE_INTEGRAL);

    float derivative = -gyroRate;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Steering / Turn Bias applied directly to motor outputs
    float left_pwm = -output + turn_bias;
    float right_pwm = -output - turn_bias;

    left_pwm = constrain(left_pwm, -255.0f, 255.0f);
    right_pwm = constrain(right_pwm, -255.0f, 255.0f);

    setMotors((int)left_pwm, (int)right_pwm);
  }

  // ── STRICT 50 Hz READ/WRITE TOGGLE ───────────────────────────────────────
  static bool isReadCycle = false;
  isReadCycle = !isReadCycle;

  if (isReadCycle) {
    pollLegServosTask();
  } else {
    handleTelemetryRX();
  }

  // ── TELEMETRY TX @ 20 Hz ──────────────────────────────────────────────────
  if (now - lastPrintTime >= 50000) {
    lastPrintTime = now;

    static uint32_t seq = 0;
    if (Serial3.availableForWrite() >= 100) {
      Serial3.print('S');
      Serial3.print(seq++);
      Serial3.print(" DT");
      Serial3.print(dt_us);
      Serial3.print(" P");
      Serial3.print(pitch, 2);
      Serial3.print(" O");
      Serial3.print(
          (motorsEnabled ? (Kp * (gui_base_angle + tilt_bias - pitch)) : 0.0f),
          2);
      Serial3.print(" I");
      Serial3.print(integral, 4);
      Serial3.print(" V");
      Serial3.print(vel_current, 1);
      Serial3.print(" TB");
      Serial3.print(tilt_bias, 3);
      Serial3.print(" EP");
      Serial3.print(pos_error, 1);
      Serial3.print(" EV");
      Serial3.print(vel_error, 1);
      Serial3.print(" VR");
      Serial3.print(vel_alpha, 3);
      Serial3.print(" PO");
      Serial3.print(target_pos_offset, 1);
      Serial3.print(" TR");
      Serial3.print(turn_bias, 1);
      Serial3.print(" A");
      Serial3.print(alpha, 2);
      Serial3.print(" T");
      Serial3.print(maxSafeTilt, 1);
      Serial3.print(" M");
      Serial3.print((int)motorsEnabled);
      Serial3.print(" L");
      Serial3.print((int)safetyLatched);
      Serial3.print('|');
    }
  }
}
