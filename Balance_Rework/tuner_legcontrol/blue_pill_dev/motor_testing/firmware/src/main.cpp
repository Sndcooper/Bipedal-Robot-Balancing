// blue_pill_dev/motor_testing firmware — STM32 Bluepill (F103C8)
// Wiring sanity checker: drive L/R motors from GUI commands, stream live
// encoder counts. Flash once, then do all direction/wiring verification from
// the GUI. FTDI wired link on Serial3 (PB10 TX / PB11 RX) @ 115200 — matches
// this bench's existing FTDI wiring (previously used for 3DR/Serial3 telemetry).

#include <Arduino.h>

// ── ENCODER PINS ─────────────────────────────────────────────────────────────
#define ENC_L_A PA6
#define ENC_L_B PA7
#define ENC_R_A PB0
#define ENC_R_B PB1

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

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

extern HardwareSerial Serial3;

int testPWM = 150; // fixed test speed, GUI can override with SP<val>

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
// F | B | L | R | S  — forward/back/turn-left/turn-right/stop
// SP<val>            — set test PWM magnitude (0-255)
void parseCommand(char *cmd) {
  switch (cmd[0]) {
  case 'F':
    setMotors(testPWM, testPWM);
    break;
  case 'B':
    setMotors(-testPWM, -testPWM);
    break;
  case 'L': // spin left in place: left wheel back, right wheel forward
             // (matches turn_bias convention in mcu_pos_wireless: left=-bias, right=+bias)
    setMotors(-testPWM, testPWM);
    break;
  case 'R': // spin right in place: left wheel forward, right wheel back
             // (matches turn_bias convention in mcu_pos_wireless: left=+bias, right=-bias)
    setMotors(testPWM, -testPWM);
    break;
  case 'S':
    if (cmd[1] == 'P') {
      testPWM = constrain(atoi(cmd + 2), 0, 255);
    } else {
      setMotors(0, 0);
    }
    break;
  default:
    break;
  }
}

// ── COMMAND RX ───────────────────────────────────────────────────────────────
static char rxBuf[32];
static uint8_t rxLen = 0;

void handleCommandRX() {
  while (Serial3.available()) {
    char c = (char)Serial3.read();
    if (c == '|' || c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        parseCommand(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ── SETUP ────────────────────────────────────────────────────────────────────
unsigned long lastPrintTime = 0;

void setup() {
  analogWriteResolution(8);

  Serial3.begin(115200);

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

  setMotors(0, 0);
  Serial3.print("BOOT OK|");
}

// ── MAIN LOOP ────────────────────────────────────────────────────────────────
void loop() {
  handleCommandRX();

  unsigned long now = millis();
  if (now - lastPrintTime >= 100) { // 10 Hz telemetry
    lastPrintTime = now;
    long el = encoderLeft;
    long er = encoderRight;
    char buf[48];
    snprintf(buf, sizeof(buf), "EL%ld ER%ld|", el, er);
    Serial3.print(buf);
  }
}
