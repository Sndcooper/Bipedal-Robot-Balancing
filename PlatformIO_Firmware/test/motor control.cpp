#include <Arduino.h>

// --- PC Communication (via USB-TTL on PA9/PA10) ---
// If you are using native USB, use Serial instead of Serial1.
// Based on digitaltwin.cpp, you usually use Serial1 for the PC.

// --- LEFT DC MOTOR (Motor 1) ---
// Physical swap applied: left motor is wired on old right channel pins.
#define ENA PA1  // Speed (PWM)
#define IN1 PB14 // Direction 1
#define IN2 PB15 // Direction 2

// --- RIGHT DC MOTOR (Motor 2) ---
// Physical swap applied: right motor is wired on old left channel pins.
#define ENB PA0  // Speed (PWM)
#define IN3 PB12 // Direction 1
#define IN4 PB13 // Direction 2

int defaultSpeed = 150; // 0 to 255

void setup() {
  // Initialize communication with PC
  Serial1.begin(115200);
  
  // Set all motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ensure motors start stopped
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(1000);
  Serial1.println("=====================================");
  Serial1.println(" DC MOTOR DIAGNOSTIC TEST ");
  Serial1.println("=====================================");
  Serial1.println("Send commands to Serial1 (115200 baud)");
  Serial1.println(" 'W' or 'w' : FORWARD");
  Serial1.println(" 'S' or 's' : BACKWARD");
  Serial1.println(" 'A' or 'a' : TURN LEFT");
  Serial1.println(" 'D' or 'd' : TURN RIGHT");
  Serial1.println(" 'Q' or 'q' : TEST LEFT MOTOR ONLY (Forward)");
  Serial1.println(" 'E' or 'e' : TEST RIGHT MOTOR ONLY (Forward)");
  Serial1.println(" SPACE or 'X': STOP ALL");
  Serial1.println("=====================================");
}

// Function to control both motors
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor (Motor 1)
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }
  
  // Right Motor (Motor 2)
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    
    // Ignore newline characters
    if (c == '\n' || c == '\r') return;

    switch (c) {
      case 'W': case 'w':
        Serial1.println(">> FORWARD (Both Motors)");
        setMotors(defaultSpeed, defaultSpeed);
        break;
        
      case 'S': case 's':
        Serial1.println(">> BACKWARD (Both Motors)");
        setMotors(-defaultSpeed, -defaultSpeed);
        break;
        
      case 'A': case 'a':
        Serial1.println(">> TURN LEFT (Left BWD, Right FWD)");
        setMotors(-defaultSpeed, defaultSpeed);
        break;
        
      case 'D': case 'd':
        Serial1.println(">> TURN RIGHT (Left FWD, Right BWD)");
        setMotors(defaultSpeed, -defaultSpeed);
        break;

      case 'Q': case 'q':
        Serial1.println(">> TEST: LEFT MOTOR ONLY");
        setMotors(defaultSpeed, 0);
        break;

      case 'E': case 'e':
        Serial1.println(">> TEST: RIGHT MOTOR ONLY");
        setMotors(0, defaultSpeed);
        break;

      case 'X': case 'x': case ' ':
        Serial1.println(">> STOP ALL");
        setMotors(0, 0);
        break;
        
      default:
        Serial1.println("Unknown command. Use W, A, S, D, Q, E, or X.");
        break;
    }
  }
}
