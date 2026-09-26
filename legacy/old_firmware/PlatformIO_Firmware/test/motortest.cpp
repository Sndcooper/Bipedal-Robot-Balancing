#include <Arduino.h>

// --- LEFT MOTOR (Motor 1) ---
#define ENA  PA0  // Speed (PWM)
#define IN1  PB12 // Direction 1
#define IN2  PB13 // Direction 2

// --- RIGHT MOTOR (Motor 2) ---
#define ENB  PA1  // Speed (PWM)
#define IN3  PB14 // Direction 1
#define IN4  PB15 // Direction 2

// Standard speed to use (0-255)
int motorSpeed = 150; 

// Track time to periodically print encoder feedback
unsigned long lastPrintTime = 0;

void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  
  // Make sure they start stopped
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setupEncoders() {
  // Set the 4 encoder pins as Input with Pullup
  pinMode(PA6, INPUT_PULLUP);
  pinMode(PA7, INPUT_PULLUP);
  pinMode(PB6, INPUT_PULLUP);
  pinMode(PB7, INPUT_PULLUP);

  // Enable clocks for Timer 3 (Left) and Timer 4 (Right)
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
  
  // --- Left Motor Encoder (Timer 3) ---
  TIM3->SMCR = 3;  // Encoder mode 3 (count on both TI1 and TI2)
  TIM3->CCER = 0;  // Rising edge polarity
  TIM3->CCMR1 = (1 << 8) | (1 << 0); // Map CC1/CC2 to TI1/TI2
  TIM3->ARR = 0xFFFF; // Auto-reload to max
  TIM3->CNT = 0;   // Initialize counter to 0
  TIM3->CR1 = 1;   // Enable counter

  // --- Right Motor Encoder (Timer 4) ---
  TIM4->SMCR = 3;  
  TIM4->CCER = 0;  
  TIM4->CCMR1 = (1 << 8) | (1 << 0); 
  TIM4->ARR = 0xFFFF; 
  TIM4->CNT = 0;   
  TIM4->CR1 = 1;   
}

void moveForward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  Serial1.println("Moving Forward");
}

void moveBackward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  Serial1.println("Moving Backward");
}

void turnLeft() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  Serial1.println("Turning Left");
}

void turnRight() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  Serial1.println("Turning Right");
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  Serial1.println("Motors Stopped");
}

void checkSerialCommands() {
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    
    // Ignore newline characters
    if (command == '\n' || command == '\r') return;

    command = toupper(command); 

    switch (command) {
      case 'W': moveForward();  break;
      case 'S': moveBackward(); break;
      case 'A': turnLeft();     break;
      case 'D': turnRight();    break;
      case 'X': stopMotors();   break;
      case '+': // Speed up
        motorSpeed += 25;
        if (motorSpeed > 255) motorSpeed = 255;
        analogWrite(ENA, motorSpeed);
        analogWrite(ENB, motorSpeed);
        Serial1.print("Speed increased to: "); Serial1.println(motorSpeed);
        break;
      case '-': // Slow down
        motorSpeed -= 25;
        if (motorSpeed < 0) motorSpeed = 0;
        analogWrite(ENA, motorSpeed);
        analogWrite(ENB, motorSpeed);
        Serial1.print("Speed decreased to: "); Serial1.println(motorSpeed);
        break;
      default:
        Serial1.println("Unknown Command. Use W/A/S/D to move, X to stop.");
        break;
    }
  }
}

void printEncoderFeedback() {
  // Read Timers as 16-bit signed ints to automatically handle polarity wrapping
  int16_t leftCount = TIM3->CNT;
  int16_t rightCount = TIM4->CNT;
  
  Serial1.print("Left Encoder: "); 
  Serial1.print(leftCount);
  Serial1.print("  | Right Encoder: ");
  Serial1.println(rightCount);
}

void setup() {
  Serial1.begin(250000); 
  delay(1000);
  
  setupEncoders();
  setupMotors();
  
  Serial1.println("Motors + Encoders Ready. Send W/A/S/D to move, X to stop.");
}

void loop() {
  checkSerialCommands();
  
  // Every 250 milliseconds, print the encoder numbers
  if (millis() - lastPrintTime >= 250) {
    printEncoderFeedback();
    lastPrintTime = millis();
  }
}
