#include <Arduino.h>

HardwareSerial Serial2(USART2); // AX-12+ Half-Duplex bus

// --- Servo Configuration ---
const uint8_t SERVO_IDS[4] = {6, 0, 14, 1}; 
const uint16_t STRAIGHT_DOWN[4] = {818, 818, 441, 441}; 

// --- Asynchronous State Machine Variables ---
bool isWaitingForServo = false;
unsigned long requestTime_us = 0;
uint8_t pollingIndex = 0;
uint8_t loopCounter = 0;

// Total expected bytes = 8 (Echo) + 56 (Response packet: 0xFF, 0xFF, ID, Len(52), Err, 50 Data bytes, Checksum)
const int EXPECTED_BYTES = 64; 

unsigned long lastTime = 0;

// Basic AX-12+ packet writers
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

// -------------------------------------------------------------
// 1. REQUEST ENTIRE CONTROL TABLE (Addresses 0 to 49, Length 50)
// -------------------------------------------------------------
void requestAllServoData(uint8_t id) {
  // Instruction: READ_DATA (0x02), Start Address: 0, Length: 50
  uint8_t checksum = ~(id + 4 + 2 + 0 + 50) & 0xFF; 
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x02, 0, 50, checksum};
  
  while(Serial2.available()) Serial2.read(); // Clear buffer
  
  Serial2.write(packet, 8);
  Serial2.flush();
  
  requestTime_us = micros();
  isWaitingForServo = true;
}

// -------------------------------------------------------------
// 2. HARVEST AND PARSE ALL DATA IN THE BACKGROUND
// -------------------------------------------------------------
void checkServoData() {
  if (!isWaitingForServo) return;

  if (Serial2.available() >= EXPECTED_BYTES) {
    unsigned long responseDelay = micros() - requestTime_us;
    
    // 1. Discard the 8-byte echo
    for (int i = 0; i < 8; i++) {
      Serial2.read();
    }
    
    // 2. Read the full 56-byte response packet
    uint8_t buf[56];
    for (int i = 0; i < 56; i++) {
      buf[i] = Serial2.read();
    }
    
    // Verify header and packet integrity
    if (buf[0] == 0xFF && buf[1] == 0xFF) {
      uint8_t id = buf[2];
      uint8_t error = buf[4];
      
      // Data payload starts at index 5 (Address 0 of the control table)
      // buf[5+Addr] maps directly to the AX-12+ control table address
      uint16_t modelNumber   = buf[5 + 0] | (buf[6] << 8);
      uint8_t  firmwareVer   = buf[5 + 2];
      uint8_t  servoId       = buf[5 + 3];
      uint16_t cwAngleLimit  = buf[5 + 6] | (buf[5 + 7] << 8);
      uint16_t ccwAngleLimit = buf[5 + 8] | (buf[5 + 9] << 8);
      uint8_t  maxTempLimit  = buf[5 + 11];
      uint8_t  minVoltage    = buf[5 + 12];
      uint8_t  maxVoltage    = buf[5 + 13];
      uint16_t maxTorque     = buf[5 + 14] | (buf[5 + 15] << 8);
      uint8_t  torqueEnabled = buf[5 + 24];
      uint8_t  ledState      = buf[5 + 25];
      uint16_t goalPos       = buf[5 + 30] | (buf[5 + 31] << 8);
      uint16_t movingSpeed   = buf[5 + 32] | (buf[5 + 33] << 8);
      uint16_t torqueLimit   = buf[5 + 34] | (buf[5 + 35] << 8);
      uint16_t presPosition  = buf[5 + 36] | (buf[5 + 37] << 8);
      uint16_t presSpeed     = buf[5 + 38] | (buf[5 + 39] << 8);
      uint16_t presLoad      = buf[5 + 40] | (buf[5 + 41] << 8);
      uint8_t  presVoltage   = buf[5 + 42];
      uint8_t  presTemp      = buf[5 + 43];
      uint8_t  isMoving      = buf[5 + 46];

      // Print parsed telemetry to Serial monitor
      Serial1.print("ID:"); Serial1.print(servoId);
      if (error > 0) Serial1.print(" [ERR!]");
      Serial1.print(" | Pos:"); Serial1.print(presPosition);
      Serial1.print(" | Goal:"); Serial1.print(goalPos);
      Serial1.print(" | Speed:"); Serial1.print(presSpeed);
      Serial1.print(" | Load:"); Serial1.print(presLoad);
      Serial1.print(" | TLimit:"); Serial1.print(torqueLimit);
      Serial1.print(" | Volt:"); Serial1.print(presVoltage * 0.1); Serial1.print("V");
      Serial1.print(" | Temp:"); Serial1.print(presTemp); Serial1.print("C");
      Serial1.print(" | Moving:"); Serial1.print(isMoving);
      Serial1.print(" | Latency:"); Serial1.print(responseDelay);
      Serial1.println("us");
    }
    
    isWaitingForServo = false;
  } 
  else if (micros() - requestTime_us > 5000) { // 5ms timeout for larger packet size
    while(Serial2.available()) Serial2.read();
    Serial1.println("Servo Read Timeout - Packet Dropped");
    isWaitingForServo = false;
  }
}

void setup() {
  Serial1.begin(115200);
  Serial2.begin(1000000);
  delay(2000); 

  Serial1.println("Setting Servos to Erected Pose...");
  for (int i = 0; i < 4; i++) {
    uint8_t id = SERVO_IDS[i];
    ax12WriteWord(id, 32, 100);              
    ax12WriteByte(id, 24, 1);                
    ax12WriteWord(id, 30, STRAIGHT_DOWN[i]); 
  }
  
  delay(1000); 
  Serial1.println("System Ready. Dumping Full Control Table Asynchronously.");
}

void loop() {
  unsigned long now = micros();

  // Free Time Zone: Continuously process incoming serial data
  checkServoData();

  // 100Hz Timing Gate
  if (now - lastTime < 10000) return;
  lastTime = now;

  // Staggered polling: Read one servo every 5 loops (50ms interval per servo)
  loopCounter++;
  if (loopCounter >= 5) {
    loopCounter = 0;
    
    if (!isWaitingForServo) {
      requestAllServoData(SERVO_IDS[pollingIndex]);
      
      pollingIndex++;
      if (pollingIndex >= 4) pollingIndex = 0;
    }
  }
}