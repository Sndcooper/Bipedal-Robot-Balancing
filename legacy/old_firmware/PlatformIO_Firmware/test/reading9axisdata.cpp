#include <Arduino.h>

// Initialize HardwareSerial for Serial3 (USART3) 
// RX = PB11, TX = PB10
HardwareSerial Serial3(USART3); // Alternatively: HardwareSerial Serial3(PB11, PB10);

// Must match the exact data structure sent by the Nano!
struct SensorData {
  float ax, ay, az; // Accelerometer
  float gx, gy, gz; // Gyroscope
  float mx, my, mz; // Magnetometer
};

SensorData receivedData;

// State machine states for reading the packet
enum ReadState {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  READ_PAYLOAD
};

ReadState currentState = WAIT_HEADER_1;
uint8_t payloadBuffer[sizeof(SensorData)];
int bytesRead = 0;

void setup() {
  // Serial1: Connects to the Serial Monitor
  Serial1.begin(250000); 

  // Serial3: Connects to Nano TX1 -> STM32 RX3
  Serial3.begin(250000);
  
  // Wait a moment for serial to initialize
  delay(1000);

  // Flush any stale/old data in the buffer before we begin reading
  while(Serial3.available()) {
    Serial3.read();
  }

  Serial1.println("STM32 Ready. Waiting for IMU data on Serial3...");
}

void loop() {
  // Process all available bytes in the Serial3 receive buffer
  while (Serial3.available() > 0) {
    byte incomingByte = Serial3.read();

    switch (currentState) {
      
      case WAIT_HEADER_1:
        if (incomingByte == 0xAA) {
          currentState = WAIT_HEADER_2;
        }
        break;

      case WAIT_HEADER_2:
        if (incomingByte == 0xBB) {
          // Header verified! Next bytes will be our struct data
          currentState = READ_PAYLOAD;
          bytesRead = 0; 
        } else if (incomingByte == 0xAA) {
          // If we got another 0xAA, we might still be overlapping with a valid header
          currentState = WAIT_HEADER_2;
        } else {
          // Invalid header byte, go back to hunting for 0xAA
          currentState = WAIT_HEADER_1;
        }
        break;

      case READ_PAYLOAD:
        // Store byte in our struct buffer
        payloadBuffer[bytesRead] = incomingByte;
        bytesRead++;

        // Once we have reached the exact size of the struct (36 bytes)
        if (bytesRead >= sizeof(SensorData)) {
          
          // Copy the raw bytes directly into the data structure
          memcpy(&receivedData, payloadBuffer, sizeof(SensorData));

          // --- Print the parsed data to the Serial Monitor (Serial1) ---
          Serial1.print("Accel [X: "); Serial1.print(receivedData.ax);
          Serial1.print(", Y: ");      Serial1.print(receivedData.ay);
          Serial1.print(", Z: ");      Serial1.print(receivedData.az);
          
          Serial1.print("]  Gyro [X: "); Serial1.print(receivedData.gx);
          Serial1.print(", Y: ");        Serial1.print(receivedData.gy);
          Serial1.print(", Z: ");        Serial1.print(receivedData.gz);
          
          Serial1.print("]  Mag [X: ");  Serial1.print(receivedData.mx);
          Serial1.print(", Y: ");        Serial1.print(receivedData.my);
          Serial1.print(", Z: ");        Serial1.print(receivedData.mz);
          Serial1.println("]");
          
          // Reset the state machine to wait for the next packet
          currentState = WAIT_HEADER_1;
        }
        break;
    }
  }
}