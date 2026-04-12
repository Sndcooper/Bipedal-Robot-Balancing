#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial Serial2(USART2);  // PA2=TX, PA3=RX → AX-12+

void sendPacket(byte id, byte address, byte value);
void sendPacketWord(byte id, byte address, uint16_t value);
void drainEcho(int bytesSent);
void readResponse();

void setup() {
  Serial1.begin(115200);          // PC Communication
  Serial2.begin(1000000);         // AX-12+ via PA2/PA3
  Serial2.setTimeout(50);
  Serial1.setTimeout(100);

  delay(2000);
  Serial1.println("STM32 AX-12+ PC Controller Ready...");
}

void loop() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      // Format expected: "id,position"
      int commaIdx = cmd.indexOf(',');
      if (commaIdx > 0) {
        int id = cmd.substring(0, commaIdx).toInt();
        int position = cmd.substring(commaIdx + 1).toInt();
        
        Serial1.print("Moving ID ");
        Serial1.print(id);
        Serial1.print(" to position ");
        Serial1.println(position);
        
        sendPacketWord(id, 30, position); // Goal Position address is 30
      }
    }
  }
}

void sendPacket(byte id, byte address, byte value) {
  byte packet[8];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x04;
  packet[4] = 0x03;
  packet[5] = address;
  packet[6] = value;

  byte checksum = 0;
  for (int i = 2; i <= 6; i++) checksum += packet[i];
  packet[7] = (~checksum) & 0xFF;

  Serial1.print("TX: ");
  for (int i = 0; i < 8; i++) {
    Serial1.print("0x"); Serial1.print(packet[i], HEX); Serial1.print(" ");
  }
  Serial1.println();

  Serial2.write(packet, 8);
  Serial2.flush();
  drainEcho(8);
  readResponse();
}

void sendPacketWord(byte id, byte address, uint16_t value) {
  byte lo = value & 0xFF;
  byte hi = (value >> 8) & 0xFF;

  byte packet[9];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x05;
  packet[4] = 0x03;
  packet[5] = address;
  packet[6] = lo;
  packet[7] = hi;

  byte checksum = 0;
  for (int i = 2; i <= 7; i++) checksum += packet[i];
  packet[8] = (~checksum) & 0xFF;

  Serial1.print("TX: ");
  for (int i = 0; i < 9; i++) {
    Serial1.print("0x"); Serial1.print(packet[i], HEX); Serial1.print(" ");
  }
  Serial1.println();

  Serial2.write(packet, 9);
  Serial2.flush();
  drainEcho(9);
  readResponse();
}

void drainEcho(int bytesSent) {
  unsigned long t = millis();
  int count = 0;
  while (count < bytesSent && millis() - t < 20) {
    if (Serial2.available()) { Serial2.read(); count++; }
  }
}

void readResponse() {
  byte response[6];
  int n = Serial2.readBytes(response, 6);

  if (n < 6) {
    Serial1.println("No / incomplete response");
    return;
  }

  Serial1.print("RX: ");
  for (int i = 0; i < 6; i++) {
    Serial1.print("0x"); Serial1.print(response[i], HEX); Serial1.print(" ");
  }

  byte errByte = response[4];
  if (errByte == 0) Serial1.println("-> OK");
  else { Serial1.print("-> ERROR: 0b"); Serial1.println(errByte, BIN); }
}