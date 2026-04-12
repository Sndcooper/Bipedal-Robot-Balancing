
#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial Serial2(USART2);

// ── Modes ──
enum Mode { MODE_TEXT, MODE_RAW };
Mode currentMode = MODE_RAW;  // Start in RAW mode for Python controller

// ── Forward declarations ──
void sendDynamixelPacketByte(byte id, byte address, byte value);
void sendDynamixelPacketWord(byte id, byte address, uint16_t value);
void sendPing(byte id);
void drainEcho(int bytesSent);
void readResponse();
void processTextCommand(String &cmd);
void handleRawPassthrough();

void setup() {
    Serial1.begin(115200);
    Serial2.begin(1000000);
    Serial2.setTimeout(50);

    delay(1000);
    Serial1.println("STM32 AX-12+ Bridge Ready");
    Serial1.println("Mode: RAW (send Dynamixel packets directly)");
    Serial1.println("Send 'TEXT\\n' to switch to text command mode");
}

void loop() {
    if (currentMode == MODE_RAW) {
        handleRawPassthrough();
    } else {
        // TEXT mode — read line commands
        if (Serial1.available()) {
            String cmd = Serial1.readStringUntil('\n');
            cmd.trim();
            if (cmd == "RAW") {
                currentMode = MODE_RAW;
                Serial1.println("Switched to RAW mode");
            } else {
                processTextCommand(cmd);
            }
        }
    }
}

// ════════════════════════════════════════
//  RAW PASSTHROUGH MODE
// ════════════════════════════════════════
enum PassthroughState {
    WAIT_HEADER1,
    WAIT_HEADER2,
    WAIT_ID,
    WAIT_LEN,
    WAIT_PAYLOAD
};

void handleRawPassthrough() {
    static PassthroughState state = WAIT_HEADER1;
    static byte packet[256];
    static int payloadCount = 0;
    static byte expectedLen = 0;

    // PC → AX-12+: Parse bytes one by one non-blocking
    while (Serial1.available()) {
        byte b = Serial1.read();

        switch (state) {
            case WAIT_HEADER1:
                if (b == 0xFF) {
                    packet[0] = 0xFF;
                    state = WAIT_HEADER2;
                } else if (b == 'T') {
                    // Check if it's "TEXT\n" command fallback
                    unsigned long t = millis();
                    String rest = String((char)b);
                    while(millis() - t < 100) {
                        if (Serial1.available()) {
                            char c = Serial1.read();
                            rest += c;
                            if (c == '\n') break;
                        }
                    }
                    rest.trim();
                    if (rest == "TEXT") {
                        currentMode = MODE_TEXT;
                        Serial1.println("Switched to TEXT mode");
                        state = WAIT_HEADER1; // reset for next time
                        return;
                    }
                }
                break;

            case WAIT_HEADER2:
                if (b == 0xFF) {
                    packet[1] = 0xFF;
                    state = WAIT_ID;
                } else {
                    state = WAIT_HEADER1; // reset
                }
                break;

            case WAIT_ID:
                packet[2] = b;
                state = WAIT_LEN;
                break;

            case WAIT_LEN:
                packet[3] = b;
                expectedLen = b; // Length includes parameters + checksum
                payloadCount = 0;
                if (expectedLen > 0) {
                    state = WAIT_PAYLOAD;
                } else {
                    state = WAIT_HEADER1; // safety fallback
                }
                break;

            case WAIT_PAYLOAD:
                packet[4 + payloadCount] = b;
                payloadCount++;

                if (payloadCount >= expectedLen) {
                    int totalLen = 4 + expectedLen;
                    byte id = packet[2];

                    // Forward entire packet to AX-12+
                    Serial2.write(packet, totalLen);
                    Serial2.flush();

                    // Drain echo (half-duplex) - wait a bit for echo to arrive
                    delay(5);
                    drainEcho(totalLen);

                    // Read AX-12+ response and forward to PC
                    // For broadcast (ID=0xFE) or SYNC_WRITE, no response expected
                    if (id != 0xFE) {
                        // Wait a bit for servo to process and respond
                        delay(10);
                        
                        // Wait for response
                        unsigned long timeout = millis() + 100;
                        byte resp[64];
                        int respLen = 0;

                        // Look for response start (0xFF 0xFF)
                        bool foundHeader = false;
                        while (millis() < timeout && !foundHeader) {
                            if (Serial2.available()) {
                                byte rb = Serial2.read();
                                resp[respLen++] = rb;
                                
                                // Check for 0xFF 0xFF header
                                if (respLen >= 2 && resp[respLen-2] == 0xFF && resp[respLen-1] == 0xFF) {
                                    foundHeader = true;
                                }
                                
                                // Prevent overflow
                                if (respLen >= 64) break;
                            }
                        }

                        // Read rest of response: ID, Length, then payload
                        if (foundHeader && respLen >= 2) {
                            // Read ID
                            while (!Serial2.available() && millis() < timeout) delay(1);
                            if (Serial2.available()) {
                                resp[respLen++] = Serial2.read(); // ID
                                
                                // Read Length
                                while (!Serial2.available() && millis() < timeout) delay(1);
                                if (Serial2.available()) {
                                    byte rLen = Serial2.read();
                                    resp[respLen++] = rLen; // Length
                                    
                                    // Read payload (rLen bytes: error + params + checksum)
                                    for (int i = 0; i < rLen && respLen < 64; i++) {
                                        while (!Serial2.available() && millis() < timeout) delay(1);
                                        if (Serial2.available()) {
                                            resp[respLen++] = Serial2.read();
                                        }
                                    }
                                }
                            }
                        }

                        // Forward response to PC
                        if (respLen > 0) {
                            Serial1.write(resp, respLen);
                            Serial1.flush();
                        }
                    }
                    
                    // Packet handled, reset state for the next incoming Dynamixel packet
                    state = WAIT_HEADER1;
                }
                break;
        }
    }
}

// ════════════════════════════════════════
//  TEXT COMMAND PROCESSOR
// ════════════════════════════════════════
void processTextCommand(String &cmd) {
    // Parse comma-separated command
    int firstComma = cmd.indexOf(',');
    if (firstComma < 0) {
        Serial1.println("ERR: Invalid command format");
        return;
    }

    String type = cmd.substring(0, firstComma);
    String rest = cmd.substring(firstComma + 1);

    if (type == "PNG") {
        byte id = rest.toInt();
        sendPing(id);
    }
    else if (type == "POS" || type == "SPD" || type == "TRQ") {
        int comma2 = rest.indexOf(',');
        byte id = rest.substring(0, comma2).toInt();
        uint16_t val = rest.substring(comma2 + 1).toInt();

        byte addr = 30; // POS
        if (type == "SPD") addr = 32;
        else if (type == "TRQ") addr = 34;

        Serial1.print(type); Serial1.print(" ID="); Serial1.print(id);
        Serial1.print(" VAL="); Serial1.println(val);
        sendDynamixelPacketWord(id, addr, val);
    }
    else if (type == "TEN" || type == "LED") {
        int comma2 = rest.indexOf(',');
        byte id = rest.substring(0, comma2).toInt();
        byte val = rest.substring(comma2 + 1).toInt();

        byte addr = (type == "TEN") ? 24 : 25;
        Serial1.print(type); Serial1.print(" ID="); Serial1.print(id);
        Serial1.print(" VAL="); Serial1.println(val);
        sendDynamixelPacketByte(id, addr, val);
    }
    else if (type == "CMG" || type == "CSL") {
        // CMG,ID,CW,CCW or CSL,ID,CW,CCW
        int c1 = rest.indexOf(',');
        int c2 = rest.indexOf(',', c1 + 1);
        byte id = rest.substring(0, c1).toInt();
        byte cw = rest.substring(c1 + 1, c2).toInt();
        byte ccw = rest.substring(c2 + 1).toInt();

        byte addr = (type == "CMG") ? 26 : 28;
        Serial1.print(type); Serial1.print(" ID="); Serial1.print(id);
        Serial1.print(" CW="); Serial1.print(cw);
        Serial1.print(" CCW="); Serial1.println(ccw);
        sendDynamixelPacketByte(id, addr, cw);
        sendDynamixelPacketByte(id, addr + 1, ccw);
    }
    else {
        Serial1.println("ERR: Unknown command type: " + type);
    }
}

// ════════════════════════════════════════
//  DYNAMIXEL PACKET SENDERS
// ════════════════════════════════════════
void sendDynamixelPacketByte(byte id, byte address, byte value) {
    byte packet[8];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x04;  // length = 4 (inst + addr + val + chk)
    packet[4] = 0x03;  // WRITE
    packet[5] = address;
    packet[6] = value;

    byte checksum = 0;
    for (int i = 2; i <= 6; i++) checksum += packet[i];
    packet[7] = (~checksum) & 0xFF;

    Serial2.write(packet, 8);
    Serial2.flush();
    drainEcho(8);
    readResponse();
}

void sendDynamixelPacketWord(byte id, byte address, uint16_t value) {
    byte lo = value & 0xFF;
    byte hi = (value >> 8) & 0xFF;

    byte packet[9];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x05;
    packet[4] = 0x03;  // WRITE
    packet[5] = address;
    packet[6] = lo;
    packet[7] = hi;

    byte checksum = 0;
    for (int i = 2; i <= 7; i++) checksum += packet[i];
    packet[8] = (~checksum) & 0xFF;

    Serial2.write(packet, 9);
    Serial2.flush();
    drainEcho(9);
    readResponse();
}

void sendPing(byte id) {
    byte packet[6];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x02;
    packet[4] = 0x01;  // PING

    byte checksum = 0;
    for (int i = 2; i <= 4; i++) checksum += packet[i];
    packet[5] = (~checksum) & 0xFF;

    Serial1.print("PING ID="); Serial1.println(id);
    Serial2.write(packet, 6);
    Serial2.flush();
    drainEcho(6);
    readResponse();
}

void drainEcho(int bytesSent) {
    unsigned long t = millis();
    int count = 0;
    while (count < bytesSent && millis() - t < 20) {
        if (Serial2.available()) {
            Serial2.read();
            count++;
        }
    }
}

void readResponse() {
    byte response[32];
    int n = 0;
    unsigned long timeout = millis() + 100;

    // Wait for header
    while (millis() < timeout && n < 2) {
        if (Serial2.available()) {
            response[n++] = Serial2.read();
        }
    }

    if (n >= 2) {
        // Read ID and Length
        while (!Serial2.available() && millis() < timeout) delay(1);
        if (Serial2.available()) response[n++] = Serial2.read(); // ID
        while (!Serial2.available() && millis() < timeout) delay(1);
        if (Serial2.available()) {
            byte len = Serial2.read();
            response[n++] = len; // Length
            for (int i = 0; i < len && n < 32; i++) {
                while (!Serial2.available() && millis() < timeout) delay(1);
                if (Serial2.available()) response[n++] = Serial2.read();
            }
        }
    }

    // Print response in TEXT mode
    if (currentMode == MODE_TEXT && n > 0) {
        Serial1.print("RX: ");
        for (int i = 0; i < n; i++) {
            Serial1.print("0x");
            Serial1.print(response[i], HEX);
            Serial1.print(" ");
        }
        if (n >= 5) {
            byte errByte = response[4];
            if (errByte == 0) Serial1.println("-> OK");
            else {
                Serial1.print("-> ERROR: 0b");
                Serial1.println(errByte, BIN);
            }
        } else {
            Serial1.println("-> Incomplete");
        }
    }
}
