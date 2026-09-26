#include <Arduino.h>
#include <HardwareSerial.h>

// ============================================================================
//  AX-12+ LEG HOMING UTILITY  —  "straight down" for leg installation
// ----------------------------------------------------------------------------
//  Drives the 4 leg servos to the digital-twin "straight down" reference so you
//  can physically install the legs aligned, then holds them there (torque ON).
//
//  Straight-down = the calibrated standing pose set by hand on hardware:
//  LEFT servos (6,0) = 818, RIGHT servos (14,1) = 441.
//  (The twin's theoretical value was 800/430; +18/+11 is the real servo-horn offset.)
//
//  Serial1 (PA9/PA10) @ 115200 = PC.   Serial2 (USART2, PA2/PA3) @ 1Mbps = AX-12+.
//  This is a standalone project; it does NOT touch the balance firmware. Re-flash
//  Balance_Rework/firmware once the legs are on.
// ============================================================================

HardwareSerial Serial2(USART2);  // PA2=TX, PA3=RX -> AX-12+ (half-duplex)

// --- Straight-down reference (AX-12 units, 0..1023 over 300 deg) ---
#define POS_LEFT_DOWN   818
#define POS_RIGHT_DOWN  441

// --- Servo IDs (from the digital twin & main.cpp initAX12Legs) ---
//   "Left"  servos: Leg1-L = 6, Leg2-L = 0   -> home to POS_LEFT_DOWN
//   "Right" servos: Leg1-R = 14, Leg2-R = 1  -> home to POS_RIGHT_DOWN
const uint8_t LEFT_IDS[]  = {6, 0};
const uint8_t RIGHT_IDS[] = {14, 1};

// Gentle move speed so nothing snaps during setup. NOTE: 0 = "max speed" on the
// AX-12 in joint mode, so never use 0 here.
#define MOVE_SPEED  80

// --- AX-12 control-table addresses ---
#define AX_TORQUE_ENABLE 24
#define AX_GOAL_POSITION 30
#define AX_MOVING_SPEED  32

// --- Low-level writers (same proven packet format as the main firmware) ---
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

void moveServo(uint8_t id, uint16_t pos) {
  ax12WriteWord(id, AX_MOVING_SPEED, MOVE_SPEED);
  ax12WriteByte(id, AX_TORQUE_ENABLE, 1);
  ax12WriteWord(id, AX_GOAL_POSITION, pos);
}

void homeAll() {
  Serial1.println("Homing all 4 servos to STRAIGHT DOWN...");
  for (uint8_t id : LEFT_IDS) {
    moveServo(id, POS_LEFT_DOWN);
    Serial1.print("  ID "); Serial1.print(id);
    Serial1.print(" (LEFT)  -> "); Serial1.println(POS_LEFT_DOWN);
  }
  for (uint8_t id : RIGHT_IDS) {
    moveServo(id, POS_RIGHT_DOWN);
    Serial1.print("  ID "); Serial1.print(id);
    Serial1.print(" (RIGHT) -> "); Serial1.println(POS_RIGHT_DOWN);
  }
  Serial1.println("Servos are holding straight-down. Install the legs now.");
}

void printHelp() {
  Serial1.println("Commands:");
  Serial1.println("  H          re-home all 4 servos to straight down");
  Serial1.println("  <id>,<pos> move one servo (pos 0..1023) e.g.  6,810");
  Serial1.println("  F<id>      free one servo (torque OFF) e.g.   F6");
  Serial1.println("  L<id>      lock one servo (torque ON)  e.g.   L6");
}

void setup() {
  Serial1.begin(115200);
  Serial2.begin(1000000);
  Serial2.setTimeout(50);
  delay(1500);  // let the servos finish powering up before we command them

  Serial1.println("=== AX-12 LEG HOMING (straight down) ===");
  Serial1.print("  LEFT servos (6, 0)  -> "); Serial1.println(POS_LEFT_DOWN);
  Serial1.print("  RIGHT servos (14, 1) -> "); Serial1.println(POS_RIGHT_DOWN);
  homeAll();
  printHelp();
}

void loop() {
  if (!Serial1.available()) return;

  String cmd = Serial1.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = toupper(cmd.charAt(0));

  if (c == 'H') {
    homeAll();
    return;
  }

  if (c == 'F' || c == 'L') {  // torque OFF / ON for a single servo
    int id = cmd.substring(1).toInt();
    ax12WriteByte(id, AX_TORQUE_ENABLE, (c == 'L') ? 1 : 0);
    Serial1.print("ID "); Serial1.print(id);
    Serial1.println((c == 'L') ? " LOCKED (torque ON)" : " FREED (torque OFF)");
    return;
  }

  int comma = cmd.indexOf(',');   // "id,pos"
  if (comma > 0) {
    int id = cmd.substring(0, comma).toInt();
    int pos = cmd.substring(comma + 1).toInt();
    pos = constrain(pos, 0, 1023);
    moveServo((uint8_t)id, (uint16_t)pos);
    Serial1.print("ID "); Serial1.print(id);
    Serial1.print(" -> "); Serial1.println(pos);
    return;
  }

  Serial1.println("?? unrecognized. Type H to re-home.");
  printHelp();
}
