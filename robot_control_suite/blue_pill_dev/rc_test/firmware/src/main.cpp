// ============================================================================
// rc_test — MINIMAL receiver-only sketch. Reads whatever is arriving on
// USART1 (PA10 RX) and prints raw bytes + a hand-rolled iBUS frame check on
// USART3 (PB10/PB11, the same 3DR radio link bfrc uses -> COM13 on this PC).
//
// Purpose: isolate the RC question from everything else in bfrc (balance PID,
// AX-12 bus, 34-field telemetry line). If this decodes valid iBUS frames, the
// receiver, wiring and iBUS config are all proven good and any remaining
// problem is in bfrc's control code. If it does not, the fault is upstream of
// any firmware logic entirely - wiring, receiver mode, baud, or power.
//
// RESULT SO FAR (first run, IBusBM-based version): RXB climbed steadily at
// ~100 B/s, FR stayed 0 forever. Bytes ARE reaching PA10, but not one had ever
// formed a valid iBUS frame - and the timing bug that caused this exact
// symptom in bfrc (polling ibus.loop() too slowly) was already fixed in that
// version, so the cause is upstream of software: wrong baud, wrong protocol
// (SBUS/PPM instead of iBUS), or noise.
//
// WHY THIS VERSION DOES NOT USE THE IBusBM LIBRARY:
// IBusBM::loop() owns Serial1 internally (calls stream->read() itself) and
// does not expose the bytes it consumes. Capturing the raw wire content
// alongside it needs one of:
//   (a) peek() before it reads - but peek() only ever returns the OLDEST
//       buffered byte, and ibus.loop() drains the WHOLE buffer per call in a
//       while(available()) loop, so a peek from this loop() undercounts and
//       desyncs the moment more than one byte is waiting per pass;
//   (b) draining Serial1 into our own buffer first - but then ibus.loop()
//       gets an EMPTY stream and never decodes anything, breaking the very
//       frame-validity check this sketch exists to show.
// Both were tried and are wrong for exactly these reasons. The fix is to stop
// sharing ownership of Serial1 with a library that was not designed to be
// observed: this sketch reads every byte itself, ONCE, and runs a small
// hand-written iBUS frame-sync state machine inline. It implements exactly
// what IBusBM.cpp's GET_LENGTH/GET_DATA/GET_CHKSUM* states do (same protocol
// constants: length byte in (3..0x20], command byte 0x40, 16-bit checksum =
// 0xFFFF - sum of all prior bytes), so a real iBUS frame decodes exactly as it
// would in bfrc, and the raw bytes are visible with nothing hidden between the
// wire and this program.
//
//   iBUS frame starts   0x20 0x40 ...      (length=0x20, command=0x40)
//   SBUS frame starts   0x0F ...           (distinct start byte, every ~25 bytes)
//   Noise / dead line    no repeating pattern, or a constant idle byte (e.g. all 0x00 or 0xFF)
// ============================================================================

#include <Arduino.h>

extern HardwareSerial Serial1;   // receiver signal in (PA10 RX) - INPUT ONLY
extern HardwareSerial Serial3;   // print output, same 3DR link as bfrc

#define RC_NUM_CH        14
#define PROTOCOL_LENGTH  0x20   // iBUS frame is always 32 bytes total
#define PROTOCOL_OVERHEAD 3     // command(1) + checksum(2) bytes beyond the data
#define PROTOCOL_COMMAND 0x40   // "channel data" command byte
#define PROTOCOL_TIMEGAP_MS 3   // gap that means "a new frame is starting"

uint16_t channel[RC_NUM_CH] = {0};
uint32_t frameTotal  = 0;   // checksum-valid frames decoded, ever
uint32_t frameBad    = 0;   // frames that reached CHKSUM stage but failed it
uint32_t rxByteTotal = 0;   // every byte ever seen on Serial1
bool     linkOK = false;
unsigned long lastFrameMs = 0;
#define RC_TIMEOUT_MS 500

// Tiny inline state machine - see file header for why this replaces IBusBM
// for this diagnostic build.
enum { ST_LEN, ST_DATA, ST_CKL, ST_CKH } state = ST_LEN;
uint8_t  frameBuf[PROTOCOL_LENGTH];
uint8_t  frameLen = 0, framePtr = 0;
uint16_t chksum = 0;
uint8_t  chkLow = 0;
unsigned long lastByteMs = 0;

// Raw capture ring buffer, filled by the SAME read that feeds the parser
// above - one read per byte, seen by both, so nothing can desync.
#define HEXBUF_LEN 512
uint8_t  hexbuf[HEXBUF_LEN];
uint16_t hexHead = 0;
uint32_t hexTotalCaptured = 0;

unsigned long lastPrintMs = 0;
#define PRINT_INTERVAL_MS 100
unsigned long lastHexDumpMs = 0;
#define HEXDUMP_INTERVAL_MS 1000

static float axis(uint16_t raw) {
  if (raw == 0) return 0.0f;
  int d = (int)raw - 1500;
  if (d > 75)  return constrain((d - 75) / 425.0f, -1.0f, 1.0f);
  if (d < -75) return constrain((d + 75) / 425.0f, -1.0f, 1.0f);
  return 0.0f;
}

// Feed exactly one raw byte through the frame-sync state machine. Mirrors
// IBusBM.cpp's own logic (same states, same constants) so a real iBUS frame
// decodes identically to how it would in bfrc.
static void feedByte(uint8_t v, unsigned long nowMs) {
  if (nowMs - lastByteMs >= PROTOCOL_TIMEGAP_MS) state = ST_LEN;
  lastByteMs = nowMs;

  switch (state) {
    case ST_LEN:
      if (v <= PROTOCOL_LENGTH && v > PROTOCOL_OVERHEAD) {
        framePtr = 0;
        frameLen = v - PROTOCOL_OVERHEAD;
        chksum   = 0xFFFF - v;
        state    = ST_DATA;
      }
      // else: not a plausible length byte, stay in ST_LEN and wait for one
      break;

    case ST_DATA:
      frameBuf[framePtr++] = v;
      chksum -= v;
      if (framePtr == frameLen) state = ST_CKL;
      break;

    case ST_CKL:
      chkLow = v;
      state  = ST_CKH;
      break;

    case ST_CKH: {
      uint16_t received = ((uint16_t)v << 8) | chkLow;
      if (chksum == received) {
        // Valid frame. frameBuf[0] is the command byte (should be 0x40);
        // frameBuf[1..] are channel values, little-endian, 2 bytes each.
        if (frameBuf[0] == PROTOCOL_COMMAND) {
          for (uint8_t i = 0; i + 2 < frameLen && (i / 2) < RC_NUM_CH; i += 2) {
            channel[i / 2] = frameBuf[i + 1] | ((uint16_t)frameBuf[i + 2] << 8);
          }
          frameTotal++;
          linkOK = true;
          lastFrameMs = nowMs;   // set right here, at the moment of success -
                                  // no approximation, no separate bookkeeping
        }
      } else {
        frameBad++;
      }
      state = ST_LEN;
      break;
    }
  }
}

void setup() {
  Serial3.begin(115200);
  Serial1.begin(115200);   // iBUS nominal baud - if the receiver runs a
                            // different rate, EVERY byte here will be garbage
                            // and the hex dump will show it (random-looking,
                            // no repeating sync byte).
  delay(200);
  Serial3.println("BOOT:RC_TEST (standalone parser, no IBusBM)");
  Serial3.println("Reading raw serial on PA10 @115200. Printing on Serial3 @115200.");
  Serial3.println("HEX: read the sync byte off these to identify the protocol:");
  Serial3.println("  iBUS frame starts 0x20 0x40 ... | SBUS frame starts 0x0F ...");
}

void loop() {
  unsigned long now = millis();

  // Single read per byte: feeds BOTH the frame parser and the hex capture
  // from the exact same call, so the two can never disagree about what was
  // actually on the wire.
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    rxByteTotal++;
    hexTotalCaptured++;
    hexbuf[hexHead] = b;
    hexHead = (uint16_t)((hexHead + 1) % HEXBUF_LEN);
    feedByte(b, now);
  }

  // Failsafe: lastFrameMs is set exactly once, inside feedByte(), the instant
  // a checksum-valid frame lands. No valid frame for RC_TIMEOUT_MS -> LOST.
  // Guarded on frameTotal so a robot that has never seen one frame does not
  // read "linkOK" off millis()'s value at boot (lastFrameMs starts at 0, and
  // millis() - 0 exceeds the timeout almost immediately either way, but this
  // makes the intent explicit rather than relying on that coincidence).
  if (frameTotal == 0 || now - lastFrameMs > RC_TIMEOUT_MS) linkOK = false;

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    Serial3.print("RC:");
    for (uint8_t i = 0; i < 10; i++) { Serial3.print(channel[i]); Serial3.print(','); }
    Serial3.print("LINK:"); Serial3.print(linkOK ? 1 : 0);
    Serial3.print(",RXB:"); Serial3.print(rxByteTotal);
    Serial3.print(",FR:");  Serial3.print(frameTotal);
    Serial3.print(",BAD:"); Serial3.print(frameBad);
    Serial3.println();

    Serial3.print("AXIS: Ch3(drive)=");  Serial3.print(axis(channel[2]), 2);
    Serial3.print(" Ch4(steer)=");       Serial3.print(axis(channel[3]), 2);
    Serial3.print(" Ch8(target)=");      Serial3.print(axis(channel[7]), 2);
    Serial3.print(" Ch10(crouch)=");     Serial3.print(axis(channel[9]), 2);
    Serial3.println();
  }

  if (now - lastHexDumpMs >= HEXDUMP_INTERVAL_MS) {
    lastHexDumpMs = now;
    uint16_t n = (hexTotalCaptured < 32) ? (uint16_t)hexTotalCaptured : 32;
    Serial3.print("HEX(last "); Serial3.print(n);
    Serial3.print(" of "); Serial3.print(hexTotalCaptured); Serial3.print(" total):");
    for (uint16_t k = 0; k < n; k++) {
      uint16_t idx = (uint16_t)((hexHead + HEXBUF_LEN - n + k) % HEXBUF_LEN);
      uint8_t v = hexbuf[idx];
      Serial3.print(' ');
      if (v < 0x10) Serial3.print('0');
      Serial3.print(v, HEX);
    }
    Serial3.println();
  }
}
