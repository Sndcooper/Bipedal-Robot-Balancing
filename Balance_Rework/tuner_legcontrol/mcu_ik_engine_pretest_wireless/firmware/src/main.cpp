// ============================================================================
// mcu_ik_engine_pretest_wireless — pure radio latency diagnostic
// STM32 Bluepill F103C8 | Serial3 PB10/PB11 @ 115200 | NO IMU | NO AX-12
// ============================================================================
// Sends at 20 Hz:  S:<seq>,T:<us>,P:<synth>,DT:<loop_us>,BD:<body_us>\n
// Handles PING:<token>\n  →  replies PONG:<token>\n
// ============================================================================

#include <Arduino.h>
#include <math.h>

// Serial3 is instantiated by -DENABLE_HWSERIAL3 in platformio.ini
extern HardwareSerial Serial3;

// ── State ─────────────────────────────────────────────────────────────────────
static uint32_t      seqNum     = 0;
static unsigned long lastLoopUs = 0;
static unsigned long lastTxUs   = 0;

// ── RX — non-blocking, 40 µs budget per call ──────────────────────────────────
static char    rxBuf[48];
static uint8_t rxLen = 0;

static void handleRX() {
  const unsigned long deadline = micros() + 40;
  while (Serial3.available() && (long)(deadline - micros()) > 0) {
    char c = (char)Serial3.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        rxLen = 0;
        // PING:<token>  →  PONG:<token>
        if (rxBuf[0]=='P' && rxBuf[1]=='I' && rxBuf[2]=='N' && rxBuf[3]=='G') {
          Serial3.print("PONG");
          Serial3.println(rxBuf + 4);  // echo token + \r\n
        }
      }
    } else if (rxLen < 47) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ── TX helper — non-blocking, skips if TX buffer cannot fit the line ───────────
//  STM32duino USART3 TX buffer = 256 bytes.  Our frame ≤ 56 bytes.
//  Check ensures we never block the 100 Hz loop.
static void safePrintln(const char* str, uint8_t len) {
  if (Serial3.availableForWrite() >= len) {
    Serial3.write((const uint8_t*)str, len);
  }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial3.begin(115200);
  lastLoopUs = micros();
  lastTxUs   = micros();
  // Wait for radio to be ready, then announce
  delay(500);
  Serial3.println("BOOT:PRETEST");
}

// ── loop (100 Hz gate) ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();

  // ── 100 Hz gate ──────────────────────────────────────────────────────────
  unsigned long dt = now - lastLoopUs;
  if (dt < 10000UL) return;
  lastLoopUs = now;

  unsigned long bodyStart = micros();

  // Synthetic pitch: ±15° sine so PC can verify values arrive intact
  float synth = 15.0f * sinf((float)now * 0.5e-6f);

  handleRX();   // ≤ 40 µs

  // ── 20 Hz telemetry ──────────────────────────────────────────────────────
  if (now - lastTxUs >= 50000UL) {
    lastTxUs = now;

    unsigned long bodyUs = micros() - bodyStart;

    char frame[64];
    uint8_t len = (uint8_t)snprintf(frame, sizeof(frame),
      "S:%lu,T:%lu,P:%.2f,DT:%lu,BD:%lu\n",
      (unsigned long)seqNum,
      (unsigned long)now,
      synth,
      dt,
      bodyUs);

    safePrintln(frame, len);
    seqNum++;
  }
}