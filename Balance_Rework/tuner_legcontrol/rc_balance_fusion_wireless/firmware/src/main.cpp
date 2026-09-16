// ============================================================================
// rc_balance_fusion_wireless — STAGE 3: BALANCER + FLYSKY RC REMOTE CONTROL
// STM32 Bluepill F103C8 | 3DR telemetry Serial3 (PB10/PB11 @ 115200)
// AX-12 bus Serial2 (PA2/PA3 @ 1 Mbaud) | MPU6050 I2C1 (PB6/PB7)
// FlySky FS-iA10B iBUS receiver on Serial1 (PA10 RX @ 115200)
// ----------------------------------------------------------------------------
// WHAT THIS IS: mcu_balance_fusion_wireless with RC added. The balance PID, the
// velocity->lean outer loop, the crouch IK, the AX-12 bus arbitration and the
// whole GUI protocol are carried over UNCHANGED so a gain set tuned there is
// still valid here. The only new control input is the transmitter.
//
// WHAT RC ADDS (and what it deliberately does NOT do):
//   * Ch2 drive  -> a VELOCITY target for the existing outer loop, NOT a PWM
//     and NOT a pitch offset. Leaning is how this robot accelerates, so a drive
//     stick that wrote pitch directly would fight the balance loop. Feeding the
//     loop that already converts velocity error into lean keeps one authority
//     over the setpoint. This is the same choice RC_mcu_IK_wireless makes.
//   * Ch1 steer  -> a differential PWM trim added after the balance PID, the
//     only place a steering term can go without corrupting the pitch loop.
//   * Ch5 arm, Ch6 crouch, Ch7 calibrate, Ch8 integral-kill.
//   * A FAILSAFE: loss of iBUS frames disarms the motors (see RC_TIMEOUT_MS).
//
// THE TIMING PROBLEM THIS FILE SOLVES (the reason it is not a copy-paste of
// RC_mcu_IK_wireless's readRC):
//   IBusBM::begin(serial) defaults to timerid=0, which on STM32 seizes TIM1 and
//   runs IBusBM::loop() from a 1 ms ISR. Inside that ISR, the sensor-telemetry
//   branch calls delayMicroseconds(100) and blocking stream->write()s. An ISR
//   that can burn 100+ us at a moment of its own choosing, 1000 times a second,
//   is unbounded jitter injected straight into a 100 Hz PID whose whole tick
//   budget is 10 000 us — exactly the "loads it and delays the system" failure
//   to avoid. TIM1 may also be wanted for PWM.
//   THEREFORE: begin(Serial1, IBUSBM_NOTIMER). No timer, no ISR. Frames are
//   drained cooperatively from loop() under an explicit microsecond budget,
//   the same discipline handleTelemetryRX() already uses for the radio. We send
//   no iBUS sensor telemetry upstream, so nothing needs 1 ms servicing.
//
// The RC drain is ALSO decimated: an iBUS frame arrives every ~7 ms, so reading
// it at the full 100 Hz would mostly find an empty buffer. RC_POLL_INTERVAL_MS
// (10 ms) sets how often the drain runs. Stick values are held between polls —
// a 10 ms deadband on the INPUT, while the balance PID keeps its full 100 Hz.
// ============================================================================

#include <Arduino.h>
#include <IBusBM.h>
#include <Wire.h>

// ── ENCODER PINS (velocity telemetry only — not used for control) ────────────
#define ENC_L_A PA6
#define ENC_L_B PA7
#define ENC_R_A PB0
#define ENC_R_B PB1

volatile long encoderLeft  = 0;
volatile long encoderRight = 0;
long prevEncoderLeft  = 0;
long prevEncoderRight = 0;

void countLeft()  { if (digitalRead(ENC_L_B)) encoderLeft--;  else encoderLeft++;  }
void countRight() { if (digitalRead(ENC_R_B)) encoderRight--; else encoderRight++; }

// ── MOTOR PINS (L298N) ───────────────────────────────────────────────────────
#define ENA PA1
#define IN1 PB14
#define IN2 PB15
#define ENB PA0
#define IN3 PB12
#define IN4 PB13

// ── SERIAL PORTS (instantiated via build_flags) ──────────────────────────────
extern HardwareSerial Serial1;   // FlySky iBUS in (PA10 RX) — INPUT ONLY
extern HardwareSerial Serial2;   // AX-12 bus
extern HardwareSerial Serial3;   // 3DR radio

// ============================================================================
// RC RECEIVER — FlySky FS-iA10B over iBUS on Serial1 (PA10 RX)
// ----------------------------------------------------------------------------
// Channel map. Ch1/Ch2 are the right-hand stick on a default FS-i6/i6X mode-2
// transmitter; verify yours with the RC:* telemetry line before trusting it —
// readChannel() is 0-indexed, so Ch<n> is readChannel(n-1).
//
//   Ch3  (idx2) DRIVE     : self-centring pot, fwd/back -> velocity target for
//                           the outer (velocity->lean) loop. Idle -> 0.
//   Ch4  (idx3) STEER     : self-centring pot -> differential PWM trim applied
//                           AFTER the balance PID. Idle -> 0.
//   Ch5  (idx4) CALIBRATE : momentary button, rising edge starts an IMU cal.
//   Ch6  (idx5) TARGET=0  : momentary button, rising edge zeroes the target.
//   Ch7  (idx6) MOTORS    : 2-position switch. HIGH = armed, LOW = disarmed.
//   Ch8  (idx7) TARGET +- : SWITCH (two-state on this TX). Held = targetAngle
//                           slews at RC_TARGET_RATE deg/sec; released = hold.
//
//   Ch9  (idx8) LEG SEL   : 3-position switch. LOW = both legs, MID = left
//                           only, HIGH = right only. Gates Ch10.
//   Ch10 (idx9) CROUCH +- : SWITCH (two-state on this TX). Held = the leg(s)
//                           Ch9 selected move at RC_CROUCH_RATE mm/sec.
//
// Ch8 and Ch10 are DISCRETE: their resting position is captured at boot and
// defined as idle, so leave every switch where you want it before powering up.
//
// Ch1/Ch2 are unused so a mode-2 TX's right-hand stick is left alone.
//
// This map now matches bfrc and mcu_balance_fusion_wireless. It previously read
// Ch1/Ch2/Ch5, which a logged session proved the transmitter never drives.
// ============================================================================
IBusBM ibus;

// Pulse-width thresholds. iBUS reports microseconds: ~1000 low, ~1500 centre,
// ~2000 high. A channel the receiver has never populated reads 0 — every parse
// below must treat 0 as "no data", never as a stick at its low endpoint, or an
// unbound channel would read as a held-down switch. This is the same hazard
// RC_mcu_IK_wireless guards with `if (chN > 0)` on every channel.
#define RC_MIN        1000
#define RC_CENTRE     1500
#define RC_MAX        2000
#define RC_SWITCH_ON  1500    // above this a 2-position switch counts as HIGH

// Stick deadband, in microseconds either side of centre. Cheap gimbals do not
// return to exactly 1500 and the potentiometers are noisy; without this the
// robot creeps whenever the sticks are nominally centred. 75 us of 500 us of
// travel = 15% dead, which is generous but a balancing robot that drifts on a
// released stick is worse than one with a slightly coarse centre.
#define RC_DEADBAND   75

// How often the iBUS buffer is drained, in milliseconds. THE USER-FACING
// TIMING KNOB. An iBUS frame is 32 bytes every ~7 ms, so:
//   * polling faster than 7 ms mostly finds an empty buffer and wastes ticks
//   * polling at 10 ms (every 100 Hz tick) is already faster than the data
//     arrives, so stick latency is bounded by the RECEIVER's 7 ms frame rate,
//     not by this number
// Stick values are held between polls, so the control loop always has a value
// and never stalls waiting for the radio. Raise this to 20 ms to halve the RC
// cost per second if the tick budget ever gets tight; latency rises to ~27 ms
// worst case, which is still far below human reaction time.
#define RC_POLL_INTERVAL_MS  10

// Hard microsecond cap on one drain. A full 32-byte iBUS frame at 115200 baud
// is ~2.8 ms of AIR time, but the bytes are already sitting in the UART ring
// buffer by the time we look, so draining them is memory-speed, not baud-speed:
// measured well under 100 us for a whole frame. 400 us is a 4% slice of the
// 10 ms budget and is a backstop against a pathological burst, not a normal
// operating limit. Partial frames are fine — IBusBM is a byte-at-a-time state
// machine, so it resumes mid-frame on the next poll with no data loss.
#define RC_BUDGET_US  400

// Failsafe. The FS-iA10B keeps emitting frames when the TX is off (it holds or
// zeroes channels depending on its failsafe config), so "no frames at all"
// means the receiver itself is unpowered, unbound, or the signal wire is cut.
// Either way the operator has lost control of an armed inverted pendulum, so
// the motors are disarmed. 500 ms is ~70 missed frames — long enough that a
// burst of interference cannot nuisance-trip it, short enough that a runaway
// robot does not get far.
#define RC_TIMEOUT_MS 500

// ── PER-CHANNEL CENTRE, AUTO-CAPTURED ──────────────────────────────────────
// A hard-coded 1500 centre is wrong on real hardware. Logged evidence: Ch8 of
// this transmitter rests at 1608-1616 us. That is 109 us off 1500 -- OUTSIDE
// the 75 us deadband -- so the Ch8 rate control saw a permanent +0.08 axis and
// integrated targetAngle at 0.53 deg/sec, with nobody touching the pot, until
// it pinned at the clamp 38 s later. The robot then balanced to that lean and
// drove away underneath itself.
//
// So each analog channel's resting value is captured as ITS OWN centre, once,
// from the first frames after the link comes up. Leave the sticks alone at
// power-up (standard RC practice) and every axis then reads exactly 0 at rest.
//
// Guarded two ways: a captured centre is only accepted if it is within +-250 us
// of nominal (a stick held hard over at boot is rejected, and 1500 is kept),
// and the capture happens once per boot so it can never chase a moving stick.
uint16_t rcCen[10] = {RC_CENTRE, RC_CENTRE, RC_CENTRE, RC_CENTRE, RC_CENTRE,
                      RC_CENTRE, RC_CENTRE, RC_CENTRE, RC_CENTRE, RC_CENTRE};
// Boot-resting position of each DISCRETE channel (-1 / 0 / +1), captured
// alongside the centres. Needed because a two-state switch has no neutral: at
// rest it sits hard at one endpoint, which a plain threshold reads as permanent
// full deflection. Whatever position the switch is in at power-up is therefore
// defined as idle, and only the OTHER position commands motion.
int8_t   rcIdle[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool     rcCenCaptured = false;
uint8_t  rcCenSamples  = 0;
#define RC_CEN_SAMPLES   20     // ~20 frames, about 150 ms of iBUS
#define RC_CEN_TOLERANCE 250    // reject an implausible "centre"
// How far from nominal centre a discrete channel must sit to count as thrown.
// 250 us puts the boundary a quarter of the way across the 1000..2000 travel,
// so a switch that does not hit its endpoints exactly still resolves.
#define RC_SWITCH_MARGIN 250
// Consecutive polls a new discrete state must hold before it is acted on.
// 3 polls at the 10 ms tick is ~30 ms -- far longer than the single-frame
// glitches seen in the logs, far shorter than any real switch throw.
#define RC_SWITCH_DEBOUNCE 3

bool  rcEnabled   = true;    // RC0/RC1 command — lets the GUI take sole control
bool  rcLinkOK    = false;   // frames arriving?
unsigned long rcLastFrameMs = 0;
uint8_t rcPrevFrameCount = 0;   // IBusBM::cnt_rec, to detect fresh frames

// Decoded stick state, held between polls.
float rc_drive     = 0.0f;   // -1..+1, forward positive
float rc_steer     = 0.0f;   // -1..+1, right positive
bool  rc_arm       = false;  // Ch5 armed?
bool  rcPrevArm    = false;  // edge detect on Ch5
bool  rcPrevCal    = false;  // edge detect on Ch5 (calibrate)
bool  rcPrevZero   = false;  // edge detect on Ch6 (target = 0)
float rc_targetAxis = 0.0f;  // Ch8, -1..+1, raw axis before rate integration
float rc_crouchAxis = 0.0f;  // Ch10, -1..+1, raw axis before rate integration
uint8_t rc_legSel   = 0;     // Ch9: 0 = both legs, 1 = left only, 2 = right only
bool  rcHaveSeenArmLow = false;  // see readRC(): boot-time arm interlock

// Raw values, forwarded to the GUI so the operator can verify the channel map
// and see the failsafe state without a transmitter-side display.
// TEN channels, not eight. Ch9 (leg select) and Ch10 (crouch rate) were
// simply never read before, which is the entire reason RC crouch did nothing:
// the pot moved, the receiver sent it, and the firmware never looked.
uint16_t rc_raw[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// How far the sticks are allowed to command. Both are deliberately modest —
// this is a tall inverted pendulum on two wheels, not a car.
// 800, doubled from 400. The logged runs sat at RCD:1.00 / TVEL:400.0 with the
// stick hard over, i.e. the operator was asking for everything the limit had.
// Kp_vel 0.015 x 800 = 12 deg of commanded lean at full stick, which is exactly
// MAX_LEAN_CMD -- so full stick now reaches the lean clamp and not past it.
float RC_MAX_VEL   = 800.0f;  // counts/sec at full drive stick   — RV cmd
// 80, doubled from 40: at 40 counts of differential PWM the machine turned
// so slowly it read as "not turning". 80 is still under a third of the 255
// full-scale, so steering alone cannot saturate a motor and starve the balance
// PID -- the property the original 40 was chosen to guarantee.
float RC_MAX_STEER = 80.0f;   // PWM counts at full steer stick    — RS cmd
float RC_MAX_CROUCH = 40.0f;  // mm of crouch, GUI CR + RC Ch10    — RCM cmd
// 7.5 mm/sec (2.5x the first pass), in the same millimetres the GUI's crouch bar
// uses, so the RC switch and the slider speak one unit. Ch10 is a switch, so this
// is a single held rate rather than a band: full RC_MAX_CROUCH travel in ~5.3 s.
float RC_CROUCH_RATE = 7.5f;  // mm/sec while Ch10 is held         — RCR cmd

// ── STEERING RESPONSE CURVE ───────────────────────────────────────────────
// "Proportional" steering without a second control loop. A linear stick->PWM
// map spends most of its useful range in the first few degrees of stick, which
// is why 80 counts feels like an on/off turn. Blending in a CUBIC term keeps
// full authority at the stops but flattens the response around centre, so small
// stick movements give small, gradual corrections:
//
//   steer = RC_MAX_STEER * ( k*a^3 + (1-k)*a )      a = -1..+1
//
// k = 0 is the old linear map, k = 1 is pure cubic. 0.6 gives roughly a third of
// the old sensitivity at quarter stick while still reaching 80 counts at full.
// This is a static shaping of the COMMAND, not feedback: nothing to tune against
// the plant, no integrator, no extra state, and no interaction with the balance
// PID, which owns the common-mode output while steering owns the differential.
float RC_STEER_EXPO = 0.6f;   // 0 = linear, 1 = fully cubic       — RSE cmd

// ── Ch8 TARGET-RATE control — IN DEGREES ───────────────────────────────────
// Ch8 slews targetAngle, the same variable the GUI's "Target" slider sets with
// S<f>. That slider's range is -20..+20 DEGREES, so this band is deg/sec and
// the clamp is degrees:
// ONE rate, not a band. Ch8 on this transmitter is a two-state switch: it reads
// one endpoint or the other and nothing in between, so a squared taper between
// a minimum and a maximum rate has no travel to interpolate over and would only
// ever produce the maximum. Held = 0.05 deg/sec, released = stop.
//
// 0.05 deg/sec is a trim, deliberately: it takes ~2.7 minutes to cross the whole
// +-8 deg clamp, so the switch cannot walk the setpoint anywhere dangerous while
// the operator's attention is on the robot.
float RC_TARGET_RATE = 0.05f;   // deg/sec while Ch8 is held   — RTR cmd
// 8, not 20. The logged run pinned targetAngle at the old +20 clamp and the
// robot tried to hold a 20 deg lean against a 25 deg safety cutoff -- 5 deg of
// margin for every disturbance, and the session duly ended in SAFETY:CUTOFF.
// 8 deg is a real, usable trim range that still leaves 17 deg of headroom.
float RC_TARGET_LIMIT    =  8.0f;   // clamp on targetAngle, deg   — RTL cmd

// ── MPU6050 ───────────────────────────────────────────────────────────────────
const int MPU_ADDR = 0x68;
float pitch = 0.0f, pitchOffset = 0.0f;
float accelPitchRaw = 0.0f;
float gyroRate = 0.0f;
#define GYRO_PITCH_SIGN 1.0f     // flip to -1.0f if pitch runs the wrong way

// ── THE SINGLE BALANCE PID (the ONLY control loop) ───────────────────────────
float Kp = 78.0f, Ki = 650.0f, Kd = 3.52f;   // neutral bench-tuning start
float integral = 0.0f;
float alpha = 0.96f;                        // complementary-filter coefficient
float targetAngle = 0.0f;                   // balance setpoint (operator trim)
float maxSafeTilt = 25.0f;                  // safety cutoff threshold (deg)
// Anti-windup cap on the Ki term, in PWM units. MUST stay well inside the
// actuator range (+/-255) or it is a clamp in name only: at 1200 the integral
// term ALONE could command 4.7x everything the motors can deliver, so the
// integrator saturated the output long before its own clamp ever engaged.
// Logged sessions showed it pinned at the clamp while the robot fell over.
const float MAX_INTEGRAL_PWM = 64.0f;       // 25% of range, leaves Kp/Kd headroom

// ── ENCODER VELOCITY (telemetry display; also feeds auto-trim below) ────────
float vel_current = 0.0f;
float vel_alpha   = 0.85f;

// ── AUTO-TRIM (opt-in drift-cancelling bias) ─────────────────────────────────
// Ports the RC_mcu_IK_wireless velocity-integral idea (same sign convention
// and same order-of-magnitude gain as its Ki_vel) into this single-loop
// firmware: while enabled, any steady encL/encR drift means targetAngle is
// off from the true balance point, so integrate vel_current toward zero and
// add the result on top of targetAngle. "TC" then bakes the converged bias
// into targetAngle permanently, replacing a manual re-trim.
// ── OUTER LOOP: VELOCITY -> LEAN ANGLE (the cascade) ────────────────────────
// THE KEY IDEA: this loop's output is an ANGLE, not a PWM. A balancing robot
// cannot be commanded to move directly -- leaning IS how it accelerates. So a
// velocity error is converted into the lean the inner loop must then hold.
//
//   pushed FORWARD  ->  vel_current > 0  ->  (0 - vel) < 0  ->  lean_cmd < 0
//                   ->  robot leans BACKWARD  ->  gravity decelerates it
//
// Verified in simulation: with this sign a 0.35 m/s push recovers with ~7 cm
// of drift; with the sign flipped the robot falls in 1.29 s.
//
// Why a cascade at all: the torque->position transfer function has a
// RIGHT-HALF-PLANE zero at ~7.1 rad/s (to move forward the wheels must first
// go backward). That caps any velocity loop at ~0.56 Hz, while the unstable
// pole at 11 rad/s forces the balance loop to run 10x faster. One flat PID
// has a single bandwidth and physically cannot serve both.
bool  autoTrimEnabled = true;           // velocity loop ON by default in this variant
                                         // (TE0 disables it -> reverts to pure single-loop
                                         //  PID, so the two can be A/B compared on the bench)
float Kp_vel  = 0.0030f;                // deg of lean per (count/s) of error  — VP cmd
float Ki_trim = 0.0015f;                // deg of lean per (count/s) per second — TG cmd
float trim_bias = 0.0f;                 // integral part of the lean command (deg)
float lean_cmd  = 0.0f;                 // TOTAL commanded lean = P + I (deg), telemetry

// Velocity SETPOINT for the outer loop. In mcu_balance_fusion_wireless this was
// the literal 0.0f — that firmware has no drive input, so any motion was error.
// Here the RC drive stick writes it, which is the single substantive change to
// the control path: the loop that used to only reject motion now also commands
// it, using the same gains. Zero when the stick is centred, so with the TX
// sticks at rest this firmware behaves EXACTLY like the variant it came from.
float target_velocity = 0.0f;           // counts/sec, forward positive

// ══ CLOSED-LOOP TURN RATE ═════════════════════════════════════════════════
// Steering used to be an open-loop differential PWM: the same stick deflection
// gave a different yaw rate as the battery sagged, the floor changed, or one
// wheel took more load than the other, and nothing measured the difference.
//
// The measurement is free. deltaL and deltaR are already mirror-normalised in
// the encoder block below (both count positive going forward), so:
//     deltaL + deltaR  -> pure TRANSLATION, rotation cancels  -> vel_current
//     deltaL - deltaR  -> pure ROTATION, translation cancels  -> yaw_current
// Both encoders are read every tick regardless, so this costs one subtraction.
//
// ONE P GAIN, deliberately, and this is not a second PID:
//   * no integrator -- it would wind up against a wheel that is scrubbing
//     rather than rolling, then dump that authority when it broke free;
//   * no derivative -- yaw_current is already an EMA of a difference of
//     integers, so differentiating it again is pure noise amplification.
// Feed-forward carries the response and the P term only trims the error, which
// is why a single gain is enough here where the balance loop needs three.
float yaw_current = 0.0f;               // counts/sec, + = turning right
float yaw_target  = 0.0f;               // counts/sec commanded by the stick
// RETAINED BUT UNUSED by the control law: steering is open loop (see the
// steer block in loop()). Kept, with their KY/RYF/RYA commands, so the closed
// loop can be re-enabled for an experiment without another firmware revision.
// Setting them has no effect while the open-loop law is in place.
float Kp_yaw      = 0.02f;              // PWM counts per counts/sec error - KY
// STILL LOAD-BEARING even with the loop gone: readRC() scales the shaped stick
// into yaw_target with it, and the open-loop law divides it back out to recover
// the -1..+1 command. Changing it does NOT change steering authority (that is
// RC_MAX_STEER); it only rescales the YT telemetry. Keep it non-zero.
float RC_MAX_YAW  = 700.0f;             // counts/sec at full steer stick  - RY
float RC_YAW_FF   = 0.55f;              // unused (see Kp_yaw)             - RYF
// ── AUTHORITY BUDGET FOR THE CORRECTION TERM ──────────────────────────────
// The total steer clamp is not enough on its own: it lets the P term spend the
// WHOLE differential budget on heading error nobody asked to correct. Measured
// in serial_COM13_20260915_235020.log over 1085 frames with no turn commanded,
// Kp_yaw*YW alone saturated the full +-RC_MAX_STEER on 16% of ticks and passed
// half authority on 39%, because idle |YW| runs a median 352 c/s (peak 1466)
// while only 80/0.09 = 889 is needed to saturate.
//
// On a machine that must stay upright, a heading trim must never outbid the
// balance PID for PWM. So the correction gets its own, much smaller budget;
// a DELIBERATE turn is untouched because that arrives via feed-forward, which
// still commands the full RC_MAX_STEER.
float RC_YAW_AUTH = 20.0f;              // unused (see Kp_yaw)             - RYA
// Clamped in DEGREES because that is what this loop outputs. 6 deg is a real
// lean the robot can hold; the old 15 deg was 60% of the 25 deg safety cutoff,
// so a wound-up bias could trip the cutoff on its own.
const float MAX_TRIM_BIAS = 6.0f;

// TOTAL commanded lean (Kp_vel term + learned trim). Doubled from the 6 deg the
// trim integrator uses, because 6 deg was the whole reason a drive command
// barely leaned the robot: with RV400 the proportional term alone asked for
// ~4.6 deg, so the clamp truncated nearly every real drive command and the
// machine crept instead of accelerating.
//
// Kept as its OWN constant rather than raising MAX_TRIM_BIAS: the trim
// integrator must stay at 6, since it is the term that can wind up unattended.
// 12 deg of lean plus the +-8 deg target clamp sits at 20 deg against the 25 deg
// safety cutoff -- deliberately tight, so do not raise this without also
// raising maxSafeTilt.
const float MAX_LEAN_CMD  = 12.0f;

bool motorsEnabled = false;
bool safetyLatched = false;

// ── LOOP TIMING ───────────────────────────────────────────────────────────────
unsigned long lastTime      = 0;
unsigned long lastPrintTime = 0;

// ── AX-12 HELPERS ─────────────────────────────────────────────────────────────
// Shared by every write/read path below — one place to discard whatever our own
// half-duplex echo (or a timed-out reply) left sitting in Serial2's RX buffer.
void ax12DrainRx() { while (Serial2.available()) Serial2.read(); }

// Status Return Level is set to 1 (reply to READ only) in initAX12Legs(), so a
// WRITE never gets a status packet back — only the half-duplex echo of the
// bytes we just sent. Draining it HERE, once, means every caller (initAX12Legs,
// applyServoSettings, ...) automatically leaves Serial2's RX buffer clean for
// whatever runs next in the tick, instead of each call site having to remember
// to do it (applyServoSettings() didn't, which desynced pollLegServosTask()'s
// byte-counted READ parser whenever a settings push and a health poll landed
// in the same or adjacent ticks).
void ax12WriteByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t checksum = ~(id + 4 + 3 + addr + val) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x03, addr, val, checksum};
  Serial2.write(packet, 8);
  Serial2.flush();       // TX drained: bus can turn around
  ax12DrainRx();          // RX drained: our own echo, discarded
}

void ax12WriteWord(uint8_t id, uint8_t addr, uint16_t val) {
  uint8_t lo = val & 0xFF;
  uint8_t hi = (val >> 8) & 0xFF;
  uint8_t checksum = ~(id + 5 + 3 + addr + lo + hi) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x05, 0x03, addr, lo, hi, checksum};
  Serial2.write(packet, 9);
  Serial2.flush();
  ax12DrainRx();
}

// ── LEG SERVO STATE (held pose + health only, no IK) ─────────────────────────
struct ServoState {
  uint8_t  id;
  uint16_t goalPos;      // calibrated standing pose
  uint16_t torqueLimit;
  uint8_t  compMargin;
  uint8_t  compSlope;
  uint8_t  temp;
  float    loadPct;
  // ── LIVENESS / ALARM ──────────────────────────────────────────────────────
  // A servo that latches an AX-12 Alarm Shutdown (overheat / overload) goes
  // limp and silently IGNORES every goal write. Before these fields existed the
  // only symptom was temp freezing at its last value — or 0 if it never
  // answered at all — which is indistinguishable from "working fine, just not
  // polled yet". That is exactly how four servos cooked themselves unnoticed.
  uint8_t  err;          // status-packet ERROR byte: bit2 overheat, bit5 overload
  uint8_t  failCount;    // consecutive reads with no valid reply (255 = capped)
};

// Torque/compliance restored to the responsive baseline (was 511 / margin 4 /
// slope 32). Slope 32 is an 8x wider proportional band and margin 4 a wide
// deadband, which together make the servo ease in slowly and hold weakly — the
// joint visibly creeps toward its target under the robot's own weight, which
// reads as "slow servos" independently of any serial latency.
//   torqueLimit 511 -> 1023 : full holding torque (see note: doubles jam force)
//   compMargin     4 -> 1   : narrow deadband, reacts to small errors
//   compSlope     32 -> 4   : tight proportional band, no sluggish ease-in
ServoState legServos[4] = {
  {6,  818, 1023, 1, 4, 0, 0.0f, 0, 0},   // Leg1 Left  (818 = straight-down left)
  {0,  818, 1023, 1, 4, 0, 0.0f, 0, 0},   // Leg2 Left
  {14, 441, 1023, 1, 4, 0, 0.0f, 0, 0},   // Leg1 Right (441 = straight-down right)
  {1,  441, 1023, 1, 4, 0, 0.0f, 0, 0},   // Leg2 Right
};

// ── GLOBAL SERVO SETTINGS (ported from ax12_control — applies to all four) ───
// Identical to what the balancing firmware hard-codes in initAX12Legs() but now
// live-tunable from the GUI without reflashing. markAllSettingsDirty() queues
// an applySettingsTask() spread — one servo per 100 Hz tick — so a compliance
// slider drag costs ~630 µs (4 ticks) instead of 2.5 ms in one shot.
uint16_t g_torqueLimit = 1023;   // addr 34, 0-1023  (1023 = full holding torque)
uint8_t  g_compMargin  = 1;      // addr 26/27, 0-254 (narrow deadband)
uint8_t  g_compSlope   = 4;      // addr 28/29, 0-254 (tight proportional band)
uint16_t g_movingSpeed = 0;      // addr 32, 0-1023  (0 = uncapped, servos race at max speed)
bool     g_torqueOn    = true;   // addr 24 master kill: false = limp legs

// Dirty bitmask — one bit per legServos[] index. Set by any settings command,
// drained one servo per tick by applySettingsTask(). Prevents the 2.5 ms
// blocking bath that would quarter the loop's free time.
uint8_t settingsDirtyMask = 0x0F;  // push all registers at first boot tick
void markAllSettingsDirty() { settingsDirtyMask = 0x0F; }

// ── POSE TRAJECTORY ENGINE (ported verbatim from ax12_control) ───────────────
// Two mutually exclusive pose modes. In CROUCH the foot x is pinned to the
// calibrated default and one knob slides it vertically; in IK the dragged (x,y)
// is used as-is. Only CROUCH is available when the balance PID is active (it
// maps 1-to-1 with the existing crouchOffset path). IK is for bench posing.
#define MODE_CROUCH 0
#define MODE_IK     1
uint8_t  poseMode   = MODE_CROUCH;

// 240 ms, 0.3x the old 800. 800 ms made every GUI-driven pose change a slow
// glide; it also meant a re-issued move (which restarts the interpolation)
// delivered only a sliver of each step. RC crouch bypasses this path entirely
// now, so this figure only shapes CR/CRL/CRR/FT/mode-change moves.
uint16_t moveTimeMs = 240;              // MT<n>, 100-3000 ms per move
bool     moveActive = false;
// Set by the RC crouch path in readRC() when cur_*/goalPos have been updated
// and need to reach the servos. NOT written to the bus there: readRC() runs at
// the top of loop(), before the 3-mode bus arbiter, so writing from it puts a
// SECOND Serial2 transaction in a tick that is architecturally allowed exactly
// one. Doing that corrupted the half-duplex byte accounting and permanently
// killed the bus 3 s into the first crouch (see the crouch block in readRC()).
// The arbiter consumes this flag in its own slot instead.
bool     crouchDirty = false;
unsigned long moveStartMs = 0;
float mv_x0[2], mv_y0[2];               // where the move started
float mv_x1[2], mv_y1[2];               // where it ends
float cur_x[2] = {1.0f, -6.0f};         // authoritative current foot position (mm)
float cur_y[2] = {-151.1f, -149.6f};
float ft_x[2]  = {1.0f, -6.0f};         // IK-mode commanded foot targets (mm)
float ft_y[2]  = {-151.1f, -149.6f};
bool  ikValid[2] = {true, true};         // last solve was in-reach?

// ── CROUCH IK (opt-in, one degree of freedom: stand tall <-> crouch) ────────
// Ports the 5-bar solve_ik/map_angle_to_ax12 pair from mcu_ik_engine_wireless
// verbatim (same mounts, same 818/441 calibration) so the math is proven, not
// re-derived. No lean/turn here on purpose — this variant only needs a single
// "how low is the body over the wheels" knob, driven by the CR<f> command.
#define SERVO_L_X -30.0f
#define SERVO_L_Y   0.0f
#define SERVO_R_X  30.0f
#define SERVO_R_Y   0.0f
#define FEMUR_LEN  55.0f
#define TIBIA_LEN 100.0f
#define LEG2_INVERTED_MOUNT true

const float ik_fx1 = 1.0f,  ik_fy1 = -151.1f;   // Leg1 standing foot target (mm)
const float ik_fx2 = -6.0f, ik_fy2 = -149.6f;   // Leg2 standing foot target (mm)
const float ik_dist = 180.0f;                    // leg separation (mm)
// Per-leg crouch: 0 = standing (default), + = crouched (mm). Independently
// settable via CRL<f>/CRR<f>; CR<f> is a convenience that sets both at once
// (kept for back-compat with the old single "one vertical knob" behaviour —
// the GUI's mirror checkbox drives this path when linked).
float crouchOffsetL = 0.0f;  // Leg1 crouch
float crouchOffsetR = 0.0f;  // Leg2 crouch

struct Point2D { float x; float y; };

bool circle_intersections(Point2D p0, float r0, Point2D p1, float r1,
                           Point2D &out1, Point2D &out2) {
  float dx = p1.x - p0.x, dy = p1.y - p0.y;
  float d  = sqrtf(dx * dx + dy * dy);
  if (d > r0 + r1 || d < fabsf(r0 - r1) || d == 0) return false;
  float a  = (r0 * r0 - r1 * r1 + d * d) / (2.0f * d);
  float h  = sqrtf(fmaxf(r0 * r0 - a * a, 0.0f));
  float px = p0.x + a * dx / d;
  float py = p0.y + a * dy / d;
  float rx = -h * dy / d, ry = h * dx / d;
  out1 = {px + rx, py + ry};
  out2 = {px - rx, py - ry};
  return true;
}

struct IK_Result { bool valid; Point2D Knee_L, Knee_R; float Angle_L, Angle_R; };

IK_Result solve_ik(float tx, float ty, float leg_offset_x) {
  IK_Result res = {false};
  Point2D foot = {tx, ty};
  Point2D sl   = {SERVO_L_X + leg_offset_x, SERVO_L_Y};
  Point2D sr   = {SERVO_R_X + leg_offset_x, SERVO_R_Y};
  Point2D li1, li2, ri1, ri2;
  if (!circle_intersections(sl, FEMUR_LEN, foot, TIBIA_LEN, li1, li2)) return res;
  if (!circle_intersections(sr, FEMUR_LEN, foot, TIBIA_LEN, ri1, ri2)) return res;
  res.valid  = true;
  res.Knee_L = (li1.x < li2.x) ? li1 : li2;
  res.Knee_R = (ri1.x > ri2.x) ? ri1 : ri2;
  res.Angle_L = atan2f(res.Knee_L.y - sl.y, res.Knee_L.x - sl.x) * 180.0f / PI;
  res.Angle_R = atan2f(res.Knee_R.y - sr.y, res.Knee_R.x - sr.x) * 180.0f / PI;
  return res;
}

uint16_t map_angle_to_ax12(float ik_angle, bool is_left, bool is_leg2) {
  float base_angle = is_leg2 ? 90.0f : -90.0f;
  float diff_deg   = fmodf(ik_angle - base_angle + 180.0f, 360.0f);
  if (diff_deg < 0) diff_deg += 360.0f;
  diff_deg -= 180.0f;
  float base_pos = is_left ? 818.0f : 441.0f;
  float ax_pos   = base_pos + (diff_deg * 3.413f);
  return (uint16_t)constrain((int)ax_pos, 0, 1023);
}

void initAX12Legs() {
  for (int i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    ax12WriteByte(id, 16, 1);                         // Status Return Level (EEPROM, once)
    ax12WriteByte(id, 5,  0);                         // Return Delay Time = 0 (EEPROM, once)
    ax12WriteByte(id, 24, 1);                         // Torque Enable
    // Globals, not the struct's frozen construction-time copies — otherwise a
    // servo reset (SR) silently reverts to 1023/1/4 and discards whatever the
    // operator set on the sliders.
    ax12WriteWord(id, 34, g_torqueLimit);             // Torque Limit
    ax12WriteByte(id, 26, g_compMargin);              // CW  Compliance Margin
    ax12WriteByte(id, 27, g_compMargin);              // CCW Compliance Margin
    ax12WriteByte(id, 28, g_compSlope);               // CW  Compliance Slope
    ax12WriteByte(id, 29, g_compSlope);               // CCW Compliance Slope
    ax12WriteWord(id, 30, legServos[i].goalPos);      // Goal Position (standing pose)
  }
}

// ── PRESENT POSITION READ (blocking) — TQ1 re-grip only, never the hot path ──
// Reads addr 36 len 2 (Present Position). Blocking is fine here: this runs
// synchronously in response to one deliberate operator command (TQ1), not on
// every 100 Hz tick like pollLegServosTask() — a 20 ms/servo worst case is
// nothing next to the human reaction time behind the command that triggered it.
bool ax12ReadPresentPos(uint8_t id, uint16_t &posOut) {
  // No pre-clear needed: ax12WriteByte/Word now drain their own echo (see
  // above), so Serial2's RX buffer is already quiet by the time this runs.
  uint8_t checksum = ~(id + 4 + 2 + 36 + 2) & 0xFF;
  uint8_t packet[] = {0xFF, 0xFF, id, 0x04, 0x02, 36, 2, checksum};
  Serial2.write(packet, 8);
  Serial2.flush();

  unsigned long start = millis();                // 8-byte echo + 8-byte reply
  while (Serial2.available() < 16) {
    if (millis() - start >= 20) {                 // servo silent — give up
      ax12DrainRx();                              // caller keeps last goalPos
      return false;
    }
  }

  uint8_t buf[16];
  for (uint8_t i = 0; i < 16; i++) buf[i] = Serial2.read();
  if (buf[8] != 0xFF || buf[9] != 0xFF || buf[10] != id) return false;
  posOut = buf[13] | ((uint16_t)buf[14] << 8);
  return true;
}

// ── FORWARD KINEMATICS — for safe re-grip after limp ─────────────────────────
// Exact inverse of map_angle_to_ax12(). Only needed when the operator has
// moved the legs by hand (TQ0 limp -> reposition -> TQ1 re-grip). Without FK,
// the first sync-write of the next move would jump back to the pre-limp pose.
float ax12_to_angle(uint16_t pos, bool is_left, bool is_leg2) {
  float base_angle = is_leg2 ? 90.0f : -90.0f;
  float base_pos   = is_left ? 818.0f : 441.0f;
  return base_angle + ((float)pos - base_pos) / 3.413f;
}

bool solve_fk(float aL_deg, float aR_deg, float leg_offset_x, Point2D &foot) {
  Point2D sl = {SERVO_L_X + leg_offset_x, SERVO_L_Y};
  Point2D sr = {SERVO_R_X + leg_offset_x, SERVO_R_Y};
  Point2D kL = {sl.x + FEMUR_LEN * cosf(aL_deg * PI / 180.0f),
                sl.y + FEMUR_LEN * sinf(aL_deg * PI / 180.0f)};
  Point2D kR = {sr.x + FEMUR_LEN * cosf(aR_deg * PI / 180.0f),
                sr.y + FEMUR_LEN * sinf(aR_deg * PI / 180.0f)};
  Point2D f1, f2;
  if (!circle_intersections(kL, TIBIA_LEN, kR, TIBIA_LEN, f1, f2)) return false;
  foot = (f1.y < f2.y) ? f1 : f2;   // knee-up branch: the lower intersection
  return true;
}

// ── SYNC_WRITE — all four goal positions in ONE 20-byte bus packet ────────────
// THE FIX for the 80 ms intra-leg stagger: one broadcast hands every servo its
// goal simultaneously. Being broadcast (ID 0xFE) there is no status reply —
// only the half-duplex echo of our own bytes, drained here.
void ax12SyncWriteGoals() {
  uint8_t pkt[20];
  pkt[0] = 0xFF; pkt[1] = 0xFF;
  pkt[2] = 0xFE;                      // broadcast ID
  pkt[3] = (2 + 1) * 4 + 4;          // LENGTH = (data_len+1)*n_servos + 4 = 16
  pkt[4] = 0x83;                      // SYNC_WRITE opcode
  pkt[5] = 30;                        // start address: Goal Position (RAM)
  pkt[6] = 2;                         // bytes per servo
  uint16_t sum = 0xFE + pkt[3] + 0x83 + 30 + 2;
  uint8_t k = 7;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t id = legServos[i].id;
    uint8_t lo = legServos[i].goalPos & 0xFF;
    uint8_t hi = (legServos[i].goalPos >> 8) & 0xFF;
    pkt[k++] = id; pkt[k++] = lo; pkt[k++] = hi;
    sum += id + lo + hi;
  }
  pkt[k++] = ~(sum & 0xFF) & 0xFF;
  Serial2.write(pkt, k);
  Serial2.flush();
  ax12DrainRx();                                 // drain our own half-duplex echo
}

// ── POSE TRAJECTORY HELPERS ──────────────────────────────────────────────────
void computeDesiredFoot(float *fx, float *fy) {
  if (poseMode == MODE_CROUCH) {
    fx[0] = ik_fx1;  fy[0] = ik_fy1 + crouchOffsetL;
    fx[1] = ik_fx2;  fy[1] = ik_fy2 + crouchOffsetR;
  } else {
    fx[0] = ft_x[0]; fy[0] = ft_y[0];
    fx[1] = ft_x[1]; fy[1] = ft_y[1];
  }
}

// Solve IK for both legs and cache counts into legServos[].goalPos.
// Does NOT touch the bus — ax12SyncWriteGoals() does that atomically.
void solveGoalsFor(const float *fx, const float *fy) {
  IK_Result sol1 = solve_ik(fx[0], fy[0], 0.0f);
  IK_Result sol2 = solve_ik(fx[1] + ik_dist, fy[1], ik_dist);
  ikValid[0] = sol1.valid;
  ikValid[1] = sol2.valid;
  if (!sol1.valid || !sol2.valid) return;   // out of reach — hold last good pose

  uint16_t p6  = map_angle_to_ax12(sol1.Angle_L, true,  false);
  uint16_t p14 = map_angle_to_ax12(sol1.Angle_R, false, false);
  float ikL2 = sol2.Angle_L, ikR2 = sol2.Angle_R;
  if (LEG2_INVERTED_MOUNT) { ikL2 = -sol2.Angle_R; ikR2 = -sol2.Angle_L; }
  uint16_t p0 = map_angle_to_ax12(ikL2, true,  true);
  uint16_t p1 = map_angle_to_ax12(ikR2, false, true);

  for (int i = 0; i < 4; i++) {
    if      (legServos[i].id == 6)  legServos[i].goalPos = p6;
    else if (legServos[i].id == 14) legServos[i].goalPos = p14;
    else if (legServos[i].id == 0)  legServos[i].goalPos = p0;
    else if (legServos[i].id == 1)  legServos[i].goalPos = p1;
  }
}

// Begin an interpolated move from current foot to new target. Safe to restart
// mid-flight: always departs from cur_*, never from the stale old endpoint.
void startPoseMove() {
  computeDesiredFoot(mv_x1, mv_y1);
  for (int i = 0; i < 2; i++) { mv_x0[i] = cur_x[i]; mv_y0[i] = cur_y[i]; }
  moveStartMs = millis();
  moveActive  = true;
}

// Snap to target immediately — no interpolation. Used at boot and on SR.
void snapPose() {
  computeDesiredFoot(cur_x, cur_y);
  solveGoalsFor(cur_x, cur_y);
  moveActive = false;
}

// 100 Hz interpolation tick: advance the foot along a smoothstep curve and
// broadcast the new IK solution to all four servos simultaneously.
void updateMoveTask() {
  if (!moveActive) return;
  unsigned long elapsed = millis() - moveStartMs;
  float f = (moveTimeMs == 0) ? 1.0f : (float)elapsed / (float)moveTimeMs;
  if (f >= 1.0f) { f = 1.0f; moveActive = false; }

  // Smoothstep: starts/ends at zero velocity so legs never snap into or slam
  // out of motion even at full torque. For a linear ramp use f directly.
  float s = f * f * (3.0f - 2.0f * f);

  for (int i = 0; i < 2; i++) {
    cur_x[i] = mv_x0[i] + (mv_x1[i] - mv_x0[i]) * s;
    cur_y[i] = mv_y0[i] + (mv_y1[i] - mv_y0[i]) * s;
  }
  solveGoalsFor(cur_x, cur_y);
  ax12SyncWriteGoals();
}

// ── SETTINGS PUSH — one servo per tick, never batched ─────────────────────────
// Applies g_torqueLimit/compMargin/compSlope/movingSpeed/torqueOn to one servo
// per tick (identified by settingsDirtyMask). 7 register writes ≈ 630 µs — a
// quarter of the tick — which is why they are spread rather than sent all at once.
void applyServoSettings(uint8_t idx) {
  uint8_t id = legServos[idx].id;
  ax12WriteWord(id, 34, g_torqueLimit);       // Torque Limit          (RAM)
  ax12WriteByte(id, 26, g_compMargin);        // CW  Compliance Margin (RAM)
  ax12WriteByte(id, 27, g_compMargin);        // CCW Compliance Margin (RAM)
  ax12WriteByte(id, 28, g_compSlope);         // CW  Compliance Slope  (RAM)
  ax12WriteByte(id, 29, g_compSlope);         // CCW Compliance Slope  (RAM)
  ax12WriteWord(id, 32, g_movingSpeed);       // Moving Speed          (RAM)
  ax12WriteByte(id, 24, g_torqueOn ? 1 : 0);  // Torque Enable         (RAM)
}

void applySettingsTask() {
  if (settingsDirtyMask == 0) return;
  for (uint8_t i = 0; i < 4; i++) {
    if (settingsDirtyMask & (1 << i)) {
      applyServoSettings(i);
      settingsDirtyMask &= ~(1 << i);
      return;                    // exactly one servo per tick
    }
  }
}

// ── SERVO HEALTH POLL — non-blocking, one servo per 20 ms ─────────────────────
enum PollState { POLL_IDLE, POLL_WAITING };
PollState     pollState       = POLL_IDLE;
unsigned long lastPollTime    = 0;
unsigned long waitStartTime   = 0;
uint8_t       currentServoIdx = 0;
const unsigned long POLL_INTERVAL_MS = 20;

void pollLegServosTask() {
  unsigned long now = millis();

  if (pollState == POLL_IDLE) {
    if (now - lastPollTime < POLL_INTERVAL_MS) return;
    ServoState &s = legServos[currentServoIdx];

    // ── REMOVED: per-servo register writes from the poll slot ──────────────
    // Goal position:  owned by ax12SyncWriteGoals() — one SYNC_WRITE packet
    //                 broadcasts all 4 goals simultaneously. Writing goalPos
    //                 here individually reintroduces the 80 ms stagger.
    // Torque/settings: owned by applySettingsTask() via settingsDirtyMask.
    //                  Writing s.torqueLimit / s.compMargin here (hardcoded
    //                  struct fields) overrides g_torqueLimit / g_compMargin
    //                  set by the GUI slider, and forces torque ON even after
    //                  a TQ0 (limp) command — the exact bugs reported.
    // The poll is now READ-ONLY: fire the request, parse the reply, nothing more.



    // Every write path already drains its own echo (see ax12DrainRx()), so
    // this is normally a no-op — but it's cheap insurance against a stray
    // byte from EMI/noise on the bus landing here and shifting the fixed
    // 8-echo/10-reply byte count this state machine relies on below.
    ax12DrainRx();

    // READ addr 40 len 4 → Load(2B) Volt(1B) Temp(1B)
    uint8_t checksum = ~(s.id + 4 + 2 + 40 + 4) & 0xFF;
    uint8_t packet[] = {0xFF, 0xFF, s.id, 0x04, 0x02, 40, 4, checksum};
    Serial2.write(packet, 8);

    pollState     = POLL_WAITING;
    waitStartTime = now;
  }
  else if (pollState == POLL_WAITING) {
    if (Serial2.available() >= 18) {              // 8-byte echo + 10-byte reply
      for (int i = 0; i < 8; i++) Serial2.read();
      uint8_t reply[10];
      for (int i = 0; i < 10; i++) reply[i] = Serial2.read();

      if (reply[0] == 0xFF && reply[1] == 0xFF &&
          reply[2] == legServos[currentServoIdx].id) {
        uint16_t loadRaw = reply[5] | (reply[6] << 8);
        uint8_t  temp    = reply[8];
        legServos[currentServoIdx].temp    = temp;
        legServos[currentServoIdx].loadPct = ((loadRaw & 0x3FF) / 1023.0f) * 100.0f;
        // reply[4] is the status packet's ERROR byte. Ignoring it was the whole
        // problem: an overheat/overload shutdown is reported HERE and nowhere
        // else, so a latched servo looked identical to a healthy one.
        legServos[currentServoIdx].err       = reply[4];
        legServos[currentServoIdx].failCount = 0;
      } else if (legServos[currentServoIdx].failCount < 255) {
        legServos[currentServoIdx].failCount++;   // header mismatch = bad read
      }
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
    else if (now - waitStartTime > 20) {          // timeout — servo silent
      if (legServos[currentServoIdx].failCount < 255)
        legServos[currentServoIdx].failCount++;
      ax12DrainRx();
      currentServoIdx = (currentServoIdx + 1) % 4;
      lastPollTime    = millis();
      pollState       = POLL_IDLE;
    }
  }
}

// ── IMU ───────────────────────────────────────────────────────────────────────
void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readIMU(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)12, (uint8_t)true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  (void)(Wire.read() << 8 | Wire.read()); // temp
  (void)(Wire.read() << 8 | Wire.read()); // gyroX
  int16_t gy = Wire.read() << 8 | Wire.read();

  accelPitchRaw = atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / PI;
  float accelPitch = accelPitchRaw - pitchOffset;
  gyroRate = GYRO_PITCH_SIGN * (float)gy / 131.0f;
  pitch = alpha * (pitch + gyroRate * dt) + (1.0f - alpha) * accelPitch;
}

// ── IMU CALIBRATION — non-blocking state machine ─────────────────────────────
// Was a blocking `for (100) { readIMU(); delay(10); }` — a full 1000 ms with
// the loop dead: no RX (so commands sent during/just after a calibration were
// delayed a whole second and often lost to UART FIFO overflow), no motor
// update, no telemetry. Same 100 samples over the same ~1 s, but now one
// sample per 100 Hz tick with the loop still servicing everything else.
// readIMU() is already called once per tick by loop(), so this only
// accumulates — it must run AFTER readIMU() in the tick.
const int CAL_SAMPLES = 100;
bool  calibrating   = false;
int   calSampleIdx  = 0;
float calSum        = 0.0f;

void startCalibration() {
  calibrating  = true;
  calSampleIdx = 0;
  calSum       = 0.0f;
  Serial3.println("CAL:START");
}

void calibrationTask() {
  if (!calibrating) return;

  calSum += accelPitchRaw;          // this tick's fresh reading, from readIMU()
  calSampleIdx++;

  if (calSampleIdx >= CAL_SAMPLES) {
    pitchOffset = calSum / (float)CAL_SAMPLES;
    pitch       = 0.0f;
    integral    = 0.0f;             // offset moved; a stale integral would kick
    calibrating = false;
    char buf[48];
    int n = snprintf(buf, sizeof(buf), "CAL:DONE,OFFSET:%.4f\n", pitchOffset);
    if (n > 0 && n < (int)sizeof(buf) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)buf, n);
  }
}

// ── MOTOR DRIVER ──────────────────────────────────────────────────────────────
void setMotors(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM  >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else               { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  analogWrite(ENA, abs(leftPWM));

  if (rightPWM >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else               { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(ENB, abs(rightPWM));
}

// ── COMMAND PARSER — single-loop balance + calibration only ──────────────────
void parseCommand(char *cmd) {
  // 224, not 160: the "Updated ->" ack below now carries the three RC limits
  // as well, and snprintf would silently truncate the tail fields the GUI
  // parses. Sized with headroom for the widest float formatting.
  // 288, not 224. The "Updated ->" ack now also carries the yaw-rate gains,
  // and snprintf drops the TAIL fields when short -- which is where they are,
  // so a too-small buffer shows up as "the new slider never syncs" rather than
  // as any kind of error.
  char ack[288];

  // PING:<token> → PONG:<token>  (keeps latency_test.py working)
  if (cmd[0]=='P' && cmd[1]=='I' && cmd[2]=='N' && cmd[3]=='G') {
    Serial3.print("PONG");
    Serial3.println(cmd + 4);
    return;
  }

  // ── NEW TWO-CHAR COMMANDS (ax12_control ported) — must precede single-char ──
  // All TQ/TL/CM/CS/MS/MT/FT/FA/HM/SR/MD/RB checked here so they can never
  // collide with the single-letter TE/TG/TC/T/C/M/R/S/P cases below.

  // TQ0 / TQ1 — limp / re-grip
  // Re-gripping seeds goalPos and cur_* from the last-read presentPos so the
  // legs move smoothly from wherever the operator's hands left them.
  if (cmd[0]=='T' && cmd[1]=='Q') {
    bool want = (cmd[2] == '1');
    if (want && !g_torqueOn) {
      // Re-seed goalPos from a live present-position read of each servo.
      // (Was `legServos[i].goalPos = legServos[i].torqueLimit` — copied the
      // torque-limit register, ~1023, into the goal position, which slammed
      // every leg toward its travel limit on every re-grip.)
      for (int i = 0; i < 4; i++) {
        uint16_t pos;
        if (ax12ReadPresentPos(legServos[i].id, pos)) legServos[i].goalPos = pos;
        // else: servo didn't answer in time — keep the last known goalPos
        // rather than guess.
      }
      // --- FK re-seed of cur_* so the first move departs from the right foot ---
      uint16_t p6=818, p14=441, p0=818, p1=441;
      for (int i = 0; i < 4; i++) {
        if      (legServos[i].id == 6)  p6  = legServos[i].goalPos;
        else if (legServos[i].id == 14) p14 = legServos[i].goalPos;
        else if (legServos[i].id == 0)  p0  = legServos[i].goalPos;
        else if (legServos[i].id == 1)  p1  = legServos[i].goalPos;
      }
      Point2D f;
      if (solve_fk(ax12_to_angle(p6,  true,  false),
                   ax12_to_angle(p14, false, false), 0.0f, f)) {
        cur_x[0] = f.x; cur_y[0] = f.y;
      }
      float ikL2 = ax12_to_angle(p0, true,  true);
      float ikR2 = ax12_to_angle(p1, false, true);
      float aL2 = ikL2, aR2 = ikR2;
      if (LEG2_INVERTED_MOUNT) { aL2 = -ikR2; aR2 = -ikL2; }
      if (solve_fk(aL2, aR2, ik_dist, f)) {
        cur_x[1] = f.x - ik_dist; cur_y[1] = f.y;
      }
      moveActive = false;
    }
    g_torqueOn = want;
    markAllSettingsDirty();
    snprintf(ack, sizeof(ack), "ACK:TORQUE_%s", g_torqueOn ? "ON" : "LIMP");
    Serial3.println(ack);
    return;
  }

  // TL<n> — torque limit 0-1023
  else if (cmd[0]=='T' && cmd[1]=='L') {
    g_torqueLimit = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  // CM<n> — compliance margin 0-254
  else if (cmd[0]=='C' && cmd[1]=='M') {
    g_compMargin = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  // CS<n> — compliance slope 0-254
  else if (cmd[0]=='C' && cmd[1]=='S') {
    g_compSlope = (uint8_t)constrain(atoi(cmd + 2), 0, 254);
    markAllSettingsDirty();
  }
  // MS<n> — moving speed 0-1023 (0 = uncapped)
  else if (cmd[0]=='M' && cmd[1]=='S') {
    g_movingSpeed = (uint16_t)constrain(atoi(cmd + 2), 0, 1023);
    markAllSettingsDirty();
  }
  // MT<n> — move time ms 100-3000
  else if (cmd[0]=='M' && cmd[1]=='T') {
    moveTimeMs = (uint16_t)constrain(atoi(cmd + 2), 100, 3000);
  }
  // FT<leg> <x> <y> — IK foot target for leg 1 or 2 (mm)
  else if (cmd[0]=='F' && cmd[1]=='T') {
    int leg = atoi(cmd + 2);
    char *s1 = strchr(cmd + 2, ' ');
    char *s2 = s1 ? strchr(s1 + 1, ' ') : NULL;
    if (s1 && s2 && (leg == 1 || leg == 2)) {
      ft_x[leg-1] = atof(s1 + 1);
      ft_y[leg-1] = atof(s2 + 1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      Serial3.println("FT:ERR BAD_FORMAT");
      return;
    }
  }
  // FA <x1> <y1> <x2> <y2> — BOTH legs atomically (avoids 2-FT stagger)
  else if (cmd[0]=='F' && cmd[1]=='A') {
    char *a = strchr(cmd + 2, ' ');
    char *b = a ? strchr(a + 1, ' ') : NULL;
    char *c = b ? strchr(b + 1, ' ') : NULL;
    char *d = c ? strchr(c + 1, ' ') : NULL;
    if (a && b && c && d) {
      ft_x[0]=atof(a+1); ft_y[0]=atof(b+1);
      ft_x[1]=atof(c+1); ft_y[1]=atof(d+1);
      if (poseMode == MODE_IK) startPoseMove();
    } else {
      Serial3.println("FA:ERR BAD_FORMAT");
      return;
    }
  }
  // HM — home: standing pose in both modes
  else if (cmd[0]=='H' && cmd[1]=='M') {
    crouchOffsetL = 0.0f;
    crouchOffsetR = 0.0f;
    ft_x[0]=ik_fx1; ft_y[0]=ik_fy1;
    ft_x[1]=ik_fx2; ft_y[1]=ik_fy2;
    startPoseMove();
  }
  // SR — servo reset (re-init bus registers)
  else if (cmd[0]=='S' && cmd[1]=='R') {
    initAX12Legs();
    snapPose();
    Serial3.println("ACK:SERVOS_RESET");
    return;
  }
  // MD0 / MD1 — pose mode: 0=CROUCH, 1=IK
  else if (cmd[0]=='M' && cmd[1]=='D') {
    uint8_t want = (cmd[2] == '1') ? MODE_IK : MODE_CROUCH;
    if (want != poseMode) {
      if (want == MODE_IK) {
        ft_x[0]=ik_fx1; ft_y[0]=ik_fy1+crouchOffsetL;
        ft_x[1]=ik_fx2; ft_y[1]=ik_fy2+crouchOffsetR;
      } else {
        crouchOffsetL = ft_y[0]-ik_fy1;
        crouchOffsetR = ft_y[1]-ik_fy2;
      }
    }
    poseMode = want;
    startPoseMove();
  }
  // RB — request state broadcast (GUI resync after connect)
  else if (cmd[0]=='R' && cmd[1]=='B') {
    // Build and send an AX12 state line immediately
    char sl[208];
    int n = snprintf(sl, sizeof(sl),
      "AX12:MODE:%u,TQ:%u,TL:%u,CM:%u,CS:%u,MS:%u,MT:%u,CRL:%.1f,CRR:%.1f,"
      "FX1:%.2f,FY1:%.2f,FX2:%.2f,FY2:%.2f,IK1:%u,IK2:%u,MOVE:%d\n",
      (unsigned)poseMode,(unsigned)(g_torqueOn?1:0),
      (unsigned)g_torqueLimit,(unsigned)g_compMargin,
      (unsigned)g_compSlope,(unsigned)g_movingSpeed,
      (unsigned)moveTimeMs, crouchOffsetL, crouchOffsetR,
      ft_x[0],ft_y[0],ft_x[1],ft_y[1],
      (unsigned)(ikValid[0]?1:0),(unsigned)(ikValid[1]?1:0),(int)moveActive);
    if (n > 0 && n < (int)sizeof(sl) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)sl, n);
    return;
  }

  // ── RC COMMANDS — multi-char, MUST precede the single-letter 'R' below ────
  // "RE0"/"RE1", "RV<f>", "RS<f>", "RCM<f>" all start with 'R', which is the
  // reset-integral command. Without these cases sitting above it, "RV400"
  // would match `cmd[0]=='R'`, reset the integrator and silently discard the
  // value — the exact prefix-collision failure the protocol doc warns about.
  // Note RCM (not RC) for max-crouch: plain "RC" would shadow nothing here,
  // but "CR" is already crouch and a second two-letter crouch command reading
  // in the opposite order is a trap, so the RC one is explicitly 3 characters.
  if (cmd[0]=='R' && cmd[1]=='E') {
    // RE0 hands sole control to the GUI: the receiver is still read (so the
    // failsafe and the RC:* telemetry keep working and you can still see the
    // sticks) but no channel is allowed to touch the control state. RE1 gives
    // the transmitter authority back. This is how you tune over the radio with
    // a powered TX on the bench without the sticks fighting your sliders.
    rcEnabled = (cmd[2] == '1');
    if (!rcEnabled) {
      rc_drive = 0.0f;
      rc_steer = 0.0f;
      target_velocity = 0.0f;
      // Deliberately does NOT disarm: pulling RC authority mid-balance would
      // drop the robot. The GUI's M command remains in charge of arming.
    }
    snprintf(ack, sizeof(ack), "ACK:RC_%s", rcEnabled ? "ON" : "OFF");
    Serial3.println(ack);
    return;
  }
  else if (cmd[0]=='R' && cmd[1]=='V') {
    // Counts/sec commanded at full drive stick. Clamped to MAX_VEL-ish sanity:
    // a target the robot cannot reach just means a permanently saturated lean.
    RC_MAX_VEL = constrain(atof(cmd + 2), 0.0f, 2000.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='S') {
    // PWM counts of differential steer at full stick. Kept well under 255 so
    // steering can never on its own saturate a motor and starve the balance
    // PID of the authority it needs to stay upright.
    RC_MAX_STEER = constrain(atof(cmd + 2), 0.0f, 120.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='C' && cmd[2]=='M') {
    RC_MAX_CROUCH = constrain(atof(cmd + 3), 0.0f, 80.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='C' && cmd[2]=='R') {
    // mm/sec of crouch travel while Ch10 is held.
    RC_CROUCH_RATE = constrain(atof(cmd + 3), 0.1f, 200.0f);
  }
  else if (cmd[0]=='K' && cmd[1]=='Y') {
    // PWM counts of differential per counts/sec of yaw-rate error.
    Kp_yaw = constrain(atof(cmd + 2), 0.0f, 2.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='Y' && cmd[2]=='A') {
    // PWM counts the yaw P term may command on its own. 0 disables heading
    // correction entirely and leaves pure open-loop feed-forward steering.
    RC_YAW_AUTH = constrain(atof(cmd + 3), 0.0f, 120.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='Y' && cmd[2]=='F') {
    // Feed-forward fraction: 0 = pure feedback, 1 = old open-loop behaviour.
    RC_YAW_FF = constrain(atof(cmd + 3), 0.0f, 1.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='Y') {
    // Counts/sec of yaw rate commanded at full steer stick. Checked AFTER RYF
    // so "RYF0.5" can never be parsed as RY with a garbage tail.
    RC_MAX_YAW = constrain(atof(cmd + 2), 0.0f, 3000.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='S' && cmd[2]=='E') {
    // Steering expo blend, 0 = linear .. 1 = fully cubic.
    RC_STEER_EXPO = constrain(atof(cmd + 3), 0.0f, 1.0f);
  }
  else if (cmd[0]=='R' && cmd[1]=='T' && cmd[2]=='R') {
    // deg/sec of target-angle travel while Ch8 is held.
    RC_TARGET_RATE = constrain(atof(cmd + 3), 0.001f, 5.0f);
  }

  // Auto-trim controls — multi-char, must be checked before the single-letter
  // 'T' (max safe tilt) case below or "TE1"/"TG.05" would parse as garbage tilt.
  else if (cmd[0]=='T' && cmd[1]=='E') {
    autoTrimEnabled = (cmd[2] == '1');
    if (!autoTrimEnabled) trim_bias = 0.0f;   // don't leave a half-converged bias live
    snprintf(ack, sizeof(ack), "ACK:AUTOTRIM_%s", autoTrimEnabled ? "ON" : "OFF");
    Serial3.println(ack);
    return;
  }
  else if (cmd[0]=='T' && cmd[1]=='C') {
    targetAngle += trim_bias;
    float committed = trim_bias;
    trim_bias = 0.0f;
    integral  = 0.0f;                         // stale balance integral would double-kick
    snprintf(ack, sizeof(ack), "TRIM:DONE COMMITTED:%.3f TARGET:%.3f", committed, targetAngle);
    Serial3.println(ack);
    return;
  }
  else if (cmd[0]=='T' && cmd[1]=='G') Ki_trim = atof(cmd + 2);
  // VP<f> — outer-loop proportional gain, deg of lean per (count/s).
  // Checked before any single-char 'V' case would be (there are none), and
  // before 'T'/'C' so it can never be mistaken for tilt or calibrate.
  else if (cmd[0]=='V' && cmd[1]=='P') Kp_vel = atof(cmd + 2);
  else if (cmd[0]=='P' && cmd[1]=='S') {
    // Raw per-servo position, e.g. "PS6 750" — bypasses the crouch IK entirely,
    // for moving/testing exactly one joint. Checked before the single-letter
    // 'P' (Kp) case below, same ordering rule as every other multi-char prefix
    // in this parser (else "PS6 750" would parse as Kp = atof("S6 750") = 0).
    char *sp = strchr(cmd + 2, ' ');
    if (sp) {
      int id  = atoi(cmd + 2);
      int pos = constrain(atoi(sp + 1), 0, 1023);
      bool found = false;
      for (int i = 0; i < 4; i++) {
        if (legServos[i].id == id) { legServos[i].goalPos = (uint16_t)pos; found = true; break; }
      }
      snprintf(ack, sizeof(ack), found ? "PS:OK ID%d POS%d" : "PS:ERR UNKNOWN_ID%d", id, pos);
    } else {
      snprintf(ack, sizeof(ack), "PS:ERR BAD_FORMAT");
    }
    Serial3.println(ack);
    return;
  }
  // Balance / tuning commands
  else if (cmd[0] == 'P' && cmd[1] != '\0') Kp = atof(cmd + 1);
  else if (cmd[0] == 'I' && cmd[1] != '\0') Ki = atof(cmd + 1);
  else if (cmd[0] == 'D' && cmd[1] != '\0') Kd = atof(cmd + 1);
  else if (cmd[0] == 'A' && cmd[1] != '\0') alpha = atof(cmd + 1);
  else if (cmd[0] == 'T' && cmd[1] != '\0') maxSafeTilt = atof(cmd + 1);
  else if (cmd[0] == 'O' && cmd[1] != '\0') {
    // Manual pitch offset — re-seed pitch against the new reference.
    pitchOffset = atof(cmd + 1);
    pitch       = accelPitchRaw - pitchOffset;
    integral    = 0.0f;
  }
  else if (cmd[0] == 'S' && cmd[1] != '\0') targetAngle = atof(cmd + 1);
  else if (cmd[0] == 'S' && cmd[1] == '\0') {
    initAX12Legs();
    Serial3.println("ACK:SERVOS_RESET");
    return;
  }
  else if (cmd[0] == 'C' && cmd[1] == 'R' && cmd[2] == 'L') {
    // Per-leg crouch — checked before plain "CR" so "CRL5" doesn't parse as
    // CR with a garbage float ("L5").
    crouchOffsetL = atof(cmd + 3);
    if (poseMode == MODE_CROUCH) startPoseMove();
  }
  else if (cmd[0] == 'C' && cmd[1] == 'R' && cmd[2] == 'R') {
    crouchOffsetR = atof(cmd + 3);
    if (poseMode == MODE_CROUCH) startPoseMove();
  }
  else if (cmd[0] == 'C' && cmd[1] == 'R') {
    // Crouch bar — sets BOTH legs (mirrored knob), checked before the bare
    // 'C' (calibrate) case, or "CR40" would trigger an IMU calibration
    // instead of setting crouch depth.
    crouchOffsetL = crouchOffsetR = atof(cmd + 2);
    // Route through the SAME interpolated trajectory engine every other pose
    // command uses. The old path called solveGoalsFor() on local fx/fy, which
    // cached goalPos but left cur_x/cur_y frozen at the standing pose. Two
    // consequences, both observed: telemetry reported FY1 = -151.10 no matter
    // what crouch was set to, and the NEXT interpolated move (mode switch, FT,
    // HM) started from that stale position and slammed the legs. It also never
    // called ax12SyncWriteGoals(), so the new goals waited on the 10 Hz hold
    // write — which is itself gated on pollState == POLL_IDLE.
    if (poseMode == MODE_CROUCH) startPoseMove();
  }
  else if (cmd[0] == 'C') { startCalibration(); return; }
  else if (cmd[0] == 'R') {
    integral  = 0.0f;
    trim_bias = 0.0f;
    Serial3.println("ACK:INT_RESET");
    return;
  }
  else if (cmd[0] == 'M') {
    motorsEnabled = !motorsEnabled;
    if (motorsEnabled) {
      safetyLatched = false;
      integral      = 0.0f;
      trim_bias     = 0.0f;
      vel_current   = 0.0f;
      encoderLeft = 0; encoderRight = 0;
      prevEncoderLeft = 0; prevEncoderRight = 0;
    }
    snprintf(ack, sizeof(ack), "Motors %s", motorsEnabled ? "ENABLED" : "DISABLED");
    Serial3.println(ack);
    return;
  }
  else return; // unknown command

  // Ack for tuning commands — parsed by _parse_fw_update() in the GUI.
  snprintf(ack, sizeof(ack),
    "Updated -> P:%.3f I:%.3f D:%.3f Offset:%.4f Target:%.3f Alpha:%.4f Tilt:%.2f TrimGain:%.4f CrouchL:%.2f CrouchR:%.2f VP:%.4f RV:%.1f RS:%.1f RCM:%.1f "
    "KY:%.3f RY:%.1f RYF:%.2f RYA:%.1f",
    Kp, Ki, Kd, pitchOffset, targetAngle, alpha, maxSafeTilt, Ki_trim, crouchOffsetL, crouchOffsetR, Kp_vel,
    RC_MAX_VEL, RC_MAX_STEER, RC_MAX_CROUCH,
    Kp_yaw, RC_MAX_YAW, RC_YAW_FF, RC_YAW_AUTH);
  uint8_t len = (uint8_t)strlen(ack);
  ack[len] = '\n'; len++;
  if (Serial3.availableForWrite() >= len)
    Serial3.write((uint8_t*)ack, len);
}

// ── NON-BLOCKING RX — 300 µs budget ──────────────────────────────────────────
// A byte at 115200 baud is 87 µs, so the old 40 µs budget drained at most ONE
// byte per call. Combined with the old 50 Hz call rate that made an 8-byte
// command ("CR40.0\n") take 8 x 20 ms = 160 ms to even reach the parser, and
// let the 64-byte UART FIFO overflow (silently corrupting commands) whenever
// the radio delivered a burst. 300 µs is 3% of the 10 ms loop and drains ~3
// bytes/tick; handleTelemetryRX() is now also called EVERY tick (see loop()).
const unsigned long RX_BUDGET_US = 300;

static char  rxBuf[48];
static uint8_t rxLen = 0;

void handleTelemetryRX() {
  unsigned long rxStart = micros();
  while (Serial3.available() && (micros() - rxStart) < RX_BUDGET_US) {
    char c = (char)Serial3.read();
    if (c >= 'a' && c <= 'z') c -= 32;         // uppercase in place
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) { rxBuf[rxLen] = '\0'; parseCommand(rxBuf); rxLen = 0; }
    } else if (rxLen < 47) {
      rxBuf[rxLen++] = c;
    }
  }
}

// ============================================================================
// RC READ — cooperative, budgeted, decimated. Called once per 100 Hz tick.
// ----------------------------------------------------------------------------
// Returns immediately on ticks where it is not due, so its cost on 9 of every
// 10 ticks is a subtraction and a compare. On the tenth it drains whatever the
// UART has buffered under RC_BUDGET_US and re-decodes the sticks.
//
// Nothing in here blocks, allocates, or calls delay(). ibus.loop() is invoked
// directly (NOT from a timer ISR — see the file header) and returns as soon as
// Serial1's buffer is empty; the budget check bounds the pathological case
// where bytes arrive as fast as we consume them.
// ============================================================================

// Map a raw iBUS pulse width to -1..+1 with a centre deadband.
// Returns exactly 0.0 inside the deadband and at the endpoints of a dead
// channel, so a disconnected or unbound channel is indistinguishable from a
// centred stick — the safe failure, not a stuck full-deflection command.
// `ch` selects that channel's captured centre. Passing the centre in (rather
// than assuming 1500) is the whole fix for the Ch8 drift described above.
static float rcAxis(uint16_t raw, uint8_t ch) {
  if (raw < RC_MIN - 200 || raw > RC_MAX + 200) return 0.0f;  // 0 or nonsense
  int delta = (int)raw - (int)rcCen[ch];
  float span = (float)(RC_MAX - RC_CENTRE - RC_DEADBAND);     // 425 us of live travel
  float axis;
  if (delta > RC_DEADBAND) {
    // Rescale so the axis still reaches 1.0 at the endpoint despite the
    // deadband eating the first RC_DEADBAND us of travel. Without the
    // rescale, full stick would only ever read (500-75)/500 = 0.85.
    axis = (float)(delta - RC_DEADBAND) / span;
  } else if (delta < -RC_DEADBAND) {
    axis = (float)(delta + RC_DEADBAND) / span;
  } else {
    return 0.0f;                                              // inside deadband
  }
  // CLAMP, and not merely belt-and-braces: the sanity window above admits up to
  // RC_MAX+200 (2200 us), and real transmitters do emit beyond 2000 — travel
  // adjust / endpoint trim above 100%, or a sub-trim offset, routinely gives
  // ~2100. Un-clamped, 2100 us rescales to 1.41, so a stick at its mechanical
  // stop would command 141% of RC_MAX_VEL and 141% of RC_MAX_STEER — silently
  // defeating both limits the operator set. Clamp AFTER the rescale.
  return constrain(axis, -1.0f, 1.0f);
}

// Discrete read for a channel that is a SWITCH rather than a proportional pot:
// returns -1, 0 or +1 and nothing between. Used for Ch8 and Ch10, which on this
// transmitter only ever emit an endpoint.
//
// The idle test is what makes a TWO-state switch safe here. A two-position
// switch has no neutral -- at rest it sits hard at 1000 or 2000 -- so a plain
// threshold would read it as permanent full deflection and the rate would
// integrate forever with nobody touching anything. Comparing against the
// position captured at boot means the resting position commands nothing, and
// only moving the switch does. On a THREE-position switch left centred at boot
// rcIdle is 0 and both directions work normally.
//
// DEBOUNCED, and not as a precaution: the logged sessions show these channels
// glitching. With nothing touching the pot, Ch8 rests at ~1611-1625 and then
// emits isolated single frames of 1023, 1000, 1512, 1077, 1119, 1129, 1583,
// 1590; Ch10 does the same (1686, 1775, 1982, 1096, 1481). Undebounced, each of
// those one-frame excursions clears the threshold and reads as a deliberate
// switch throw -- stepping targetAngle or the crouch accumulator on pure noise,
// with nobody touching the transmitter. A genuine throw lasts hundreds of
// milliseconds, so requiring the new state to survive 3 consecutive polls
// (~30 ms) rejects every glitch in those logs and costs no perceptible lag.
static int rcTriState(uint16_t raw, uint8_t ch) {
  static int8_t  stable[10] = {0,0,0,0,0,0,0,0,0,0};
  static int8_t  pending[10] = {0,0,0,0,0,0,0,0,0,0};
  static uint8_t pendCount[10] = {0,0,0,0,0,0,0,0,0,0};

  if (raw < RC_MIN - 200 || raw > RC_MAX + 200) return 0;   // 0 or nonsense
  int s;
  if      ((int)raw < RC_CENTRE - RC_SWITCH_MARGIN) s = -1;
  else if ((int)raw > RC_CENTRE + RC_SWITCH_MARGIN) s = +1;
  else                                              s =  0;

  if (s == stable[ch]) {
    pendCount[ch] = 0;                     // no change in progress
  } else if (s == pending[ch]) {
    if (++pendCount[ch] >= RC_SWITCH_DEBOUNCE) {
      stable[ch]    = (int8_t)s;           // held long enough: accept it
      pendCount[ch] = 0;
    }
  } else {
    pending[ch]   = (int8_t)s;             // a different candidate: restart
    pendCount[ch] = 1;
  }

  return (stable[ch] == rcIdle[ch]) ? 0 : stable[ch];
}

// Unipolar 0..1 read, for a channel whose pot travels one way only (Ch9's
// 3-position switch). Deliberately NOT centre-relative: a switch legitimately
// rests at an endpoint, so rcAxis()'s deadband-about-centre is meaningless here.
static float rcUnipolar(uint16_t raw) {
  if (raw == 0) return 0.0f;
  return constrain(((float)raw - (float)RC_MIN) / (float)(RC_MAX - RC_MIN),
                   0.0f, 1.0f);
}

void readRC() {
  static unsigned long rcLastPollMs = 0;
  unsigned long nowMs = millis();

  // ── DECIMATION GATE — the "10 ms deadband" on the input path ─────────────
  if (nowMs - rcLastPollMs < RC_POLL_INTERVAL_MS) return;
  rcLastPollMs = nowMs;

  // ── BUDGETED DRAIN ───────────────────────────────────────────────────────
  // ibus.loop() is a byte-at-a-time state machine that consumes whatever is in
  // Serial1's buffer and returns; it does not block waiting for more. So the
  // budget is enforced as a LOOP: keep handing it bytes while there are bytes
  // and time remains. Calling it once would be enough in normal operation, but
  // this bounds the pathological case where bytes arrive as fast as we consume
  // them — there the budget exits with a partial frame buffered, and IBusBM
  // resumes mid-frame on the next poll with no data loss.
  //
  // (Checking `micros() - t0 < RC_BUDGET_US` BEFORE the first call would be
  // vacuous — no time has passed yet — which is why the test is the loop
  // condition and not a guard in front of a single call.)
  unsigned long t0 = micros();
  while (Serial1.available() && (micros() - t0) < RC_BUDGET_US) {
    ibus.loop();
  }

  // ── LINK HEALTH ──────────────────────────────────────────────────────────
  // cnt_rec increments on every checksum-valid servo frame. A changed count
  // means a genuinely new frame landed — a far stronger signal than
  // Serial1.available(), which line noise alone can satisfy.
  //
  // cnt_rec is a uint8_t and wraps at 256. At ~140 frames/sec a wrap takes
  // ~1.8 s, so a poll interval would have to span exactly 256 frames for the
  // counts to collide — impossible at a 10 ms poll (≈1.4 frames). Noted
  // because it WOULD matter if RC_POLL_INTERVAL_MS were ever raised into the
  // seconds: the failsafe would then false-trip on a healthy link.
  if (ibus.cnt_rec != rcPrevFrameCount) {
    rcPrevFrameCount = ibus.cnt_rec;
    rcLastFrameMs    = nowMs;
    rcLinkOK         = true;
  } else if (nowMs - rcLastFrameMs > RC_TIMEOUT_MS) {
    rcLinkOK = false;
  }

  for (uint8_t i = 0; i < 10; i++) rc_raw[i] = ibus.readChannel(i);

  // ── FAILSAFE ─────────────────────────────────────────────────────────────
  // No valid frames for RC_TIMEOUT_MS: zero every command and disarm. Done
  // before any channel is decoded so a stale buffer cannot command anything.
  if (!rcLinkOK) {
    rc_drive = 0.0f;
    rc_steer = 0.0f;
    rc_crouchAxis = 0.0f;   // stop crouch travel; the legs hold where they are
    target_velocity = 0.0f;
    if (rc_arm) {                     // we were armed and just lost the link
      rc_arm = false;
      rcPrevArm = false;
      if (rcEnabled && motorsEnabled) {
        motorsEnabled = false;
        setMotors(0, 0);
        integral = 0.0f; trim_bias = 0.0f;
        Serial3.println("SAFETY:RC_LINK_LOST");
      }
    }
    // The arm interlock is re-armed too: after a link loss the operator must
    // return the switch to LOW before it can arm again, so a link that recovers
    // with the switch still HIGH does not instantly re-energise the motors.
    rcHaveSeenArmLow = false;
    return;
  }

  // ── RE0: GUI has taken control — but DISARM is never ignored ─────────────
  // RE0 parks the sticks so GUI sliders can be used with a live TX nearby. It
  // must NOT also neuter the arm switch as a kill switch: an operator reaching
  // for it on a robot that is misbehaving does not know or care that the GUI
  // holds authority. So under RE0 the switch loses its ability to ARM (the GUI
  // is driving) but keeps its ability to STOP.
  // ── ONE-SHOT CENTRE CAPTURE ──────────────────────────────────────────────
  // Runs only while disarmed, so it can never redefine "centre" mid-flight.
  if (!rcCenCaptured && !motorsEnabled) {
    if (++rcCenSamples >= RC_CEN_SAMPLES) {
      for (uint8_t i = 0; i < 10; i++) {
        int v = (int)rc_raw[i];
        // Only the self-centring axes are captured. Switches legitimately sit
        // at an endpoint, and adopting that as their "centre" would break the
        // HIGH/LOW test that arms the motors.
        // Ch8/Ch10 are discrete here, so their RESTING POSITION is captured
        // instead of a centre -- see rcTriState().
        if ((i == 7 || i == 9) && v > 0) {
          if      (v < RC_CENTRE - RC_SWITCH_MARGIN) rcIdle[i] = -1;
          else if (v > RC_CENTRE + RC_SWITCH_MARGIN) rcIdle[i] = +1;
          else                                       rcIdle[i] =  0;
        }
        bool isAxis = (i == 2 || i == 3);
        if (isAxis && v > 0 &&
            v > RC_CENTRE - RC_CEN_TOLERANCE && v < RC_CENTRE + RC_CEN_TOLERANCE) {
          rcCen[i] = (uint16_t)v;
        }
      }
      rcCenCaptured = true;
      char cb[80];
      int n = snprintf(cb, sizeof(cb), "RCCEN:%u,%u,IDLE8:%d,IDLE10:%d\n",
                       (unsigned)rcCen[2], (unsigned)rcCen[3],
                       (int)rcIdle[7], (int)rcIdle[9]);
      if (n > 0) Serial3.write((uint8_t*)cb, n);
    }
  }

  if (!rcEnabled) {
    bool armSwitchOff = (rc_raw[6] > 0) && (rc_raw[6] <= RC_SWITCH_ON);
    if (armSwitchOff && motorsEnabled) {
      rc_arm        = false;
      rcPrevArm     = false;
      motorsEnabled = false;
      setMotors(0, 0);
      integral = 0.0f; trim_bias = 0.0f; target_velocity = 0.0f;
      Serial3.println("Motors DISABLED");
    }
    return;
  }

  // ── Ch1 / Ch2: steer and drive ───────────────────────────────────────────
  // Ch3 = drive (fwd/back), Ch4 = steer (left/right).
  // REMAPPED from Ch1/Ch2. A logged session proved Ch1/Ch2 sat at 1500 for all
  // 869 telemetry lines while Ch3/Ch4 swept their full range: the transmitter
  // drives Ch3/Ch4, so reading Ch1/Ch2 made both axes permanently 0.00 and the
  // drive/steer commands were silently dead. This now matches bfrc and
  // mcu_balance_fusion_wireless.
  rc_drive = rcAxis(rc_raw[2], 2);   // Ch3
  rc_steer = rcAxis(rc_raw[3], 3);   // Ch4
  // Drive stick -> velocity setpoint for the outer loop. Only while armed;
  // a stick pushed before arming must not bank a setpoint that takes effect
  // the instant the motors come on.
  // ── EXPO ON DRIVE ────────────────────────────────────────────────────────
  // Drive was already a velocity command, so it was "proportional" in the loop
  // sense, but it had no curve: the first millimetre of stick threw a large
  // velocity step at the outer loop. The cubic blend keeps full RC_MAX_VEL at
  // the stops and softens everything around centre, which is where the operator
  // does the fine work. Shares RC_STEER_EXPO with the steering below so drive
  // and turn feel like one control rather than two differently-geared ones.
  float dA = rc_drive;
  float driveShaped = RC_STEER_EXPO * dA * dA * dA + (1.0f - RC_STEER_EXPO) * dA;
  target_velocity = (motorsEnabled && rc_arm) ? (driveShaped * RC_MAX_VEL) : 0.0f;

  // ── STEER STICK -> YAW RATE TARGET ───────────────────────────────────────
  // Same curve, and negated for the same reason the old PWM path was: this
  // machine's Ch4 reads the opposite way round (stick left commanded a right
  // turn). Putting the inversion here, at the one place the transmitter's
  // polarity enters, keeps the motor-wiring convention downstream intact.
  float sA = -rc_steer;
  float steerShaped = RC_STEER_EXPO * sA * sA * sA + (1.0f - RC_STEER_EXPO) * sA;
  yaw_target = (motorsEnabled && rc_arm) ? (steerShaped * RC_MAX_YAW) : 0.0f;

  // ── Ch5: ARM / DISARM (2-position switch, edge-detected) ─────────────────
  // Interlock: the switch must be observed LOW at least once since boot (or
  // since a link loss) before it can arm. Otherwise powering the robot up with
  // the switch already HIGH would arm the motors the moment the first frame
  // arrives, with nobody's hand on it.
  // Ch7 = ARM. REMAPPED from Ch5: the log showed Ch7 toggling 1000<->2000
  // (the operator working the arm switch) while Ch5 stayed at 1000, so RCA
  // read 0 on 866 of 869 lines and the motors could not be armed from the TX.
  bool armSwitch = (rc_raw[6] > 0) && (rc_raw[6] > RC_SWITCH_ON);
  if (rc_raw[6] > 0 && !armSwitch) rcHaveSeenArmLow = true;

  // Resync with a disarm that came from somewhere else — the GUI's 'M' command
  // or the tilt-cutoff latch. Without this, rc_arm would still be true while
  // motorsEnabled is false, so the switch (already HIGH) would produce no edge
  // and the operator could not re-arm from the transmitter without cycling the
  // switch. Clearing rc_arm here makes the next LOW->HIGH flip a real edge.
  if (rc_arm && !motorsEnabled) rc_arm = false;

  if (armSwitch != rcPrevArm) {
    rcPrevArm = armSwitch;
    if (armSwitch && rcHaveSeenArmLow) {
      rc_arm        = true;
      motorsEnabled = true;
      safetyLatched = false;          // an RC arm clears a previous tilt latch
      integral      = 0.0f;
      trim_bias     = 0.0f;
      vel_current   = 0.0f;
      target_velocity = 0.0f;
      encoderLeft = 0; encoderRight = 0;
      prevEncoderLeft = 0; prevEncoderRight = 0;
      Serial3.println("Motors ENABLED");   // same string the GUI already parses
    } else if (!armSwitch) {
      rc_arm        = false;
      motorsEnabled = false;
      setMotors(0, 0);
      integral = 0.0f; trim_bias = 0.0f; target_velocity = 0.0f;
      Serial3.println("Motors DISABLED");
    }
  }

  // ── Ch6: crouch knob -> crouch depth, both legs ──────────────────────────
  // Routed through the SAME trajectory engine the CR<f> command uses, and only
  // on a meaningful change: startPoseMove() restarts the interpolation, so
  // calling it on every poll from a jittering pot would continuously restart
  // the move and the legs would never arrive. 1 mm of hysteresis on a 40 mm
  // range is well outside pot noise.
  // Ch6 = "target = 0" momentary button (REMAPPED; was a crouch position pot).
  // "Stop here": clears the Ch8 trim and the state that would otherwise carry
  // the old target forward -- velocity setpoint, the outer loop's learned trim,
  // and the encoder origin. Safe to press while balancing.
  //
  // The old Ch6 crouch-pot behaviour is dropped because the operator's layout
  // puts crouch on Ch9/Ch10 (leg-select + rate), and this variant only decodes
  // 8 channels -- so crouch is not reachable here at all. Use bfrc or
  // mcu_balance_fusion_wireless for RC crouch control.
  if (rc_raw[5] > 0) {
    bool zeroSwitch = (rc_raw[5] > RC_SWITCH_ON);
    if (zeroSwitch && !rcPrevZero) {
      targetAngle     = 0.0f;
      target_velocity = 0.0f;
      trim_bias       = 0.0f;
      encoderLeft = 0; encoderRight = 0;
      prevEncoderLeft = 0; prevEncoderRight = 0;
      Serial3.println("RC:TARGET_ZEROED");
    }
    rcPrevZero = zeroSwitch;
  }

  // Ch8 = targetAngle +/- as a RATE, in degrees. Centre holds, deflection slews
  // the balance setpoint -- the same variable the GUI's "Target" slider sets via
  // S<f>, so the two are one control with two inputs. Squared taper for fine
  // control near centre.
  //
  // Integrated against real elapsed time so the slew rate does not depend on
  // the tick rate. A stale gap falls back to the nominal step rather than
  // applying one huge jump.
  // integ_dt is shared by BOTH rate controls (Ch8 target, Ch10 crouch), so it is
  // computed once here in readRC()'s scope rather than inside either block.
  static unsigned long rcLastIntegMs = 0;
  float integ_dt = (rcLastIntegMs == 0) ? (RC_POLL_INTERVAL_MS * 1.0e-3f)
                                        : ((nowMs - rcLastIntegMs) * 1.0e-3f);
  if (integ_dt > 0.5f) integ_dt = RC_POLL_INTERVAL_MS * 1.0e-3f;
  rcLastIntegMs = nowMs;

  int ch8 = rcTriState(rc_raw[7], 7);
  rc_targetAxis = (float)ch8;                 // telemetry mirror, now -1/0/+1
  if (ch8 != 0) {
    targetAngle += (float)ch8 * RC_TARGET_RATE * integ_dt;
    targetAngle  = constrain(targetAngle, -RC_TARGET_LIMIT, RC_TARGET_LIMIT);
  }

  // -- Ch9: LEG SELECTOR (3-position switch) -------------------------------
  // Gates which leg(s) Ch10 moves. Boundaries at a quarter and three quarters
  // of travel, so a switch that does not sit at exactly 1000/1500/2000 still
  // resolves to the right position.
  if (rc_raw[8] > 0) {
    float u = rcUnipolar(rc_raw[8]);
    rc_legSel = (u < 0.25f) ? 0 : ((u < 0.75f) ? 1 : 2);
  }

  // -- Ch10: CROUCH / STRETCH as a RATE, gated by Ch9 ----------------------
  // Centre = hold. Off-centre moves the selected leg(s) at a rate: positive
  // crouches, negative stretches back toward standing.
  //
  // A rate, not a pot->depth map, because startPoseMove() RESTARTS the
  // interpolation every time it is called. Driving it directly from a jittering
  // pot would restart the move on every poll and the legs would never arrive.
  //
  // ── WHY THIS DOES NOT GO THROUGH startPoseMove() ─────────────────────────
  // It used to, and that is why crouch did nothing even after the accumulator
  // was fixed. startPoseMove() RESTARTS an 800 ms smoothstep interpolation from
  // the current foot position every time it is called. Re-issuing it as the
  // accumulator creeps means every move is restarted a small fraction of the way
  // in, and a smoothstep barely moves near f=0:
  //
  //   re-issue every 20 ms  -> f = 20/800  = 0.025 -> s = 0.0018
  //   re-issue every 167 ms -> f = 167/800 = 0.21  -> s = 0.11
  //
  // i.e. between 0.2% and 11% of each commanded step is actually delivered, and
  // the legs never converge. An interpolator exists to turn a STEP into smooth
  // motion; a rate command is already smooth motion, so feeding one into the
  // other is the bug. The RC path therefore writes the foot position directly
  // and cancels any interpolation in flight.
  //
  // The servos still see a rate-limited stream, not a flood: cur_* is updated
  // here at up to 100 Hz but ax12SyncWriteGoals() runs from the hold-pose path
  // at 10 Hz, so at 3 mm/sec the legs receive 0.3 mm steps.
  static float lastIssuedCrouchL = 0.0f;
  static float lastIssuedCrouchR = 0.0f;

  int ch10 = rcTriState(rc_raw[9], 9);
  rc_crouchAxis = (float)ch10;                // telemetry mirror, now -1/0/+1
  if (ch10 != 0) {
    float step = (float)ch10 * RC_CROUCH_RATE * integ_dt;

    if (rc_legSel == 0)      { crouchOffsetL += step; crouchOffsetR += step; }
    else if (rc_legSel == 1) { crouchOffsetL += step; }
    else                     { crouchOffsetR += step; }
    // Clamped independently, so driving one leg to its limit does not eat the
    // other's remaining travel.
    crouchOffsetL = constrain(crouchOffsetL, 0.0f, RC_MAX_CROUCH);
    crouchOffsetR = constrain(crouchOffsetR, 0.0f, RC_MAX_CROUCH);
  }

  // Push whenever the commanded pose has actually changed -- including the tick
  // after the switch is released, so the last partial millimetre is not left
  // stranded. 0.01 mm is far below the ~0.3 mm the AX-12's position resolution
  // can express, so this cannot chatter.
  if (poseMode == MODE_CROUCH &&
      (fabsf(crouchOffsetL - lastIssuedCrouchL) > 0.01f ||
       fabsf(crouchOffsetR - lastIssuedCrouchR) > 0.01f)) {
    lastIssuedCrouchL = crouchOffsetL;
    lastIssuedCrouchR = crouchOffsetR;
    // NO INTERPOLATION, and no waiting for the 10 Hz hold-pose writer either:
    // the foot position is written straight through and the goals go on the bus
    // on this same tick, so the legs track the commanded mm/sec with nothing
    // filtering it.
    //
    // EXPECT POSSIBLE STEPPING. At 7.5 mm/sec on a 10 ms tick this is 0.075 mm
    // per write, well under the AX-12's own position resolution (~0.29 deg), so
    // consecutive writes will often land on the same count and the motion can
    // look stepped rather than smooth. If it does, the fix is a slower rate or
    // a small first-order filter on cur_y -- NOT the trajectory interpolator,
    // which structurally cannot chase a setpoint that keeps moving (see above).
    //
    // THE BUS IS NOT TOUCHED HERE. readRC() runs at the top of loop(), before
    // the "exactly one Serial2 transaction per tick" arbiter below. An earlier
    // version called ax12SyncWriteGoals() right here, guarded on POLL_IDLE.
    // That guard was useless: it tests whether a READ is in flight, not whether
    // the arbiter is about to WRITE, so the tick ended up carrying two
    // transactions -- this one plus applySettingsTask() or the hold-pose write.
    // On a half-duplex 1 Mbaud bus the second packet overlapped the first (or
    // its echo), the poll state machine's fixed 8-echo/10-reply byte accounting
    // shifted, and every later read mis-parsed. Logged result: all four servos
    // healthy for 95 s, then fail counts climbing 3 s into the first crouch and
    // never recovering -- the robot crouched and could not stand back up.
    //
    // So: update the pose, flag it, and let the arbiter do the write in its own
    // slot. Costs a few hundred microseconds of latency, i.e. 0.0006 mm at
    // 7.5 mm/s -- four orders of magnitude below AX-12 resolution.
    moveActive = false;               // cancel any interpolation in flight
    computeDesiredFoot(cur_x, cur_y); // the rate IS the trajectory
    solveGoalsFor(cur_x, cur_y);      // cache counts; the arbiter sends them
    crouchDirty = true;
  }

  // ── Ch7: IMU calibrate (momentary, rising edge) ──────────────────────────
  // Guarded on !motorsEnabled: startCalibration() holds the motors off for its
  // ~1 s sampling window, so triggering it while balancing drops the robot.
  // Ch5 = IMU calibrate (REMAPPED from Ch7, which is now ARM).
  if (rc_raw[4] > 0) {
    bool calSwitch = (rc_raw[4] > RC_SWITCH_ON);
    if (calSwitch && !rcPrevCal && !motorsEnabled) startCalibration();
    rcPrevCal = calSwitch;
  }

  // ── Ch8: integral kill (2-position, active HIGH) ─────────────────────────
  // Active-HIGH so an unbound channel (which reads ~1000, i.e. LOW) leaves the
  // integrator running normally rather than silently disabling it.
  // (The old Ch8 "integral kill" switch is removed: Ch8 is now the target-rate
  // pot. Leaving it would have let a pot above mid-travel continuously zero
  // both integrators -- which, with the pot resting at ~1614 as logged, would
  // have held the balance integral at 0 permanently.)
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
  delay(2000);                 // let AX-12 servos stabilise before UART traffic

  Serial3.begin(115200);       // 3DR radio
  Serial2.begin(1000000);      // AX-12 bus

  // FlySky iBUS on Serial1 (PA10 RX). IBusBM::begin() calls Serial1.begin()
  // itself at 115200 8N1, so do NOT begin Serial1 beforehand.
  //
  // IBUSBM_NOTIMER is the whole point: the default (timerid=0) would grab TIM1
  // and call ibus.loop() from a 1 ms interrupt, where its sensor branch can sit
  // in delayMicroseconds(100) and blocking writes. That is unbounded jitter in
  // a 100 Hz control loop — see the file header. With NOTIMER we own the
  // servicing, and readRC() does it under a microsecond budget.
  ibus.begin(Serial1, IBUSBM_NOTIMER);

  initAX12Legs();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), countRight, RISING);

  setupMPU();
  // Snap legs to standing pose and broadcast it via SYNC_WRITE before the
  // balance loop starts. This means cur_* is valid from the very first tick.
  snapPose();
  ax12SyncWriteGoals();

  lastTime     = micros();
  lastPollTime = millis();

  Serial3.println("BOOT:OK RC");
}

// ── MAIN LOOP (100 Hz) ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastTime < 10000) return;   // enforce 100 Hz
  float dt = (now - lastTime) * 1.0e-6f;
  lastTime = now;

  // ── RC INPUT (self-decimating to RC_POLL_INTERVAL_MS, budgeted) ─────────
  // First in the tick so the stick values the control code below reads are the
  // freshest available, and so a link loss disarms BEFORE the PID runs rather
  // than one tick later. On ticks where the poll is not due this returns after
  // a compare — see readRC().
  readRC();

  // ── IMU ────────────────────────────────────────────────────────────────
  readIMU(dt);
  calibrationTask();   // accumulates this tick's sample when a cal is running

  // ── SAFETY CUTOFF ───────────────────────────────────────────────────────
  if (fabsf(pitch) > maxSafeTilt && motorsEnabled) {
    motorsEnabled = false;
    safetyLatched = true;
    integral      = 0.0f;
    setMotors(0, 0);
    Serial3.println("SAFETY:CUTOFF");
  }

  // ── ENCODER → VELOCITY (telemetry display only) ─────────────────────────
  long encL = encoderLeft;
  long encR = encoderRight;
  float deltaL =  (float)(encL - prevEncoderLeft);
  float deltaR = -(float)(encR - prevEncoderRight);   // mirror-mount normalise
  float vel_raw = ((deltaL + deltaR) * 0.5f) / dt;
  vel_current = vel_alpha * vel_current + (1.0f - vel_alpha) * vel_raw;

  // YAW RATE from the SAME two deltas, opposite combination. The sum above is
  // translation with rotation cancelled; this difference is rotation with
  // translation cancelled. Same EMA, because it is the same integer-quantised
  // noise: without it a 1-count/tick jitter is +-100 counts/sec of phantom yaw
  // at 100 Hz, and Kp_yaw would pump that straight into the motors.
  //
  // (deltaR - deltaL), NOT (deltaL - deltaR). This orientation is MEASURED, not
  // derived: in serial_COM13_20260915_235020.log the commanded yaw (YT) and the
  // measured yaw (YW) came out OPPOSITE in sign on 89 of 95 turning frames
  // (97% on firm turns) -- e.g. YT +504 read back as YW -1050. With the
  // subtraction the other way round, yaw_error = yaw_target - yaw_current
  // evaluated to target PLUS the disturbance, so the "correction" reinforced
  // the rotation: positive feedback. That is what pinned steer at full
  // authority for up to 11.3 s and, whenever |steer| exceeded |PID_OUT|, made
  // left=-out+steer and right=-out-steer diverge in sign so the robot spun in
  // place instead of driving straight.
  //
  // The tempting derivation -- "+steer speeds the left wheel, so
  // deltaL > deltaR" -- is locally true at every step but silently assumes the
  // left motor's rotation reaches the left encoder positive after the mirror
  // normalisation above. On this machine it does not. The robot is the
  // authority on its own wiring, so the measurement is matched to the command
  // convention here rather than inferred.
  float yaw_raw = (deltaR - deltaL) / dt;
  yaw_current = vel_alpha * yaw_current + (1.0f - vel_alpha) * yaw_raw;

  prevEncoderLeft  = encL;
  prevEncoderRight = encR;

  // Leg torque is intentionally independent of motorsEnabled (the drive-wheel
  // arm state): pollLegServosTask() below keeps torque enabled and re-asserts
  // legServos[].goalPos every 20 ms/servo regardless, so the CR<f> crouch bar
  // can stand/crouch the legs on the bench with the wheel motors disarmed.

  // ══════════════════════════════════════════════════════════════════════════
  // THE SINGLE BALANCE PID — the only control loop
  // ══════════════════════════════════════════════════════════════════════════
  // `calibrating` holds the motors off for the ~1 s sampling window. The old
  // blocking calibrateIMU() froze the whole loop, so the motors could not act
  // on a half-settled pitch; now that the loop keeps running, say so explicitly.
  float output = 0.0f;
  if (!motorsEnabled || calibrating) {
    integral  = 0.0f;
    trim_bias = 0.0f;
    lean_cmd  = 0.0f;
    setMotors(0, 0);
  } else {
    // ── OUTER LOOP (slow, ~0.5 Hz): velocity error -> lean angle ───────────
    // target_velocity is 0 when the drive stick is centred, which makes this
    // identical to mcu_balance_fusion_wireless: any steady vel_current is then
    // unwanted motion and gets leaned out. Push the stick and the SAME loop
    // now chases a non-zero velocity instead, leaning the robot to accelerate.
    // The P term reacts immediately; the I term learns the standing
    // CoM/mounting offset that used to need a hand re-trim. Both output
    // DEGREES OF LEAN, which is why a drive command belongs here and not on
    // targetAngle: one authority over the setpoint, not two fighting.
    if (autoTrimEnabled) {
      float vel_error = target_velocity - vel_current;  // counts/s
      // ── INTEGRATOR FREEZE WHILE DRIVING ────────────────────────────────
      // trim_bias exists to learn the STANDING balance point (the CoM/mounting
      // offset that used to need a hand re-trim). It is an integrator with a
      // slow gain and a +-MAX_TRIM_BIAS clamp, both sized for the case this
      // firmware originally had: target_velocity always 0, so any vel_error was
      // a small drift to null out.
      //
      // A drive command breaks that assumption. The robot never tracks the
      // commanded velocity exactly, so holding the stick leaves a LARGE
      // sustained vel_error, and the integrator accumulates it without bound
      // until it hits the clamp. Measured with the bench gains (Kp_vel 0.0115,
      // Ki_trim 0.0020, RV150): at 40% velocity tracking trim_bias reaches the
      // full +6 deg in about 20 s. Two things then go wrong:
      //   1. lean_cmd saturates, so Kp_vel's push-rejection term is completely
      //      masked and the robot stops responding to disturbances;
      //   2. on stick release the decay is Ki_trim*vel_error = -0.3 deg/s, so
      //      it takes ~20 SECONDS to unwind +6 deg. The operator centres the
      //      stick and the robot keeps leaning and driving away from them.
      //
      // Freezing the integrator while a drive command is active is the standard
      // cascade fix. Kp_vel still supplies the drive lean -- it is proportional,
      // has no memory, and cannot wind up -- while trim_bias holds the standing
      // trim it already learned instead of accumulating drive error into it.
      // Stick release then settles on the Kp_vel timescale, i.e. immediately.
      bool rc_driving = (fabsf(target_velocity) > 1.0f);
      if (!rc_driving) {
        trim_bias += Ki_trim * vel_error * dt;
        trim_bias  = constrain(trim_bias, -MAX_TRIM_BIAS, MAX_TRIM_BIAS);
      }
      lean_cmd   = (Kp_vel * vel_error) + trim_bias;
      lean_cmd   = constrain(lean_cmd, -MAX_LEAN_CMD, MAX_LEAN_CMD);
    } else {
      // Pure single-loop mode (TE0). The velocity loop is the ONLY thing that
      // can turn a drive command into motion, so with it off the drive stick is
      // inert by construction — deliberately, so the A/B comparison against the
      // parent variant stays honest. Steering still works: it is a PWM trim
      // applied after the PID, not a velocity command.
      lean_cmd = 0.0f;
    }

    // ── INNER LOOP (fast, ~10 Hz): hold the commanded lean ─────────────────
    // Unchanged from the validated single-loop tune EXCEPT its setpoint, which
    // is now driven by the outer loop instead of being a fixed trim.
    float error = (targetAngle + lean_cmd) - pitch;

    integral += error * dt;
    if (Ki > 1e-6f) {                       // anti-windup: clamp integrator STATE
      float intLimit = MAX_INTEGRAL_PWM / Ki;
      integral = constrain(integral, -intLimit, intLimit);
    } else {
      integral = 0.0f;                       // Ki off: never bank a latent kick
    }

    float derivative = -gyroRate;            // derivative on measurement
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // ── STEERING — differential trim, applied AFTER the balance PID ────────
    // This is the only correct place for it. The balance PID owns the COMMON
    // component of the two motor outputs (that is what holds pitch); steering
    // is the DIFFERENTIAL component, which pitch dynamics are blind to. Adding
    // it to one and subtracting from the other therefore turns the robot
    // without perturbing the balance loop's authority at all.
    //
    // Sign convention matches the L/R spin cases in blue_pill_dev/motor_testing
    // and mcu_pos_wireless: positive steer = turn right = left wheel forward.
    // NEGATED. The sign convention below is right for the motor wiring, but
    // this machine's Ch4 reads the opposite way round: stick left commanded a
    // right turn. Inverting the axis here (rather than swapping the two _pwm
    // lines) keeps the wiring convention documented above intact and puts the
    // correction at the one place the transmitter's polarity enters.
    // ── OPEN-LOOP PROPORTIONAL STEERING ───────────────────────────────────
    // Deliberately NOT a closed yaw-rate loop. There was one; it was removed
    // after measuring it on hardware.
    //
    // The loop worked once its sign was fixed (YT/YW agreement 6% -> 99%,
    // corr -0.86 -> +0.88, idle |YW| 474 -> 53). It was dropped because it
    // could not earn its keep: turn tracking sat at YW/YT = 0.37, and it had no
    // way to do better, since its own authority caps -- feed-forward 0.55*80 =
    // 44 counts plus a 20-count P budget -- ceiling the command at 64 of 80
    // while it averaged 47.6. The P term never reached its cap on a single turn
    // frame, so nothing was saturating; the command was just small. Those caps
    // existed only to restrain the loop back when its feedback was POSITIVE,
    // and with the sign right they were throttling a healthy control.
    //
    // A yaw-RATE loop also cannot hold a heading: it nulls rate, not
    // accumulated angle, so it never returns the robot to where it pointed.
    // That is a heading (integrating) loop's job, and nothing here asks for one.
    //
    // Open loop is honest here because the drive is symmetric: measured wheel
    // efficiency is 0.252 (L) vs 0.259 (R) counts per PWM count, a ratio of
    // 1.03, so there is no standing bias for feedback to trim out.
    //
    // rc_steer is already expo-shaped and sign-corrected for this transmitter
    // in readRC(); yaw_target carries that same shaped value scaled to
    // counts/sec, so dividing it back out recovers the -1..+1 command without
    // duplicating the curve.
    float steer = 0.0f;
    if (rc_arm && RC_MAX_YAW > 1.0f) {
      steer = (yaw_target / RC_MAX_YAW) * RC_MAX_STEER;
      steer = constrain(steer, -RC_MAX_STEER, RC_MAX_STEER);
    }

    float left_pwm  = -output + steer;
    float right_pwm = -output - steer;

    setMotors((int)constrain(left_pwm,  -255.0f, 255.0f),
              (int)constrain(right_pwm, -255.0f, 255.0f));
  }


  // ── RX EVERY TICK, SERVO POLL AT 50 Hz ──────────────────────────────────
  // Uplink commands are latency-critical and the downlink was starving them,
  // so RX now runs on every 10 ms tick instead of every other one. The servo
  // health poll keeps its old 50 Hz slot — it is a slow, purely cosmetic read
  // and pollLegServosTask() already rate-limits itself to POLL_INTERVAL_MS.
  handleTelemetryRX();

  // ── SETTINGS PUSH (gated on POLL_IDLE to avoid collision with reply bytes) ──
  if (pollState == POLL_IDLE) applySettingsTask();

  // ── 3-MODE BUS ARBITRATION — exactly one Serial2 transaction per tick ────────
  // Mode A: move active → SYNC_WRITE at full 100 Hz, health suspended.
  // Mode B: idle, settings dirty → applySettingsTask() (already ran above).
  // Mode C: idle, no settings → hold pose at 10 Hz + health poll at 50 Hz.
  if (moveActive) {
    // Abandon any in-flight health read so it does not collide with move writes.
    if (pollState == POLL_WAITING) {
      ax12DrainRx();
      pollState    = POLL_IDLE;
      lastPollTime = millis();
    }
    updateMoveTask();   // solveGoalsFor() + ax12SyncWriteGoals() inside
  } else {
    // Hold pose at 10 Hz — re-asserts goals to fight servo drift, but at a
    // cadence slow enough that the health poll can run in the gaps.
    //
    // crouchDirty jumps the 10 Hz timer: an RC crouch is actively moving the
    // legs and should reach the servos on the next tick, not up to 100 ms
    // later. It still goes out HERE, inside the arbiter, so the tick carries
    // exactly one Serial2 transaction — the invariant this whole block exists
    // to enforce, and the one the old in-readRC() write violated.
    static unsigned long lastHoldUs = 0;
    bool wantHold = crouchDirty || (now - lastHoldUs >= 100000);
    if (g_torqueOn && pollState == POLL_IDLE && wantHold) {
      lastHoldUs  = now;
      crouchDirty = false;
      ax12SyncWriteGoals();
    } else {
      // Only poll when the write slot was not used, so the two can never share
      // a tick. (Previously the poll ran on its own alternating cycle, which
      // was safe only because the write was strictly 10 Hz.)
      static bool isReadCycle = false;
      isReadCycle = !isReadCycle;
      if (isReadCycle) pollLegServosTask();
    }
  }

  // ── TELEMETRY @ 10 Hz ────────────────────────────────────────────────────
  // Halved from 20 Hz. The 3DR/SiK link is half-duplex with a TDM air protocol:
  // each end only gets ~half the airtime, so the old ~2.7 kB/s downlink left no
  // window for the ground unit to transmit, and commands sat in the radio's
  // buffer for seconds. (latency_test.py measured 97% downlink loss from a
  // single PING every 2 s — proof the air link had zero headroom.)
  if (now - lastPrintTime >= 100000) {
    lastPrintTime = now;

    // ── Balance telemetry (10 Hz) ────────────────────────────────────────────
    // Extended with leg subsystem state fields so the GUI can display both
    // the balance loop AND the AX-12 trajectory/mode in one pass.
    // 384, not 256. Adding the yaw fields pushes the WORST-CASE frame (large
    // negative encoder counts and a saturated yaw rate at the same time) past
    // 256, and snprintf truncates SILENTLY from the tail -- which is exactly
    // where newly-added fields land. An undersized buffer therefore does not
    // error, it just makes the new telemetry vanish once the numbers get big.
    // Matches SERIAL_TX_BUFFER_SIZE=384 in platformio.ini, so a full frame
    // still clears the availableForWrite() guard below in one go.
    char line[384];
    int n = snprintf(line, sizeof(line),
      "PITCH:%.2f,PID_OUT:%.2f,INT:%.4f,EL:%ld,ER:%ld,VEL:%.1f,"
      "MOT:%d,TILT:%.1f,TRIM:%.3f,ATE:%d,LATCH:%d,"
      "TORQ:%d,CRL:%.1f,CRR:%.1f,FX1:%.2f,FY1:%.2f,FX2:%.2f,FY2:%.2f,IK1:%d,IK2:%d,MOVE:%d,"
      "RCL:%d,RCE:%d,RCA:%d,RCD:%.2f,RCS:%.2f,TVEL:%.1f,RCT:%.2f,RC8:%u,"
      "YW:%.0f,YT:%.0f\n",
      pitch, output, integral, encL, encR, vel_current,
      (int)motorsEnabled, maxSafeTilt, lean_cmd,
      (int)autoTrimEnabled, (int)safetyLatched,
      (int)g_torqueOn, crouchOffsetL, crouchOffsetR,
      cur_x[0], cur_y[0], cur_x[1], cur_y[1],
      (int)ikValid[0], (int)ikValid[1], (int)moveActive,
      (int)rcLinkOK, (int)rcEnabled, (int)rc_arm,
      rc_drive, rc_steer, target_velocity,
      targetAngle, (unsigned)rc_raw[7],
      yaw_current, yaw_target);
    if (n > 0 && n < (int)sizeof(line) && Serial3.availableForWrite() >= n)
      Serial3.write((uint8_t*)line, n);

    // ── AX-12 state line (every 5 ticks = 500 ms) — compliance, speed, mode ─
    // Emitted on alternating frames so it never competes with the PITCH line.
    static uint8_t ax12StateDiv = 0;
    if (++ax12StateDiv >= 5) {
      ax12StateDiv = 0;
      int m = snprintf(line, sizeof(line),
        "AX12:MODE:%u,TQ:%u,TL:%u,CM:%u,CS:%u,MS:%u,MT:%u\n",
        (unsigned)poseMode, (unsigned)(g_torqueOn?1:0),
        (unsigned)g_torqueLimit, (unsigned)g_compMargin,
        (unsigned)g_compSlope,  (unsigned)g_movingSpeed,
        (unsigned)moveTimeMs);
      if (m > 0 && m < (int)sizeof(line) && Serial3.availableForWrite() >= m)
        Serial3.write((uint8_t*)line, m);
    }

    // ── RAW RC CHANNELS at 2 Hz ────────────────────────────────────────────
    // Emitted so the operator can confirm which physical stick drives which
    // channel WITHOUT inferring it from robot behaviour — the single most
    // common RC bring-up problem. Sent on its own line and on a different
    // divisor to the AX12 line so the two never land in the same tick and
    // compete for the radio TX buffer. 2 Hz is plenty to watch a switch flip.
    static uint8_t rcStateDiv = 0;
    if (++rcStateDiv >= 5) {
      rcStateDiv = 0;
      int r = snprintf(line, sizeof(line),
        "RC:%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,LINK:%d\n",
        (unsigned)rc_raw[0], (unsigned)rc_raw[1], (unsigned)rc_raw[2],
        (unsigned)rc_raw[3], (unsigned)rc_raw[4], (unsigned)rc_raw[5],
        (unsigned)rc_raw[6], (unsigned)rc_raw[7], (unsigned)rc_raw[8],
        (unsigned)rc_raw[9], (int)rcLinkOK);
      if (r > 0 && r < (int)sizeof(line) && Serial3.availableForWrite() >= r)
        Serial3.write((uint8_t*)line, r);
    }

    // ── Servo health at 1 Hz (round-robin one servo per second) ─────────────
    static uint8_t healthIdx   = 0;
    static unsigned long lastHealth = 0;
    if (now - lastHealth >= 1000000) {
      lastHealth = now;
      ServoState &h = legServos[healthIdx];
      int k = snprintf(line, sizeof(line), "SRV:%u,%u,%.1f,%u,%u\n",
                       (unsigned)h.id, (unsigned)h.temp, h.loadPct,
                       (unsigned)h.err, (unsigned)h.failCount);
      if (k > 0 && k < (int)sizeof(line) && Serial3.availableForWrite() >= k)
        Serial3.write((uint8_t*)line, k);
      healthIdx = (healthIdx + 1) % 4;
    }
  }

}
