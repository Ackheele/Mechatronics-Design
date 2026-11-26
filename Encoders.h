#pragma once
#include <Arduino.h>
#include <util/atomic.h>
#include "Pins.h"
#include "Motor.h"
#include "IMU.h"

// ============================================================
//  Encoders.h - REAR wheel encoders (N20 motors, 210:1 gearbox)
//  RL = D32 (A) / D33 (B)      RR = D36 (A) / D37 (B)
// ============================================================
// D30-D37 have no interrupts on the Mega, so a 25 kHz timer interrupt
// (Timer2) samples them in the background and decodes the quadrature
// signal. They are all on PORTC, so one read of PINC grabs both encoders.
// Counting keeps going while the rest of the code runs (IMU, ultrasonics...).
//
// Timer2 is taken: don't use tone(), or analogWrite() on pins 9/10
// (the servos on 9/10 are fine - Servo uses Timer5 on the Mega).

static_assert(ENC_RL_A_PIN == 32 && ENC_RL_B_PIN == 33 && ENC_RR_A_PIN == 36 && ENC_RR_B_PIN == 37,
              "Encoders.h reads PORTC bits directly - update the ISR if you rewire the encoders");

// ---- Settings ----
const float ENC_PPR        = 7.0;     // pulses per MOTOR-shaft turn, one channel. 7 is typical for N20
                                      // magnetic encoders, but check it with encoderWatch() (W command)
const float GEAR_RATIO     = 210.0;
const float COUNTS_PER_REV = ENC_PPR * 4.0 * GEAR_RATIO;   // counts per WHEEL turn (= 5880)

// If driving a side FORWARD makes its count go DOWN, flip that side to -1.
const int ENC_RL_SIGN = -1;
const int ENC_RR_SIGN = 1;

const float ENC_TEST_POWER = 0.40;    // spinWheel() speed
const float ENC_MIN_POWER  = 0.15;    // lowest power that still turns the wheel. If it stalls just
                                      // before the target ("no encoder pulses"), raise this a little
const float ENC_SLOW_TURNS = 0.40;    // start ramping power down this many wheel turns before the target
const long  ENC_TOLERANCE  = 30;      // counts (~2 deg) - close enough, no correction needed
const int   ENC_MAX_FIXES  = 4;       // max correction nudges after the first stop
const unsigned long ENC_NO_PULSE_MS = 1500;  // motor on but no counts for this long -> stop

enum Side { LEFT_SIDE, RIGHT_SIDE };

// ---- State (written by the interrupt) ----
volatile long encRL = 0, encRR = 0;
volatile unsigned long encErrRL = 0, encErrRR = 0;   // impossible jumps = a step was missed
uint8_t encPrevRL = 0, encPrevRR = 0;

// Quadrature table. index = (old << 2) | new, state = (A << 1) | B.
// 2 = both pins changed between samples -> a step was missed.
const int8_t QDEC[16] = { 0, -1, +1,  2,
                         +1,  0,  2, -1,
                         -1,  2,  0, +1,
                          2, +1, -1,  0 };

ISR(TIMER2_COMPA_vect) {
  uint8_t p  = PINC;
  uint8_t rl = (p >> 4) & 0x03;   // PC5 = D32 = A, PC4 = D33 = B
  uint8_t rr =  p       & 0x03;   // PC1 = D36 = A, PC0 = D37 = B

  int8_t d = QDEC[(encPrevRL << 2) | rl];
  if (d == 2) encErrRL++; else encRL += d;
  encPrevRL = rl;

  d = QDEC[(encPrevRR << 2) | rr];
  if (d == 2) encErrRR++; else encRR += d;
  encPrevRR = rr;
}

// Call once in setup().
void encodersBegin() {
  pinMode(ENC_RL_A_PIN, INPUT_PULLUP);
  pinMode(ENC_RL_B_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_A_PIN, INPUT_PULLUP);
  pinMode(ENC_RR_B_PIN, INPUT_PULLUP);
  // Front encoders (30/31, 34/35) are not used.

  uint8_t p = PINC;
  encPrevRL = (p >> 4) & 0x03;
  encPrevRR =  p       & 0x03;

  noInterrupts();
  TCCR2A = (1 << WGM21);    // CTC mode
  TCCR2B = (1 << CS21);     // 16 MHz / 8 = 2 MHz
  OCR2A  = 79;              // 2 MHz / 80 = 25 kHz
  TCNT2  = 0;
  TIMSK2 = (1 << OCIE2A);
  interrupts();
}

// ---- Reading ----
long encRead(Side s) {
  long v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = (s == LEFT_SIDE) ? encRL : encRR; }
  return v * ((s == LEFT_SIDE) ? ENC_RL_SIGN : ENC_RR_SIGN);
}
unsigned long encErrors(Side s) {
  unsigned long v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = (s == LEFT_SIDE) ? encErrRL : encErrRR; }
  return v;
}
float encTurns(Side s) { return encRead(s) / COUNTS_PER_REV; }
void encReset() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { encRL = encRR = 0; encErrRL = encErrRR = 0; }
}

// ---- Test: spin one side a number of wheel turns, then stop ----
// spinWheel(RIGHT_SIDE, 1);     one full turn forward on the right
// spinWheel(LEFT_SIDE, -2.5);   two and a half turns backward on the left
// Each Cytron channel drives BOTH motors on a side, so the front wheel on that
// side spins too - only the REAR encoder is counted. Put the robot on a box.
//
// How it stops on target:
//   1. full power until ENC_SLOW_TURNS before the target
//   2. power ramps down smoothly to ENC_MIN_POWER as it gets close (little coasting left)
//   3. after it stops, if it is still more than ENC_TOLERANCE off (short OR past),
//      it nudges forward/back at ENC_MIN_POWER, up to ENC_MAX_FIXES times
// Returns false if it had to stop early (no pulses / counting the wrong way).

// Waits for the wheel to stop moving (or maxMs), keeps the IMU alive.
void encWaitStill(Side side, unsigned long maxMs = 600) {
  unsigned long t = millis();
  long last = encRead(side);
  unsigned long stillSince = millis();
  while (millis() - t < maxMs) {
    updateHeading();
    long now = encRead(side);
    if (now != last) { last = now; stillSince = millis(); }
    if (millis() - stillSince > 80) break;     // no counts for 80 ms = stopped
  }
}

bool spinWheel(Side side, float turns, float power = ENC_TEST_POWER) {
  CytronMD &motor  = (side == LEFT_SIDE) ? leftside : rightside;
  const char* name = (side == LEFT_SIDE) ? "LEFT" : "RIGHT";

  long target = (long)(fabs(turns) * COUNTS_PER_REV + 0.5);
  if (target == 0) return true;
  float dir   = (turns >= 0) ? 1.0 : -1.0;
  long slowAt = (long)(ENC_SLOW_TURNS * COUNTS_PER_REV);
  power = max(fabs(power), ENC_MIN_POWER);

  robotStop();
  pauseWithHeading(200);

  long start = encRead(side);
  unsigned long errStart = encErrors(side);
  long lastCount = start;
  unsigned long lastMoveMs = millis(), lastPrint = 0;
  bool ok = true;

  Serial.print("Spinning "); Serial.print(name); Serial.print(" rear ");
  Serial.print(turns, 2); Serial.print(" turns = "); Serial.print(target); Serial.println(" counts");

  // ---- 1 + 2: drive with a smooth slow-down ----
  while (true) {
    updateHeading();
    long now  = encRead(side);
    long done = (now - start) * (long)dir;      // counts moved in the direction we asked for
    long left = target - done;
    if (left <= 0) break;

    if (now != lastCount) { lastCount = now; lastMoveMs = millis(); }
    if (millis() - lastMoveMs > ENC_NO_PULSE_MS) {
      Serial.println("  STOP: no encoder pulses - check encoder power/wiring, or raise ENC_MIN_POWER (motor stalled)");
      ok = false; break;
    }
    if (done < -(long)(0.05 * COUNTS_PER_REV)) {
      Serial.print("  STOP: counting the WRONG way - flip ENC_");
      Serial.print(side == LEFT_SIDE ? "RL" : "RR"); Serial.println("_SIGN in Encoders.h");
      ok = false; break;
    }

    float p = power;
    if (left < slowAt) p = ENC_MIN_POWER + (power - ENC_MIN_POWER) * (float)left / slowAt;
    motorMove(motor, p * dir);

    if (millis() - lastPrint > 250) {
      lastPrint = millis();
      Serial.print("  "); Serial.print(done); Serial.print(" / "); Serial.print(target);
      Serial.print("   power "); Serial.println(p, 2);
    }
  }
  robotStop();
  encWaitStill(side);

  long firstStop = (encRead(side) - start) * (long)dir;
  Serial.print("  First stop: "); Serial.print(firstStop - target); Serial.println(" counts off");

  // ---- 3: small corrections (either direction) ----
  for (int fix = 0; ok && fix < ENC_MAX_FIXES; fix++) {
    long err = target - (encRead(side) - start) * (long)dir;   // + = short, - = past
    if (labs(err) <= ENC_TOLERANCE) break;

    float d = (err > 0) ? dir : -dir;
    unsigned long t0 = millis();
    motorMove(motor, ENC_MIN_POWER * d);
    while (millis() - t0 < 1000) {
      updateHeading();
      long e = target - (encRead(side) - start) * (long)dir;
      if ((err > 0 && e <= 0) || (err < 0 && e >= 0)) break;   // crossed the target
    }
    robotStop();
    encWaitStill(side);
  }

  long finalDone = (encRead(side) - start) * (long)dir;
  long over = finalDone - target;
  Serial.print("  Done: "); Serial.print(finalDone); Serial.print(" counts (");
  Serial.print(finalDone / COUNTS_PER_REV, 3); Serial.print(" turns), error ");
  Serial.print(over); Serial.print(" counts = ");
  Serial.print(over * 360.0 / COUNTS_PER_REV, 1); Serial.println(" deg");

  unsigned long missed = encErrors(side) - errStart;
  if (missed > 0) {
    Serial.print("  WARNING: "); Serial.print(missed);
    Serial.println(" missed steps - encoder too fast for 25 kHz sampling or noisy wires; lower ENC_TEST_POWER");
  }
  return ok;
}

// ---- Calibration: motors off, prints both rear counts until a key is sent ----
// Mark the wheel, turn it by hand exactly ONE turn: the count should change by
// about COUNTS_PER_REV (5880). If it changes by e.g. 2520, ENC_PPR is 3; 10080 -> 12.
void encoderWatch() {
  robotStop();
  while (Serial.available()) Serial.read();
  Serial.println("Watching encoders (motors off). Turn a wheel by hand. Send any key to stop.");
  while (!Serial.available()) {
    Serial.print("LEFT: ");   Serial.print(encRead(LEFT_SIDE));
    Serial.print(" ("); Serial.print(encTurns(LEFT_SIDE), 3); Serial.print(" turns)");
    Serial.print("  |  RIGHT: "); Serial.print(encRead(RIGHT_SIDE));
    Serial.print(" ("); Serial.print(encTurns(RIGHT_SIDE), 3); Serial.print(" turns)");
    Serial.print("  |  missed L/R: "); Serial.print(encErrors(LEFT_SIDE));
    Serial.print("/"); Serial.println(encErrors(RIGHT_SIDE));
    pauseWithHeading(200);
  }
  while (Serial.available()) Serial.read();
  Serial.println("Stopped watching.");
}

// ======================================================
//  Drive a distance in cm using both rear encoders
// ======================================================
#include "Drive.h"   // heading hold (KP_DRIVE, goalHeading...)

const float WHEEL_DIAMETER_CM = 2.5 * 2.54;                    // 2.5" wheels = 6.35 cm
const float WHEEL_CIRC_CM     = PI * WHEEL_DIAMETER_CM;        // 19.95 cm per wheel turn
const float DIST_SCALE        = 1.105;  // calibrated: F 20 went 7 1/8" (18.10 cm) -> 20 / 18.10
                                        // to re-check: drive 100 cm, measure real X cm, DIST_SCALE = old * 100 / X
const float COUNTS_PER_CM     = COUNTS_PER_REV / WHEEL_CIRC_CM * DIST_SCALE;   // ~295

const float DRIVE_CM_MIN_POWER = 0.22;  // lowest power that still moves the robot ON THE FLOOR
                                        // (needs more than in the air - raise if it stalls near the end)
const float DRIVE_CM_SLOW_CM   = 8.0;   // start ramping down this far before the target
const long  DRIVE_CM_TOL       = 60;    // counts (~2 mm) - close enough
const int   DRIVE_CM_MAX_FIXES = 3;

// Average distance (in counts) of both rear wheels since the given start counts.
long encAvgSince(long l0, long r0) {
  return ((encRead(LEFT_SIDE) - l0) + (encRead(RIGHT_SIDE) - r0)) / 2;
}

// Waits until both rear wheels stop turning (or maxMs).
void encWaitBothStill(unsigned long maxMs = 600) {
  unsigned long t = millis(), stillSince = millis();
  long last = encRead(LEFT_SIDE) + encRead(RIGHT_SIDE);
  while (millis() - t < maxMs) {
    updateHeading();
    long now = encRead(LEFT_SIDE) + encRead(RIGHT_SIDE);
    if (now != last) { last = now; stillSince = millis(); }
    if (millis() - stillSince > 80) break;
  }
}

// One drive step toward the target: heading hold on targetHeading + given base power.
void driveStraightStep(float targetHeading, float basePower) {
  float err  = wrapAngle(targetHeading - heading);
  float corr = 0;
  if (fabs(err) > DRIVE_DEADBAND) corr = constrain(err * KP_DRIVE, -MAX_CORR, MAX_CORR);
  motorMove(leftside,  basePower + corr);
  motorMove(rightside, basePower - corr);
}

// Drives straight for `cm` centimetres (negative = backward), then stops.
//   distance = AVERAGE of the two rear encoders
//   straight = gyro heading hold on targetHeading (same as your other drive functions)
// Returns false if it stopped early (no pulses / timeout).
bool driveCmHeading(float targetHeading, float cm, float speed = FORWARD_SPEED) {
  long target = (long)(fabs(cm) * COUNTS_PER_CM + 0.5);
  if (target == 0) return true;
  float dir   = (cm >= 0) ? 1.0 : -1.0;
  long slowAt = (long)(DRIVE_CM_SLOW_CM * COUNTS_PER_CM);
  speed = max(fabs(speed), DRIVE_CM_MIN_POWER);

  robotStop();
  pauseWithHeading(100);

  long l0 = encRead(LEFT_SIDE), r0 = encRead(RIGHT_SIDE);
  long lastAvg = 0;
  unsigned long startMs = millis(), lastMoveMs = millis(), lastPrint = 0;
  bool ok = true;

  Serial.print("Driving "); Serial.print(cm, 1); Serial.print(" cm = ");
  Serial.print(target); Serial.println(" counts (avg of both rear wheels)");

  while (true) {
    updateHeading();
    long done = encAvgSince(l0, r0) * (long)dir;
    long left = target - done;
    if (left <= 0) break;

    if (done != lastAvg) { lastAvg = done; lastMoveMs = millis(); }
    if (millis() - lastMoveMs > ENC_NO_PULSE_MS) {
      Serial.println("  STOP: wheels not turning - stalled? raise DRIVE_CM_MIN_POWER, or check encoders");
      ok = false; break;
    }
    if (millis() - startMs > DRIVE_TIMEOUT_MS) {
      Serial.println("  STOP: timeout"); ok = false; break;
    }

    float p = speed;
    if (left < slowAt) p = DRIVE_CM_MIN_POWER + (speed - DRIVE_CM_MIN_POWER) * (float)left / slowAt;
    driveStraightStep(targetHeading, p * dir);

    if (millis() - lastPrint > 250) {
      lastPrint = millis();
      Serial.print("  "); Serial.print(done / COUNTS_PER_CM, 1); Serial.print(" / ");
      Serial.print(fabs(cm), 1); Serial.print(" cm   heading err ");
      Serial.println(wrapAngle(targetHeading - heading), 1);
    }
  }
  robotStop();
  encWaitBothStill();

  // Small corrections if it coasted past / stopped short
  for (int fix = 0; ok && fix < DRIVE_CM_MAX_FIXES; fix++) {
    long err = target - encAvgSince(l0, r0) * (long)dir;      // + = short, - = past
    if (labs(err) <= DRIVE_CM_TOL) break;
    float d = (err > 0) ? dir : -dir;
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
      updateHeading();
      long e = target - encAvgSince(l0, r0) * (long)dir;
      if ((err > 0 && e <= 0) || (err < 0 && e >= 0)) break;
      driveStraightStep(targetHeading, DRIVE_CM_MIN_POWER * d);
    }
    robotStop();
    encWaitBothStill();
  }

  float lCm = (encRead(LEFT_SIDE)  - l0) / COUNTS_PER_CM;
  float rCm = (encRead(RIGHT_SIDE) - r0) / COUNTS_PER_CM;
  Serial.print("  Done: avg "); Serial.print((lCm + rCm) / 2, 2);
  Serial.print(" cm  (left "); Serial.print(lCm, 2);
  Serial.print(", right "); Serial.print(rCm, 2);
  Serial.print(")  heading err "); Serial.print(wrapAngle(targetHeading - heading), 1);
  Serial.println(" deg");
  return ok;
}

// Same, along goalHeading (works with turnR() / turnL()):
//   driveCm(10);  turnR();  driveCm(25);
bool driveCm(float cm, float speed = FORWARD_SPEED) {
  return driveCmHeading(goalHeading, cm, speed);
}
