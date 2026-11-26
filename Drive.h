#pragma once
#include <Arduino.h>
#include "Motor.h"
#include "IMU.h"
#include "Ultrasonics.h"

// ---- General ----
const long move_del = 1000;            // pause after each move

// ---- Jolt tuning ----
const float JOLT_POWER = 0.6;
const unsigned long JOLT_MS = 40;

// ---- Turn tuning ----
const float MAX_SPEED  = 0.80;
const float MIN_SPEED  = 0.50;
const float KP         = 0.02;
const float TOLERANCE  = 2.5;
const unsigned long SETTLE_MS = 200;
const unsigned long TURN_TIMEOUT_MS = 0;

// ---- Stall recovery ----
const unsigned long STALL_CHECK_MS = 120;
const float STALL_MIN_MOVE = 0.5;
const float BOOST_STEP     = 0.04;
const float MAX_TURN_POWER = 0.8;

// ---- Straight-driving tuning ----
const float FORWARD_SPEED  = 0.50;
const float KP_DRIVE       = 0.03;
const float MAX_CORR       = 0.25;
const float DRIVE_DEADBAND = 1.0;

// ---- Drive-until-obstacle tuning ----
const unsigned long PING_INTERVAL_MS = 70;   // must be > US_SAME_SENSOR_MS (60), or every other
                                              // "confirming" reading is just a cached copy of the last one
const int CONFIRM_READINGS = 2;
const unsigned long DRIVE_TIMEOUT_MS = 15000;
const float SLOW_START_CM   = 35.0;   // begin slowing this far from the stop distance... (cm reading)
const float CREEP_SPEED     = 0.15;   // speed right at the target. MUST be below your drive speed.
                                      // If the robot stalls before reaching the wall, raise this a bit.
const float BRAKE_MARGIN_CM = 2.0;    // stop this much early to cover coast

// ======================================================

// Short equal-power kick. power < 0 = backward.
// stopAfter = false hands off straight into the next move.
void jolt(float power = JOLT_POWER, unsigned long ms = JOLT_MS, bool stopAfter = false) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    motorMove(rightside, power);
    motorMove(leftside,  power);
    updateHeading();
  }
  if (stopAfter) {
    robotStop();
    pauseWithHeading(100);
  }
}

void turnToHeading(float target) {
  unsigned long start = millis();
  unsigned long inTolSince = 0;
  unsigned long lastPrint = 0;

  float boost = 0;
  float lastCheckHeading;
  unsigned long lastCheck = millis();

  updateHeading();
  lastCheckHeading = heading;

  while (TURN_TIMEOUT_MS == 0 || millis() - start < TURN_TIMEOUT_MS) {
    updateHeading();
    float err = wrapAngle(target - heading);

    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print("Turning... heading: ");
      Serial.print(heading, 1);
      Serial.print("  error: ");
      Serial.print(err, 1);
      Serial.print("  boost: ");
      Serial.println(boost, 2);
    }

    if (fabs(err) <= TOLERANCE) {
      robotStop();
      boost = 0;
      if (inTolSince == 0) inTolSince = millis();
      if (millis() - inTolSince >= SETTLE_MS) break;
      lastCheck = millis();
      lastCheckHeading = heading;
      continue;
    }
    inTolSince = 0;

    if (millis() - lastCheck >= STALL_CHECK_MS) {
      if (fabs(heading - lastCheckHeading) < STALL_MIN_MOVE) {
        boost += BOOST_STEP;
      } else {
        boost = max(0.0f, boost - BOOST_STEP);
      }
      lastCheck = millis();
      lastCheckHeading = heading;
    }

    float s = constrain(fabs(err) * KP, MIN_SPEED, MAX_SPEED) + boost;
    s = min(s, MAX_TURN_POWER);
    if (err < 0) s = -s;
    motorMove(leftside,   s);
    motorMove(rightside, -s);
  }
  robotStop();
  pauseWithHeading(move_del);
}

void driveForward(float targetHeading, unsigned long ms = 1000, float manSPEED = 0, int x_time = move_del) {
  unsigned long start = millis();
  unsigned long lastPrint = 0;
  updateHeading();
  float err = wrapAngle(targetHeading - heading);

  bool stopAtHeading = fabs(err) > TOLERANCE;

  while (millis() - start < ms) {
    updateHeading();
    err = wrapAngle(targetHeading - heading);

    if (stopAtHeading && fabs(err) <= TOLERANCE) {
      Serial.println("Reached target heading - stopping");
      break;
    }

    float corr = 0;
    if (fabs(err) > DRIVE_DEADBAND) {
      corr = constrain(err * KP_DRIVE, -MAX_CORR, MAX_CORR);
    }

    if (manSPEED == 100) {
      motorMove(leftside,  1 + corr);
      motorMove(rightside, 1 - corr);
    } else {
      motorMove(leftside,  FORWARD_SPEED + corr);
      motorMove(rightside, FORWARD_SPEED - corr);
    }

    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print("Driving... heading: ");
      Serial.print(heading, 1);
      Serial.print("  error: ");
      Serial.println(err, 1);
    }
  }
  robotStop();
  pauseWithHeading(x_time);
}

void driveUntilDistance(float targetHeading, float stopCm, int index, float manSpeed) {
  unsigned long start = millis();
  unsigned long lastPing = 0;
  unsigned long lastPrint = 0;
  int closeCount = 0;
  float distance = -1;
  float lastValid = 999;
  float speed = manSpeed;
  updateHeading();

  while (true) {
    updateHeading();
    float err = wrapAngle(targetHeading - heading);

    float corr = 0;
    if (fabs(err) > DRIVE_DEADBAND) {
      corr = constrain(err * KP_DRIVE, -MAX_CORR, MAX_CORR);
    }

    motorMove(leftside,  speed + corr);
    motorMove(rightside, speed - corr);

    if (millis() - lastPing >= PING_INTERVAL_MS) {
      lastPing = millis();

      distance = readUltrasonicCm(index);   // ultrasonic: 0 bottom, 1 front-top, 2 right, 3 left

      // No echo (-1) is ignored: it does NOT count as "close" any more.
      // (It used to be turned into a fake 1 cm, which stopped the robot
      //  whenever the sensor missed two pings with something ~20-30 cm ahead.)
      if (distance > 0) {
        lastValid = distance;
      }

      // Slow down as the wall gets closer: manSpeed far away -> CREEP_SPEED at stopCm.
      // Uses the last GOOD reading (a missed ping can't bounce it back to full speed)
      // and speed is only ever allowed to go DOWN during the approach.
      if (lastValid < SLOW_START_CM) {
        float t = constrain((lastValid - stopCm) / (SLOW_START_CM - stopCm), 0.0, 1.0);
        float target = CREEP_SPEED + t * (manSpeed - CREEP_SPEED);
        speed = min(speed, target);
      }

      if (distance > 0) {                      // only real readings change the count
        if (distance <= stopCm + BRAKE_MARGIN_CM) closeCount++;
        else closeCount = 0;
      }

      if (closeCount >= CONFIRM_READINGS) {
        robotStop();
        Serial.print("wall is in front  (");
        Serial.print(distance, 1);
        Serial.println(" cm)");
        break;
      }

      if (index == 2) {
        bool rightwall = distance < 20;
        Serial.println(rightwall);
        if (!rightwall) { break; }
      }
    }

    if (DRIVE_TIMEOUT_MS > 0 && millis() - start > DRIVE_TIMEOUT_MS) {
      Serial.println("Drive timeout - stopping");
      break;
    }

    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      //Serial.print("Driving... heading: ");
      //Serial.print(heading, 1);
      //Serial.print("  dist: ");
      //Serial.print(lastValid, 0);
      //Serial.print("  speed: ");
      //Serial.println(speed, 2);
    }
  }
  robotStop();
  pauseWithHeading(move_del);
}



// ======================================================
//  Relative turning (To-Do #1)
// ======================================================
// Instead of zeroing the IMU after every turn, we remember which way the
// robot is SUPPOSED to face (goalHeading) and turn relative to that.
// Same simplicity (turnR / turnL / turnAround, no angles to track), but the
// 0-2.5 deg left over from each turn is corrected on the next move instead of
// being baked in forever - after 10 turns a hard reset could be ~20 deg off.
//
// If turnR() actually turns LEFT on your robot, change this to -90.0.
const float RIGHT_TURN_DEG = 90.0;

float goalHeading = 0;              // the direction the robot should be facing

void setGoal(float h)  { goalHeading = wrapAngle(h); }
void resetDirection()  { resetHeading(); goalHeading = 0; }   // "current way = straight"

void turnR()      { setGoal(goalHeading + RIGHT_TURN_DEG);       turnToHeading(goalHeading); }
void turnL()      { setGoal(goalHeading - RIGHT_TURN_DEG);       turnToHeading(goalHeading); }
void turnAround() { setGoal(goalHeading + 2 * RIGHT_TURN_DEG);   turnToHeading(goalHeading); }

// Drive along goalHeading - no angle argument needed.
void forwardUntil(float stopCm, int sensor = FRONT_SENSOR, float speed = FORWARD_SPEED) {
  driveUntilDistance(goalHeading, stopCm, sensor, speed);
}
void forwardFor(unsigned long ms, unsigned long pauseAfter = move_del) {
  driveForward(goalHeading, ms, 0, pauseAfter);
}

// ======================================================
//  Right-wall following (To-Do #4)
// ======================================================
enum WallResult { WALL_AHEAD, RIGHT_OPEN, TIMED_OUT };

const float RIGHT_WALL_CM  = 20.0;   // right reading above this (or none) = opening
const float FRONT_WALL_CM  = 12.0;   // front reading below this = wall ahead (was 5: too close for an HC-SR04)
const int   OPEN_CONFIRM   = 3;      // right must read "open" this many times in a row

const float WALL_TARGET_CM = 10.0;   // distance to hold from the right wall
const float KP_WALL        = 1.5;    // degrees of steering per cm of error
const float MAX_WALL_STEER = 12.0;   // never steer more than this off goalHeading

const unsigned long PASS_OPENING_MS = 350;  // after the right wall ends, drive this long so the robot's
                                            // middle is past the corner before turning right (tune!)

// Drives along a right wall at WALL_TARGET_CM. Stops and returns why as soon as something changes.
WallResult driveAlongRightWall(float targetHeading) {
  unsigned long start = millis();
  unsigned long lastPing = 0;
  int frontCount = 0;
  int openCount  = 0;
  float steer = 0;                   // degrees added to targetHeading to hold wall distance
  WallResult result = TIMED_OUT;

  while (millis() - start < DRIVE_TIMEOUT_MS) {

    // 1. Hold heading (target nudged toward/away from the wall)
    updateHeading();
    float err = wrapAngle(targetHeading + steer - heading);
    float corr = 0;
    if (fabs(err) > DRIVE_DEADBAND) {
      corr = constrain(err * KP_DRIVE, -MAX_CORR, MAX_CORR);
    }
    motorMove(leftside,  FORWARD_SPEED + corr);
    motorMove(rightside, FORWARD_SPEED - corr);

    // 2. Check front and right every interval
    if (millis() - lastPing >= PING_INTERVAL_MS) {
      lastPing = millis();

      float front = readUltrasonicCm(FRONT_SENSOR);
      if (front > 0 && front <= FRONT_WALL_CM) frontCount++;
      else frontCount = 0;

      float right = readUltrasonicCm(RIGHT_SENSOR);
      if (right < 0 || right > RIGHT_WALL_CM) {
        openCount++;                 // no reading = open
        steer = 0;                   // nothing to follow: go straight
      } else {
        openCount = 0;
        // too close (right < target) -> negative steer -> turn away from wall
        steer = constrain((right - WALL_TARGET_CM) * KP_WALL * (RIGHT_TURN_DEG > 0 ? 1 : -1),
                          -MAX_WALL_STEER, MAX_WALL_STEER);
      }

      // 3. Stop once a change is confirmed
      if (frontCount >= CONFIRM_READINGS) { result = WALL_AHEAD; break; }
      if (openCount  >= OPEN_CONFIRM)     { result = RIGHT_OPEN; break; }
    }
  }

  robotStop();
  return result;
}

// Drives straight until a wall appears on the right (used right after a right turn).
// Also stops if a wall appears in front, or after DRIVE_TIMEOUT_MS.
void driveUntilRightWall(float targetHeading) {
  unsigned long start = millis();
  unsigned long lastPing = 0;
  int wallCount = 0;

  while (millis() - start < DRIVE_TIMEOUT_MS) {
    // Hold heading
    updateHeading();
    float err = wrapAngle(targetHeading - heading);
    float corr = 0;
    if (fabs(err) > DRIVE_DEADBAND) {
      corr = constrain(err * KP_DRIVE, -MAX_CORR, MAX_CORR);
    }
    motorMove(leftside,  FORWARD_SPEED + corr);
    motorMove(rightside, FORWARD_SPEED - corr);

    if (millis() - lastPing >= PING_INTERVAL_MS) {
      lastPing = millis();

      float right = readUltrasonicCm(RIGHT_SENSOR);
      if (right > 0 && right <= RIGHT_WALL_CM) wallCount++;
      else wallCount = 0;

      float front = readUltrasonicCm(FRONT_SENSOR);
      if (front > 0 && front <= FRONT_WALL_CM) break;   // safety: don't hit a wall

      if (wallCount >= CONFIRM_READINGS) break;        // right wall found
    }
  }
  robotStop();
}

// Right-hand rule maze program. Start it with a wall on the robot's right.
//   right side opens  -> drive past the corner, turn right, drive in until there's a wall on the right again
//   wall in front     -> turn right if the right is open, otherwise left (twice in a row = dead-end U-turn)
// Runs until maxMs has passed. Uses goalHeading, so call resetDirection() first if needed.
void followRightWall(unsigned long maxMs = 60000) {
  unsigned long start = millis();
  Serial.println("Following right wall...");

  while (millis() - start < maxMs) {
    WallResult r = driveAlongRightWall(goalHeading);

    if (r == RIGHT_OPEN) {
      Serial.println("Right opening -> turn right");
      forwardFor(PASS_OPENING_MS, 200);
      turnR();
      driveUntilRightWall(goalHeading);
      pauseWithHeading(200);
    }
    else if (r == WALL_AHEAD) {
      pauseWithHeading(US_SAME_SENSOR_MS);          // make sure the next right reading is fresh
      float right = readUltrasonicCm(RIGHT_SENSOR);
      if (right < 0 || right > RIGHT_WALL_CM) {
        Serial.println("Wall ahead, right open -> turn right");
        turnR();
      } else {
        Serial.println("Wall ahead, right blocked -> turn left");
        turnL();
      }
    }
    // TIMED_OUT: long corridor, just keep going
  }

  robotStop();
  Serial.println("followRightWall done (time limit)");
}
