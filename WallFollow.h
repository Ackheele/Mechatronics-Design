#pragma once
#include <Arduino.h>
#include "Drive.h"
#include "Encoders.h"

// ============================================================
//  WallFollow.h - drive along a wall, stop at the first gap
// ============================================================
//
//   followRightWall();          hold SIDE_GAP_CM from the right wall, stop at a gap
//   followLeftWall();           same on the left
//   followRightWall(7.0);       hold 7 cm from the right wall
//   followWall(LEFT_SIDE, 5.0, 0.35);   side, distance (cm), speed
//
// It drives forward along goalHeading (works with turnR / turnL / driveCm),
// steering a little toward / away from the wall to keep the distance, and
// stops as soon as one of these happens:
//   WALL_GAP     the wall on that side ended (corner / opening)
//   WALL_FRONT   a wall in front (safety, so it never drives into one)
//   WALL_TIMEOUT DRIVE_TIMEOUT_MS passed
// It returns which one, so you can decide what to do next.

// ---- Geometry: 20.3 cm corridor, 4" (10.16 cm) wide robot, sensors on its edges ----
const float CORRIDOR_CM    = 20.3;
const float ROBOT_WIDTH_CM = 4.0 * 2.54;
const float SIDE_GAP_CM    = (CORRIDOR_CM - ROBOT_WIDTH_CM) / 2;   // ~5.1 cm = centred in the corridor

// ---- Tuning ----
const float WF_SPEED      = 0.40;   // driving power
const float WF_GAP_CM     = 12.0;   // side reading above this (or no echo) = gap
                                    // (along a wall it reads ~5 cm, a gap reads 25+ cm)
const int   WF_GAP_CONFIRM = 2;     // gap must be seen this many pings in a row before stopping.
                                    // 1 = stop on the very first gap reading (faster, but one bad
                                    // ping can stop it); 2 = ~70 ms later, ignores single glitches
const float WF_KP         = 2.0;    // degrees of steering per cm off the target distance
const float WF_MAX_STEER  = 12.0;   // never steer more than this off the driving direction
const float WF_FRONT_STOP_CM = 6.0; // safety: stop if a wall is this close in front

enum WallStop { WALL_GAP, WALL_FRONT, WALL_TIMEOUT };

// Side distance: touching the wall (closer than MIN_VALID_CM) still counts as a wall,
// no echo = -1 (gap).
float wfSideCm(int sensor) {
  float d = readUltrasonicCm(sensor);
  if (d < 0 && usTooClose[sensor]) d = MIN_VALID_CM;
  return d;
}

WallStop followWall(Side side, float wallCm = SIDE_GAP_CM, float speed = WF_SPEED) {
  int sensor = (side == RIGHT_SIDE) ? RIGHT_SENSOR : LEFT_SENSOR;
  const char* name = (side == RIGHT_SIDE) ? "right" : "left";

  long l0 = encRead(LEFT_SIDE), r0 = encRead(RIGHT_SIDE);
  unsigned long start = millis(), lastPing = 0, lastPrint = 0;
  float steer = 0, wallNow = -1, frontNow = -1;
  int gapCount = 0, frontCount = 0;
  WallStop result = WALL_TIMEOUT;

  Serial.print("Following "); Serial.print(name); Serial.print(" wall at ");
  Serial.print(wallCm, 1); Serial.println(" cm");

  while (millis() - start < DRIVE_TIMEOUT_MS) {
    updateHeading();

    if (millis() - lastPing >= PING_INTERVAL_MS) {
      lastPing = millis();

      // ---- side wall ----
      wallNow = wfSideCm(sensor);
      bool wall = (wallNow > 0 && wallNow <= WF_GAP_CM);
      if (!wall) {
        if (++gapCount >= WF_GAP_CONFIRM) { result = WALL_GAP; break; }
      } else {
        gapCount = 0;
        // too far from the wall -> steer toward it, too close -> steer away
        float off = wallNow - wallCm;                   // + = too far away
        float s = constrain(off * WF_KP, -WF_MAX_STEER, WF_MAX_STEER);
        if (side == LEFT_SIDE) s = -s;                  // toward the LEFT wall = turn left
        steer = s * (RIGHT_TURN_DEG > 0 ? 1 : -1);      // match turnR()'s sign convention
      }

      // ---- front safety ----
      frontNow = readUltrasonicCm(FRONT_SENSOR);
      bool frontClose = (frontNow > 0 && frontNow <= WF_FRONT_STOP_CM) ||
                        (frontNow < 0 && usTooClose[FRONT_SENSOR]);
      if (frontClose) { if (++frontCount >= CONFIRM_READINGS) { result = WALL_FRONT; break; } }
      else if (frontNow > 0) frontCount = 0;
    }

    driveStraightStep(goalHeading + steer, speed);

    if (millis() - lastPrint > 300) {
      lastPrint = millis();
      Serial.print("  "); Serial.print(name); Serial.print(" ");
      Serial.print(wallNow, 1); Serial.print(" cm  front ");
      Serial.print(frontNow, 1); Serial.print("  steer ");
      Serial.print(steer, 1); Serial.print("  dist ");
      Serial.print(encAvgSince(l0, r0) / COUNTS_PER_CM, 1); Serial.println(" cm");
    }
  }

  robotStop();
  float drove = encAvgSince(l0, r0) / COUNTS_PER_CM;
  Serial.print("  Stopped: ");
  Serial.print(result == WALL_GAP ? "gap on the " : result == WALL_FRONT ? "wall in front" : "timeout");
  if (result == WALL_GAP) { Serial.print(name); Serial.print(" side"); }
  Serial.print(" after "); Serial.print(drove, 1); Serial.println(" cm");
  encWaitBothStill();
  return result;
}

WallStop followRightWall(float wallCm = SIDE_GAP_CM, float speed = WF_SPEED) {
  return followWall(RIGHT_SIDE, wallCm, speed);
}
WallStop followLeftWall(float wallCm = SIDE_GAP_CM, float speed = WF_SPEED) {
  return followWall(LEFT_SIDE, wallCm, speed);
}
