#pragma once
#include <Arduino.h>
#include "CytronMotorDriver.h"
#include "Pins.h"

// ---- Motors (pins in Pins.h) ----
// M1 (D5/D4) = left side, M2 (D6/D7) = right side, as in the wiring diagram.
CytronMD leftside(PWM_DIR, LEFT_PWM_PIN, LEFT_DIR_PIN);
CytronMD rightside(PWM_DIR, RIGHT_PWM_PIN, RIGHT_DIR_PIN);

// speed is -1.0 to 1.0
void motorMove(CytronMD &drive, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  drive.setSpeed((int)(speed * 255));
}
void robotStop() {
  motorMove(leftside, 0);
  motorMove(rightside, 0);
}