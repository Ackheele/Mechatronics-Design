#pragma once
#include <Arduino.h>
#include "CytronMotorDriver.h"

// ---- Pins ----
CytronMD leftside(PWM_DIR, 6, 7);
CytronMD rightside(PWM_DIR, 5, 4);

// speed is -1.0 to 1.0
void motorMove(CytronMD &drive, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  drive.setSpeed((int)(speed * 255));
}
void robotStop() {
  motorMove(leftside, 0);
  motorMove(rightside, 0);
}