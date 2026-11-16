#include <SimpleKalmanFilter.h>
#include "Arm.h"
#include "Drive.h"     // Motor.h, IMU.h, Ultrasonic.h

// ---- Modes ----
const bool DIAGNOSTIC_MODE = false;
const bool USE_ARM = false;

// ---- Box detection ----
const float BOX_DETECT_CM = 15.0;
const int BOX_CONFIRM = 3;

int boxCount = 0;
bool boxFound = false;

// ---- Serial printing ----
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200;

// ---- Box detection ----
bool boxDetected() {
  float bottom = readUltrasonicCm(BOTTOM_SENSOR);
  float top = readUltrasonicCm(FRONT_SENSOR);

  bool bottomSeesObject =
      (bottom >= 0 && bottom < BOX_DETECT_CM);

  bool topSeesObject =
      (top >= 0 && top < BOX_DETECT_CM);

  return bottomSeesObject && !topSeesObject;
}

// ---- Update box confirmation ----
void updateBoxCount() {
  if (boxDetected()) {
    boxCount++;
  } else {
    boxCount = 0;
  }

  if (boxCount >= BOX_CONFIRM) {
    boxFound = true;
    robotStop();
    Serial.println("BOX CONFIRMED - ROBOT STOPPED");
  }
}

// ---- Print all readings on one timer ----
void printSensorReadings() {
  if (millis() - lastPrint < PRINT_INTERVAL) return;

  lastPrint = millis();

  float bottom = readUltrasonicCm(BOTTOM_SENSOR);
  float front  = readUltrasonicCm(FRONT_SENSOR);
  float left   = readUltrasonicCm(LEFT_SENSOR);
  float right  = readUltrasonicCm(RIGHT_SENSOR);

  Serial.print("BOTTOM: ");
  if (bottom < 0) Serial.print("NO READING");
  else {
    Serial.print(bottom, 1);
    Serial.print(" cm");
  }

  Serial.print(" | FRONT: ");
  if (front < 0) Serial.print("NO READING");
  else {
    Serial.print(front, 1);
    Serial.print(" cm");
  }

  Serial.print(" | LEFT: ");
  if (left < 0) Serial.print("NO READING");
  else {
    Serial.print(left, 1);
    Serial.print(" cm");
  }

  Serial.print(" | RIGHT: ");
  if (right < 0) Serial.print("NO READING");
  else {
    Serial.print(right, 1);
    Serial.print(" cm");
  }

  Serial.print(" | BOX COUNT: ");
  Serial.print(boxCount);
  Serial.print("/");
  Serial.print(BOX_CONFIRM);

  if (boxFound) {
    Serial.print(" | BOX FOUND");
  }

  Serial.println();
}

// ---- Called during driving ----
bool whileDriving() {
  pollTof();
  if (boxFound) {
    robotStop();
    return true;
  }

  updateBoxCount();
  printSensorReadings();

  if (boxFound) {
    robotStop();
    return true;
  }

  return false;
}

// ---- Setup ----
void setup() {
  robotStop();

  Serial.begin(115200);

  if (USE_ARM) {
    armInit();
    moveHome(HOME);
  }

  delay(1000);

  imuInit();

  sensorsInit();

  Serial.println("Calibrating / initializing ultrasonic sensors...");

  Serial.print("Initial heading: ");
  Serial.println(heading, 1);

  // Diagnostic mode
  if (DIAGNOSTIC_MODE) {
    while (true) {
      updateHeading();
      pollTof();
      Serial.print("ToF BOTTOM: ");
      Serial.print(readDistanceCm(BOTTOM_SENSOR), 1);
      Serial.print(" cm   ");
      printAllUltrasonics();
    }
  }

  // ---- Original movement sequence ----

  jolt();

  driveUntilDistance(0, 12, BOTTOM_SENSOR);

  driveForward(90, 750, 0);
  driveForward(90, 750, 0);

  driveUntilDistance(90, 12, BOTTOM_SENSOR);

  driveForward(180, 750, 0);
  driveForward(180, 750, 0);

  driveUntilDistance(180, 55, BOTTOM_SENSOR);

  driveForward(270, 750, 0);
  driveForward(270, 750, 0);

  robotStop();

  if (USE_ARM) {
    armToPickup();
  }

  Serial.println("Done.");
}

// ---- Loop ----
void loop() {
  updateHeading();

  if (whileDriving()) {
    robotStop();
  }
}
