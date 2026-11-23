#include <SimpleKalmanFilter.h>
#include "Pins.h"      // ALL wiring lives here (matches the wiring diagram)
#include "Arm.h"
#include "Drive.h"     // Motor.h, IMU.h, Ultrasonics.h

// ---- Modes ----
const bool DIAGNOSTIC_MODE = false;    // true = motors OFF, only print heading + all ultrasonics
const bool USE_ARM = false;

// ---- Serial printing ----
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200;

// ---- Print one distance ----
void printCm(const char* label, float cm) {
  Serial.print(label);
  if (cm < 0) Serial.print("NO READING");
  else {
    Serial.print(cm, 1);
    Serial.print(" cm");
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

  printCm("BOTTOM: ", bottom);
  printCm(" | FRONT: ", front);
  printCm(" | LEFT: ", left);
  printCm(" | RIGHT: ", right);
  Serial.println();
}

// ---- Diagnostic: motors off, print heading + every ultrasonic forever ----
void runDiagnostic() {
  robotStop();
  Serial.println();
  Serial.println("===== DIAGNOSTIC MODE: motors OFF, printing sensors =====");
  Serial.println("Pins: BOTTOM 22/23 | FRONT-TOP 46/47 | RIGHT 24/25 | LEFT 26/27 (Trig/Echo)");
  Serial.println("Turn the robot by hand to check heading; hold a hand in front of each sensor.");

  while (true) {
    robotStop();                          // make sure nothing moves
    readAllUltrasonics();                 // reads all 4 with a quiet gap between them (keeps heading updated)

    Serial.print("HEADING: ");
    Serial.print(wrapAngle(heading), 1);
    Serial.print(" deg");
    printCm(" | BOTTOM: ",    distances[BOTTOM_SENSOR]);
    printCm(" | FRONT-TOP: ", distances[FRONT_SENSOR]);
    printCm(" | RIGHT: ",     distances[RIGHT_SENSOR]);
    printCm(" | LEFT: ",      distances[LEFT_SENSOR]);
    Serial.println();

    pauseWithHeading(100);
  }
}

void phaseA(){
  driveUntilDistance(0, 13, FRONT_SENSOR, 0.35);
  turnToHeading(90);
  driveUntilDistance(90, 25, FRONT_SENSOR, 0.35);
  driveUntilDistance(90, 10, FRONT_SENSOR, 0.30);
  turnToHeading(180);
  robotStop();
}

void phaseB(){
  driveUntilDistance(0, 13, FRONT_SENSOR, 0.35);
  turnToHeading(90);
  driveUntilDistance(90, 13, FRONT_SENSOR, 0.35);
  turnToHeading(180);
  driveUntilDistance(180, 13, FRONT_SENSOR, 0.35);
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

  Serial.print("Initial heading: ");
  Serial.println(heading, 1);

  // Diagnostic mode (never returns)
  if (DIAGNOSTIC_MODE) {
    runDiagnostic();
  }

  // ---- Original movement sequence ----

  // Start of Phase A, goes forward turns left, goes forward, turns right, goes forward
  phaseA();
  delay(1000);
  imuInit();
  phaseB();
  
  // At start of phase B, goes down the ramp, stops infront of the maze
  //driveUntilDistance(90, 10, FRONT_SENSOR, 0.5);
  //turnToHeading(180);
  //driveUntilDistance(180, 10, FRONT_SENSOR, 0.4);
  //turnToHeading(270);
  //driveUntilDistance(270, 40, FRONT_SENSOR, 0.4);
  //turnToHeading(0);

  robotStop();

  if (USE_ARM) {
    armToPickup();
  }

  Serial.println("Done.");
}

// ---- Loop ----
void loop() {
  updateHeading();
}
