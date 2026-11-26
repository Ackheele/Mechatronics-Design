#include <SimpleKalmanFilter.h>
#include "Pins.h"      // ALL wiring lives here (matches the wiring diagram)
#include "Arm.h"
#include "Drive.h"     // Motor.h, IMU.h, Ultrasonics.h
#include "Encoders.h"  // rear wheel encoders

// ---- Modes ----
const bool ENCODER_TEST_MODE = true;  // true = Serial-controlled encoder test (never drives off). ROBOT ON A BOX!
const bool DIAGNOSTIC_MODE = false;    // true = only print heading + all ultrasonics (never drives off)
const bool DIAG_MOTORS_ON  = false;   // with DIAGNOSTIC_MODE: spin the wheels too. PUT THE ROBOT ON A BOX FIRST!
const float DIAG_MOTOR_POWER = 0.4;   // same as FORWARD_SPEED, so the electrical noise matches real driving
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

// ---- Diagnostic: print heading + every ultrasonic forever ----
// Run it twice with every sensor pointed at a wall/box 15-50 cm away:
//   DIAG_MOTORS_ON = false, then true (wheels in the air).
// Every 3 s it prints each sensor's hit rate and why the misses happened.
//   Rate drops a lot with motors on  -> electrical noise: see the wiring notes
//   Rate fine both ways but bad when driving -> angle/vibration: sensor sees the wall at a slant
void runDiagnostic() {
  robotStop();
  Serial.println();
  if (DIAG_MOTORS_ON) Serial.println("===== DIAGNOSTIC MODE: motors ON (wheels must be off the ground!) =====");
  else                Serial.println("===== DIAGNOSTIC MODE: motors OFF, printing sensors =====");
  Serial.println("Pins: BOTTOM 22/23 | FRONT-TOP 13/12 | RIGHT 28/29 | LEFT 26/27 (Trig/Echo)");
  Serial.println("Turn the robot by hand to check heading; hold a hand in front of each sensor.");

  usResetStats();
  unsigned long lastStats = millis();

  while (true) {
    if (DIAG_MOTORS_ON) {
      motorMove(leftside,  DIAG_MOTOR_POWER);
      motorMove(rightside, DIAG_MOTOR_POWER);
    } else {
      robotStop();                        // make sure nothing moves
    }
    readAllUltrasonics();                 // reads all 4 with a quiet gap between them (keeps heading updated)

    Serial.print("HEADING: ");
    Serial.print(wrapAngle(heading), 1);
    Serial.print(" deg");
    printCm(" | BOTTOM: ",    distances[BOTTOM_SENSOR]);
    printCm(" | FRONT-TOP: ", distances[FRONT_SENSOR]);
    printCm(" | RIGHT: ",     distances[RIGHT_SENSOR]);
    printCm(" | LEFT: ",      distances[LEFT_SENSOR]);
    Serial.println();

    if (millis() - lastStats > 3000) {
      lastStats = millis();
      Serial.print("HIT RATE  ");
      usPrintStats();
    }

    pauseWithHeading(100);
  }
}

// ---- Encoder test: type commands in the Serial Monitor (115200, "Newline") ----
//   R 1     right side, 1 wheel turn forward
//   L -2    left side, 2 turns backward
//   L 0.5   half a turn
//   W       watch counts with motors off (turn a wheel by hand to calibrate)
//   Z       zero both counts
//   F 10    drive 10 cm forward (F -10 = backward)  <- robot ON THE FLOOR for this one
void runEncoderTest() {
  robotStop();
  Serial.println();
  Serial.println("===== ENCODER TEST (rear encoders, robot must be on a box) =====");
  Serial.print("Counts per wheel turn: "); Serial.println(COUNTS_PER_REV, 0);
  Serial.print("Wheel: "); Serial.print(WHEEL_DIAMETER_CM, 2); Serial.print(" cm dia, ");
  Serial.print(COUNTS_PER_CM, 1); Serial.println(" counts per cm");
  Serial.println("Commands: R <turns> | L <turns> | F <cm> | W (watch) | Z (zero)   e.g.  R 1   F 10");

  while (true) {
    updateHeading();
    if (!Serial.available()) continue;
    char c = toupper(Serial.read());

    if (c == 'R' || c == 'L') {
      float turns = Serial.parseFloat();          // "R" on its own = 1 turn
      if (turns == 0) turns = 1;
      spinWheel(c == 'R' ? RIGHT_SIDE : LEFT_SIDE, turns);
    } else if (c == 'F') {
      float cm = Serial.parseFloat();
      if (cm == 0) cm = 10;
      resetDirection();                          // "straight" = the way it faces right now
      driveCm(cm);
    } else if (c == 'W') {
      encoderWatch();
    } else if (c == 'Z') {
      encReset();
      Serial.println("Counts zeroed");
    }
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

  //ultrasonicInit();
  imuInit();
  resetDirection();          // current way the robot faces = 0

  Serial.print("Initial heading: ");
  Serial.println(heading, 1);

  encodersBegin();

  // Encoder test mode (never returns)
  if (ENCODER_TEST_MODE) {
    runEncoderTest();
  }
  // Or call it straight from code, e.g.:
  //   spinWheel(RIGHT_SIDE, 1);    // one full turn, right side
  //   spinWheel(LEFT_SIDE, 3);     // three turns, left side
  //   driveCm(10);                 // 10 cm straight forward (both encoders averaged)

  // Diagnostic mode (never returns)
  if (DIAGNOSTIC_MODE) {
    runDiagnostic();
  }

  // Troubleshooting

  


  // ---- New style: relative turns, no angles to keep track of ----
  //forwardUntil(6);            // drive until 13 cm from the front wall
  //turnL();
  //robotStop();
  //delay(1000);                     // 90 deg left of where it should be facing
  //forwardUntil(6);
  //turnL();
  //forwardUntil(6);

  //turnAround();
  // resetDirection();            // only if the robot was re-placed / re-aligned by hand
  //
  // followRightWall(60000);      // right-hand-rule maze for up to 60 s (start with a wall on the right)

  // ---- Original movement sequence ----

  // Start of Phase A, goes forward turns left, goes forward, turns right, goes forward
  //phaseA();
  //delay(1000);
  //imuInit();
  //phaseB();
  
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
