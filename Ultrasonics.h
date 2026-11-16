#pragma once
#include <Arduino.h>
#include <Ultrasonic.h>
#include "IMU.h"          // readUltrasonicCm() calls updateHeading()

// ---- Pins ----
const int TRIG_PIN = 22;                       // all four Trig pins joined here
const unsigned long US_TIMEOUT_US = 12000;     // ~2 m max range

Ultrasonic ultrasonic1(TRIG_PIN, 30, US_TIMEOUT_US);   // bottom, Echo via divider
Ultrasonic ultrasonic2(TRIG_PIN, 31, US_TIMEOUT_US);   // top
Ultrasonic ultrasonic3(TRIG_PIN, 34, US_TIMEOUT_US);   // right
Ultrasonic ultrasonic4(TRIG_PIN, 33, US_TIMEOUT_US);   // left (45 deg)

// ---- Sensor indexes (use these names instead of 0-3) ----
const int NUM_US        = 4;
const int BOTTOM_SENSOR = 0;   // US1
const int FRONT_SENSOR  = 1;   // US2 top (wall distance)
const int RIGHT_SENSOR  = 2;   // US3
const int LEFT_SENSOR   = 3;   // US4 (45 deg)

Ultrasonic* sensors[NUM_US] = {&ultrasonic1, &ultrasonic2, &ultrasonic3, &ultrasonic4};
const char* US_NAMES[NUM_US] = {"Bottom", "Top", "Right", "Left"};

// ---- Settings ----
const unsigned long US_GAP_MS = 20;   // quiet time between sensors
const float MIN_VALID_CM      = 4.0;  // anything closer is treated as a bad reading

// ---- State ----
float distances[NUM_US];              // latest readings, -1 = no echo

// Returns cm, or -1 for no echo / impossibly close readings.
float readUltrasonicCm(int i) {
  unsigned int cm = sensors[i]->read();
  updateHeading();                    // catch up after the ping's wait
  if (cm == 0 || cm < MIN_VALID_CM) return -1;
  return (float)cm;
}

// Reads all four one after another with a quiet gap between them.
void readAllUltrasonics() {
  for (int i = 0; i < NUM_US; i++) {
    distances[i] = readUltrasonicCm(i);
    pauseWithHeading(US_GAP_MS);
  }
}

// Reads all four and prints them on one line.
void printAllUltrasonics() {
  readAllUltrasonics();
  for (int i = 0; i < NUM_US; i++) {
    Serial.print(US_NAMES[i]);
    Serial.print(": ");
    Serial.print(distances[i], 1);
    Serial.print("   ");
  }
  Serial.print(" HEADING: ");
  Serial.println(heading, 1);
}