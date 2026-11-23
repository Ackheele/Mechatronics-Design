#pragma once
#include <Arduino.h>
#include <Ultrasonic.h>
#include "IMU.h"          // readUltrasonicCm() calls updateHeading()

#include "Pins.h"

// ---- Sensors (pins in Pins.h, each has its own Trig + Echo) ----
const unsigned long US_TIMEOUT_US = 12000;     // ~2 m max range

Ultrasonic ultrasonic1(US_BOTTOM_TRIG_PIN, US_BOTTOM_ECHO_PIN, US_TIMEOUT_US);   // front-bottom
Ultrasonic ultrasonic2(US_TOP_TRIG_PIN,    US_TOP_ECHO_PIN,    US_TIMEOUT_US);   // front-top
Ultrasonic ultrasonic3(US_RIGHT_TRIG_PIN,  US_RIGHT_ECHO_PIN,  US_TIMEOUT_US);   // right side
Ultrasonic ultrasonic4(US_LEFT_TRIG_PIN,   US_LEFT_ECHO_PIN,   US_TIMEOUT_US);   // left side

// ---- Sensor indexes (use these names instead of 0-3) ----
const int NUM_US        = 4;
const int BOTTOM_SENSOR = 0;   // US1
const int FRONT_SENSOR  = 1;   // US2 front-top (wall distance)
const int RIGHT_SENSOR  = 2;   // US3
const int LEFT_SENSOR   = 3;   // US4

Ultrasonic* sensors[NUM_US] = {&ultrasonic1, &ultrasonic2, &ultrasonic3, &ultrasonic4};
const char* US_NAMES[NUM_US] = {"Bottom", "Top", "Right", "Left"};

// ---- Settings ----
const unsigned long US_GAP_MS = 20;   // quiet time between sensors
const float MIN_VALID_CM      = 4.0;  // anything closer is treated as a bad reading
// An HC-SR04 needs ~60 ms between its own pings, and two sensors pinging
// back-to-back hear each other's echoes. Pinging too fast gives fake
// "NO READING"s and random 200 cm values, so readUltrasonicCm() enforces:
const unsigned long US_SAME_SENSOR_MS = 60;  // same sensor asked again sooner -> return its last value
const unsigned long US_ANY_SENSOR_MS  = 15;  // minimum quiet time between ANY two pings

// ---- State ----
float distances[NUM_US];              // latest readings, -1 = no echo
float usLastCm[NUM_US]          = {-1, -1, -1, -1};
unsigned long usLastPingMs[NUM_US] = {0, 0, 0, 0};
unsigned long usLastAnyPingMs   = 0;

// Returns cm, or -1 for no echo / impossibly close readings.
// Safe to call as often as you like: it never pings faster than the sensors can handle.
float readUltrasonicCm(int i) {
  if (i < 0 || i >= NUM_US) return -1;

  // Same sensor pinged very recently: reuse that reading.
  if (usLastPingMs[i] != 0 && millis() - usLastPingMs[i] < US_SAME_SENSOR_MS) {
    return usLastCm[i];
  }

  // Another sensor just pinged: let its echo die out first.
  unsigned long sinceAny = millis() - usLastAnyPingMs;
  if (usLastAnyPingMs != 0 && sinceAny < US_ANY_SENSOR_MS) {
    pauseWithHeading(US_ANY_SENSOR_MS - sinceAny);
  }

  unsigned int cm = sensors[i]->read();
  usLastPingMs[i] = millis();
  usLastAnyPingMs = usLastPingMs[i];
  updateHeading();                    // catch up after the ping's wait

  usLastCm[i] = (cm == 0 || cm < MIN_VALID_CM) ? -1 : (float)cm;
  return usLastCm[i];
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