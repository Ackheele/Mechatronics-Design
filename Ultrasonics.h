#pragma once
#include <Arduino.h>
#include "IMU.h"          // readUltrasonicCm() calls updateHeading()
#include "Pins.h"

// ============================================================
//  Ultrasonics.h - 4x HC-SR04, read directly with pulseIn()
//  (no Ultrasonic.h library any more - see "why" notes below)
// ============================================================

// ---- Sensor indexes (use these names instead of 0-3) ----
const int NUM_US        = 4;
const int BOTTOM_SENSOR = 0;   // front-bottom
const int FRONT_SENSOR  = 1;   // front-top (wall distance)
const int RIGHT_SENSOR  = 2;
const int LEFT_SENSOR   = 3;

const int US_TRIG[NUM_US] = {US_BOTTOM_TRIG_PIN, US_TOP_TRIG_PIN, US_RIGHT_TRIG_PIN, US_LEFT_TRIG_PIN};
const int US_ECHO[NUM_US] = {US_BOTTOM_ECHO_PIN, US_TOP_ECHO_PIN, US_RIGHT_ECHO_PIN, US_LEFT_ECHO_PIN};
const char* US_NAMES[NUM_US] = {"Bottom", "Top", "Right", "Left"};

// ---- Settings ----
const unsigned long US_TIMEOUT_US = 12000;   // ~200 cm max range
const float MIN_VALID_CM          = 3.0;     // closer than this = bad reading
const float MAX_VALID_CM          = 200.0;
const unsigned long US_GAP_MS     = 20;      // quiet time between sensors in readAllUltrasonics()
const unsigned long US_SAME_SENSOR_MS = 60;  // same sensor asked again sooner -> return its last value
const unsigned long US_ANY_SENSOR_MS  = 15;  // minimum quiet time between ANY two pings
const unsigned long US_BUSY_WAIT_MS   = 40;  // max wait for a sensor that is still "busy"

// WHY the library was replaced:
// When an HC-SR04 hears no echo it keeps its Echo pin HIGH for 38 ms (cheap
// clones: up to ~200 ms). The old library gave up after 12 ms, so the NEXT
// ping was often sent while the sensor was still busy -> the trigger was
// ignored and we measured the tail of the old pulse (random value / no reading).
// pingCm() now waits for Echo to go LOW before triggering, and counts every
// failure type so the diagnostic can tell you WHY readings are missing.

// ---- State ----
float distances[NUM_US]            = {-1, -1, -1, -1};   // latest readings, -1 = no echo
float usLastCm[NUM_US]             = {-1, -1, -1, -1};
unsigned long usLastPingMs[NUM_US] = {0, 0, 0, 0};
unsigned long usLastAnyPingMs      = 0;
bool usReady = false;

// ---- Stats (printed by the diagnostic) ----
unsigned long usPings[NUM_US], usGood[NUM_US], usBusy[NUM_US], usNoEcho[NUM_US], usRange[NUM_US];

void ultrasonicInit() {
  for (int i = 0; i < NUM_US; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
  }
  usReady = true;
}

void usResetStats() {
  for (int i = 0; i < NUM_US; i++) usPings[i] = usGood[i] = usBusy[i] = usNoEcho[i] = usRange[i] = 0;
}

// One real ping. Returns cm, or -1.
float pingCm(int i) {
  usPings[i]++;

  // Sensor still busy with its previous ping? Wait for it (up to US_BUSY_WAIT_MS).
  unsigned long t = millis();
  while (digitalRead(US_ECHO[i]) == HIGH) {
    if (millis() - t > US_BUSY_WAIT_MS) { usBusy[i]++; return -1; }
    updateHeading();
  }

  digitalWrite(US_TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG[i], LOW);

  // pulseInLong (not pulseIn): pulseIn counts CPU cycles and reads short when the
  // encoder timer interrupt is running; pulseInLong uses micros() so it stays accurate.
  unsigned long us = pulseInLong(US_ECHO[i], HIGH, US_TIMEOUT_US);
  if (us == 0) { usNoEcho[i]++; return -1; }

  float cm = us / 58.0;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) { usRange[i]++; return -1; }

  usGood[i]++;
  return cm;
}

// Returns cm, or -1 for no echo / bad reading.
// Safe to call as often as you like: it never pings faster than the sensors can handle.
float readUltrasonicCm(int i) {
  if (i < 0 || i >= NUM_US) return -1;
  if (!usReady) ultrasonicInit();

  // Same sensor pinged very recently: reuse that reading.
  if (usLastPingMs[i] != 0 && millis() - usLastPingMs[i] < US_SAME_SENSOR_MS) {
    return usLastCm[i];
  }

  // Another sensor just pinged: let its echo die out first.
  unsigned long sinceAny = millis() - usLastAnyPingMs;
  if (usLastAnyPingMs != 0 && sinceAny < US_ANY_SENSOR_MS) {
    pauseWithHeading(US_ANY_SENSOR_MS - sinceAny);
  }

  usLastCm[i] = pingCm(i);
  usLastPingMs[i] = millis();
  usLastAnyPingMs = usLastPingMs[i];
  updateHeading();                    // catch up after the ping's wait
  return usLastCm[i];
}

// Reads all four one after another with a quiet gap between them.
void readAllUltrasonics() {
  for (int i = 0; i < NUM_US; i++) {
    distances[i] = readUltrasonicCm(i);
    pauseWithHeading(US_GAP_MS);
  }
}

// Per-sensor hit rate + why the misses happened.
//  busy   = sensor still holding Echo HIGH (crosstalk / clone holding the pin / noise)
//  noEcho = nothing came back (aimed at nothing, bad angle, loose Echo wire)
//  range  = came back too close / too far (noise spikes, sensor <3 cm from wall)
void usPrintStats() {
  for (int i = 0; i < NUM_US; i++) {
    Serial.print(US_NAMES[i]);
    Serial.print(": ");
    if (usPings[i] == 0) { Serial.print("--"); }
    else {
      Serial.print(100.0 * usGood[i] / usPings[i], 0);
      Serial.print("% ok (busy ");  Serial.print(usBusy[i]);
      Serial.print(", noEcho ");    Serial.print(usNoEcho[i]);
      Serial.print(", range ");     Serial.print(usRange[i]);
      Serial.print(")");
    }
    Serial.print(i < NUM_US - 1 ? " | " : "\n");
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
