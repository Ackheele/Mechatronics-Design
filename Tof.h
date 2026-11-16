#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#define TOF_BUS Wire1

// ---- Sensor indexes ----
const int NUM_TOF = 4;
/*const int BOTTOM_SENSOR = 0;
const int FRONT_SENSOR  = 1;
const int RIGHT_SENSOR  = 2;
const int LEFT_SENSOR   = 3;*/

const char* TOF_NAMES[NUM_TOF] = {
  "Bottom", "Front", "Right", "Left"
};

// ---- Hardware ----
const int XSHUT_PINS[NUM_TOF] = {40, 41, 42, 43};

const uint8_t TOF_ADDR[NUM_TOF] = {
  0x2A, 0x2B, 0x2C, 0x2D
};

const int OFFSET_MM[NUM_TOF] = {
  -10, -30, -30, -30
};

// ---- Settings ----
const int TOF_AVG = 1;
const unsigned long TOF_STALE_MS = 500;    // no VALID reading this long -> -1
const unsigned long TOF_DEAD_MS  = 1000;   // no data AT ALL this long -> sensor stopped, restart

VL53L1X tof[NUM_TOF];

bool tofOk[NUM_TOF] = {false};

int tofBuf[NUM_TOF][TOF_AVG] = {};
int tofCount[NUM_TOF] = {};
int tofIdx[NUM_TOF] = {};

unsigned long tofLastValid[NUM_TOF] = {};   // last time a VALID distance came in
unsigned long tofLastData[NUM_TOF]  = {};   // last time ANY measurement came in

uint8_t tofLastStatus[NUM_TOF] = {};
int tofRestarts = 0;                        // how many times the sensors were restarted

// ---- Initialize sensors ----
void sensorsInit() {

  TOF_BUS.begin();
  TOF_BUS.setClock(400000);

  for (int i = 0; i < NUM_TOF; i++) {

    tofOk[i] = false;
    tofCount[i] = 0;
    tofIdx[i] = 0;
    tofLastValid[i] = 0;
    tofLastData[i] = millis();

    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }

  delay(10);

  for (int i = 0; i < NUM_TOF; i++) {

    pinMode(XSHUT_PINS[i], INPUT);
    delay(10);

    tof[i].setBus(&TOF_BUS);
    tof[i].setTimeout(500);

    if (!tof[i].init()) {

      Serial.print("ToF ");
      Serial.print(TOF_NAMES[i]);
      Serial.println(" NOT FOUND");

      pinMode(XSHUT_PINS[i], OUTPUT);
      digitalWrite(XSHUT_PINS[i], LOW);

      continue;
    }

    tof[i].setAddress(TOF_ADDR[i]);
    tof[i].setDistanceMode(VL53L1X::Medium);
    tof[i].setMeasurementTimingBudget(50000);
    tof[i].startContinuous(50);

    tofOk[i] = true;
    tofLastData[i] = millis();

    Serial.print("ToF ");
    Serial.print(TOF_NAMES[i]);
    Serial.print(" OK at 0x");
    Serial.println(TOF_ADDR[i], HEX);
  }

  Serial.println("ToF initialization complete.");
}

// ---- Continuously poll all sensors ----
void pollTof() {

  for (int i = 0; i < NUM_TOF; i++) {

    if (!tofOk[i]) continue;

    if (!tof[i].dataReady()) continue;

    tof[i].read(false);
    tofLastData[i] = millis();               // sensor is alive

    tofLastStatus[i] = tof[i].ranging_data.range_status;

    if (tofLastStatus[i] == VL53L1X::RangeValid) {

      tofBuf[i][tofIdx[i]] =
        tof[i].ranging_data.range_mm;

      tofIdx[i] = (tofIdx[i] + 1) % TOF_AVG;

      if (tofCount[i] < TOF_AVG) {
        tofCount[i]++;
      }

      tofLastValid[i] = millis();
    }
  }

  // ---- Watchdog: if a working sensor has gone silent, restart them all ----
  // A sensor that browns out or resets goes back to address 0x29 and stops
  // answering at its own address. Re-running sensorsInit() wakes them one at a
  // time again and gives each its address back.
  for (int i = 0; i < NUM_TOF; i++) {
    if (tofOk[i] && millis() - tofLastData[i] > TOF_DEAD_MS) {
      tofRestarts++;
      Serial.print("ToF ");
      Serial.print(TOF_NAMES[i]);
      Serial.print(" stopped responding - restarting sensors (restart #");
      Serial.print(tofRestarts);
      Serial.println(")");
      sensorsInit();
      return;
    }
  }
}

// ---- Get averaged distance in cm ----
float readDistanceCm(int i) {

  if (i < 0 || i >= NUM_TOF) return -1;

  if (!tofOk[i] || tofCount[i] == 0) return -1;

  if (millis() - tofLastValid[i] > TOF_STALE_MS) {
    return -1;
  }

  long sum = 0;

  for (int k = 0; k < tofCount[i]; k++) {
    sum += tofBuf[i][k];
  }

  return ((float)sum / tofCount[i] + OFFSET_MM[i]) / 10.0;
}

// ---- Print sensor readings ----
// Status 0 = valid. Any other number = sensor is alive but the reading was rejected.
void printAllSensors() {

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint < 200) return;

  lastPrint = millis();

  for (int i = 0; i < NUM_TOF; i++) {

    Serial.print(TOF_NAMES[i]);
    Serial.print(": ");

    if (!tofOk[i]) {
      Serial.print("NOT FOUND   ");
      continue;
    }

    float cm = readDistanceCm(i);

    if (cm < 0) {
      Serial.print("NO VALID READING");
    }
    else {
      Serial.print(cm, 1);
      Serial.print(" cm");
    }

    Serial.print(" [Status: ");
    Serial.print(tofLastStatus[i]);
    Serial.print(", data ");
    Serial.print(millis() - tofLastData[i]);
    Serial.print(" ms ago]   ");
  }

  Serial.print("Restarts: ");
  Serial.println(tofRestarts);
}