#pragma once
#include <Arduino.h>
#include <Wire.h>

// ** ARDUINO DUE **: IMU is on Wire = pins 20 (SDA) / 21 (SCL), powered from 3.3V.

// ---- Settings ----
const uint8_t MPU_ADDR  = 0x68;
const float   GYRO_SENS = 65.5;   // LSB per deg/s at ±500 deg/s

// ---- State ----
float gyroZ_bias = 0;
float heading    = 0;             // degrees, read this from anywhere
unsigned long lastMicros;

// ---------- Low-level (only used inside this file) ----------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return (int16_t)gyroZ_bias;   // read failed: treat as no rotation
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void mpuInit() {
  Wire.begin();
  Wire.setClock(400000);
  delay(1000);

  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    Wire.beginTransmission(MPU_ADDR);
    ok = (Wire.endTransmission() == 0);
    if (!ok) delay(200);
  }
  if (!ok) {
    Serial.println("MPU6050 not found on pins 20/21! Check wiring.");
    while (true);
  }

  writeReg(0x6B, 0x00);   // wake up
  writeReg(0x1B, 0x08);   // gyro ±500 deg/s
  writeReg(0x1A, 0x03);   // low-pass filter
  Serial.println("IMU OK at 0x68");
}

void calibrateGyro() {
  const int N = 2000;
  long sum = 0;
  for (int i = 0; i < N; i++) { sum += readGyroZRaw(); delay(2); }
  gyroZ_bias = (float)sum / N;
}

// ---------- Public (what the rest of the robot uses) ----------

// Call once in setup(). Robot must be still.
void imuInit() {
  Serial.println("Calibrating gyro... don't move the robot");
  mpuInit();
  delay(3000);
  calibrateGyro();
  heading = 0;
  lastMicros = micros();   // must come right before tracking starts
}

void resetHeading() {
  heading = 0;
}

// Call as often as possible. Integrates gyro rate into heading.
void updateHeading() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;
  float rate = (readGyroZRaw() - gyroZ_bias) / GYRO_SENS;
  heading -= rate * dt;
}

// Like delay(), but keeps heading tracking alive.
void pauseWithHeading(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) updateHeading();
}

// Keeps an angle in (-180, 180].
float wrapAngle(float a) {
  while (a > 180.0)   a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}