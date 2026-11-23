#pragma once
#include <Arduino.h>
#include <Wire.h>

// Arduino Mega 2560: IMU is on Wire = pins 20 (SDA) / 21 (SCL), powered from 3.3V.

// ---- Settings ----
const uint8_t MPU_ADDR  = 0x68;
const float   GYRO_SENS = 65.5;   // LSB per deg/s at ±500 deg/s

// ---- I2C safety ----
// Motor noise can lock up the I2C bus. Without a timeout the Mega's Wire
// library then waits FOREVER: the whole program freezes, Serial goes quiet,
// and the motors keep their last speed (robot drives off and never stops).
// With a timeout, a stuck read just fails, the bus is reset, and we carry on.
const unsigned long I2C_TIMEOUT_US = 3000;   // give up on a stuck I2C transfer after 3 ms
const long          I2C_CLOCK_HZ   = 100000; // 100 kHz is more noise-tolerant than 400 kHz

// ---- State ----
float gyroZ_bias = 0;
float heading    = 0;             // degrees, read this from anywhere
unsigned long lastMicros;
unsigned long imuErrors = 0;      // failed gyro reads (I2C noise / timeouts)

void i2cBegin() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);   // true = reset the I2C hardware on timeout
#else
  #warning "This Arduino core has no Wire timeout: update Arduino AVR Boards in the Boards Manager"
#endif
}

// ---------- Low-level (only used inside this file) ----------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void writeReg(uint8_t reg, uint8_t val);

// Called when a gyro read fails: count it, and after a timeout wake the MPU again.
void imuReadFailed() {
  imuErrors++;
#if defined(WIRE_HAS_TIMEOUT)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    writeReg(0x6B, 0x00);          // make sure the MPU is awake after the bus reset
  }
#endif
  if (imuErrors % 20 == 1) {
    Serial.print("WARNING: IMU I2C read failed (");
    Serial.print(imuErrors);
    Serial.println(" total) - motor noise? heading may drift slightly");
  }
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) {                  // bus error / timeout
    imuReadFailed();
    return (int16_t)gyroZ_bias;                            // treat as no rotation
  }
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {                              // read failed
    while (Wire.available()) Wire.read();
    imuReadFailed();
    return (int16_t)gyroZ_bias;                            // treat as no rotation
  }
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void mpuInit() {
  i2cBegin();
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