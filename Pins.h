#pragma once
// ============================================================
//  Pins.h - ALL robot wiring in one place (Arduino Mega 2560 +
//  Cytron 2A shield stacked). Matches robot_wiring_breadboard_full.
// ============================================================

// ---- Motors (Cytron shield, PWM_DIR mode) ----
// M1 = left side (FL + RL), M2 = right side (FR + RR)
const int LEFT_PWM_PIN  = 5;    // PWM1
const int LEFT_DIR_PIN  = 4;    // DIR1
const int RIGHT_PWM_PIN = 6;    // PWM2
const int RIGHT_DIR_PIN = 7;    // DIR2

// ---- Ultrasonics (HC-SR04, each with its own Trig + Echo, 5 V) ----
const int US_BOTTOM_TRIG_PIN = 22;   // front-bottom (box detection)
const int US_BOTTOM_ECHO_PIN = 23;
const int US_TOP_TRIG_PIN  = 46;   // front-top
const int US_TOP_ECHO_PIN  = 47;
const int US_LEFT_TRIG_PIN   = 26;   // left side
const int US_LEFT_ECHO_PIN   = 27;
const int US_RIGHT_TRIG_PIN    = 28;   // right side
const int US_RIGHT_ECHO_PIN    = 29;

// ---- Encoders (wired, not used in code yet) ----
// Note: 30-37 are NOT interrupt pins on the Mega.
const int ENC_FL_A_PIN = 30, ENC_FL_B_PIN = 31;
const int ENC_RL_A_PIN = 32, ENC_RL_B_PIN = 33;
const int ENC_FR_A_PIN = 34, ENC_FR_B_PIN = 35;
const int ENC_RR_A_PIN = 36, ENC_RR_B_PIN = 37;

// ---- Servos (signal pins, powered from DC-DC 5 V rail) ----
const int largeSV_PIN  = 8;    // Servo 1 (medium)
const int linkSV_PIN   = 9;    // Servo 2 (small)
const int bucketSV_PIN = 10;   // Servo 3 (small)

// ---- IMU (MPU-6050, 3.3 V) ----
// SDA = D20, SCL = D21 (hardware I2C, Wire)
