#pragma once
#include <Arduino.h>
#include <Servo.h>
// ---- Pins ----
const int largeSV_PIN  = 24;
const int linkSV_PIN   = 25;
const int bucketSV_PIN = 26;

struct Pose {
  int large;
  int link;
  int bucket;
};

// ---- EDIT THESE ----s
Pose HOME   = {170, 180, 175};
Pose PICKUP = { 20, 160,   5};
Pose DROP = {20, 165, 60};
// --------------------

Servo largeSV;
Servo linkSV;
Servo bucketSV;

int largePos, linkPos, bucketPos;

void moveSlow(Servo &s, int &pos, int to, int stepDelay) {
  int dir = (to > pos) ? 1 : -1;
  while (pos != to) {
    pos += dir;
    s.write(pos);
    delay(stepDelay);
  }
}

void moveOut(Pose p) {
  moveSlow(bucketSV, bucketPos, p.bucket, 5);
  moveSlow(linkSV,   linkPos,   p.link,   15);
  moveSlow(largeSV,  largePos,  p.large,  10);
}


void moveHome(Pose p) {
  moveSlow(bucketSV, bucketPos, p.bucket, 5);
  moveSlow(linkSV,   linkPos,   p.link,   15);
  moveSlow(largeSV,  largePos,  p.large,  20);
}

void armInit() {
  largePos  = HOME.large;
  linkPos   = HOME.link;
  bucketPos = HOME.bucket;

  largeSV.write(largePos);
  linkSV.write(linkPos);
  bucketSV.write(bucketPos);

  largeSV.attach(largeSV_PIN);
  linkSV.attach(linkSV_PIN);
  bucketSV.attach(bucketSV_PIN);

  delay(1000);
}

void boxdrop(Pose p){
  moveSlow(linkSV,   linkPos,   p.link,   15);
  moveSlow(largeSV,  largePos,  p.large,  20);
  moveSlow(bucketSV, bucketPos, p.bucket, 5);
}

void armToPickup() {
  moveOut(PICKUP);
}


void armToHome() {
  moveHome(HOME);
}