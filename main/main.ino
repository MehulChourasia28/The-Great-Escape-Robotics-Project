// -------- Main.ino ------------
#include <Arduino.h>
#include "LineFollowing.h"
#include "gyro.h"
#include "LavaPit.h"
#include "Steps.h"
#include "distance.h"
#include "WallFollower.h"

const int buttonPin = 13;
bool lastButtonState = HIGH;
bool active = true;

void setup() {
  // The Arduino core already calls init() before setup(), so we do not call init() again.
  Serial.begin(9600);

  pinMode(buttonPin, INPUT_PULLUP);

  setupLineFollowing();
  setupDistance();
  setupLava();
  setupSteps();
  setupWall();
}

void loop() {

  // Ramp
  mc1.setSpeed(1,500);
  mc1.setSpeed(2,500);

  while (getDistance(A0) > 7.5f){
    mc1.setSpeed(1,500);
    mc1.setSpeed(2,500);
  }

  while (getDistance(A0) < 20.0f){
    loopWall();
  }

  // Steps
  while (getDistance(A0) > 7.0f){
    loopSteps();
  }

  while (getDistance(A0) < 20.0f){
    loopWall();
  }

  loopSteps();
  loopSteps();
  loopSteps();

  while (true){
    loopWall();
  }
}
