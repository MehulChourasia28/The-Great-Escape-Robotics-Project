// -------- MotorServo.h ------------
#ifndef MOTORSERVO_H
#define MOTORSERVO_H

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>
#include "LavaPit.h"
#include "LineFollowing.h"

// Base speed for all four DC motors (±800 max)
extern const int16_t BASE_SPEED;

// Call once in Arduino’s setup()
void setupSteps();

// Call repeatedly in Arduino’s loop()
void loopSteps();

#endif // MOTORSERVO_H
