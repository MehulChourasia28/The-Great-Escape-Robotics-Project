// -------- LavaPit.h ------------
#ifndef LAVAPIT_H
#define LAVAPIT_H

#include <Arduino.h>
#include <Servo.h>
#include <Motoron.h>
#include "LineFollowing.h"

extern Servo servo1;
extern Servo servo2;

// Call this once in Arduino’s setup()
void setupLava();

// Call this repeatedly in Arduino’s loop()
void loopLava();

#endif // LAVAPIT_H
