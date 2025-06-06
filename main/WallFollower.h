// -------- WallFollower.h ------------
#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Average.h>
#include <math.h>
#include "LineFollowing.h"

// Rolling-average buffers (10 samples each) for the left and front distance sensors
extern Average<float> ave;  
extern Average<float> ave1; 

// Analog input pin where the left-facing distance sensor is connected (A1)
extern const int DIST_PIN;   // e.g., A1

// How far (in cm) we want to stay from the left wall, plus PD control gains
extern const float DESIRED_CM;  // e.g., 6.0 cm
extern const float Kp_wall;     // e.g., 20.0 (proportional gain)
extern const float Kd_wall;     // e.g., 0.0  (derivative gain)

// Base forward speed for the drive motors (range is ±800 on Motoron controllers)
extern const int16_t BASE_SPEED;

// Two MotoronI2C instances for controlling the drive motors (I²C addresses 0x10 and 0x14)
extern MotoronI2C mc1;
extern MotoronI2C mc2;

// Call this once from setup() to initialize everything needed for wall following
void setupWall();

// Call this repeatedly from loop() to read sensors, compute control, and drive motors
void loopWall();

#endif // WALLFOLLOWER_H
