// -------- gyro.h ------------
#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include <SPI.h>   // Needed for SPI.transfer() and related calls

/*
  Call this once in setup() to get the gyro talking over SPI,
  configure its registers, and do a quick “static” calibration
  so we know what zero-rate looks like.
*/
void setupGyro();

/*
  If the robot has been sitting still for about 2 seconds (≈2000 ms),
  this will calculate the gyro’s zero‐bias (xBias, yBias, zBias),
  zero out the integrated angles (ax, ay, az), and reset the timer.
  Blocks until the 2 s of stillness is detected.
*/
void gyroCalibrateIfStatic();

/*
  Reads the raw X, Y, and Z outputs from the gyro, subtracts out
  the biases we found during calibration, integrates over the time
  since the last call, and updates ax, ay, az (in degrees).

  You can call this at any frequency—the function uses micros()
  internally to figure out Δt.

  Example use:
    float ax, ay, az;
    gyroReadAngles(ax, ay, az);
*/
void gyroReadAngles(float& ax, float& ay, float& az);

#endif // GYRO_H
