#include "WallFollower.h"

// Moving-average filters (10 readings each) for the left and front distance sensors
Average<float> ave(10);
Average<float> ave1(10);

// Which analog pin is our left-facing distance sensor plugged into?
const int DIST_PIN = A1;

// How far we want to stay from the left wall (in cm) and our PD gains
const float DESIRED_CM = 6.0f;   // Aim for about 6 cm away
const float Kp_wall    = 1.0f;   // Proportional gain (tweak this as needed)
const float Kd_wall    = 0.2f;   // Derivative gain (smooths out jerks)

// Remember the last error so we can calculate the derivative term
static float prevError = 0.0f;


// setupWall(): run this once in setup()
void setupWall() {
  Serial.begin(9600);
  // Give Serial a moment to wake up (up to about 2 seconds)
  while (!Serial && millis() < 2000);

  // Wire up I²C and reset both Motoron controllers so they're ready to drive
  Wire.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  Serial.println(F("Wall-follower starting..."));
}

// loopWall(): run this repeatedly in loop()
void loopWall() {
  // --- Read the front sensor (on A0) and smooth it ---
  float mv0 = analogRead(A0) * (5000.0f / 1023.0f);
  mv0 = max(mv0, 1.0f);  // Make sure we don't feed powf() a value <1
  float d0 = 4600.5f * powf(mv0, -0.94f);
  if (isfinite(d0)) {
    ave1.push(d0);
  }
  float distFront = ave1.mean() - 2.0f;  // Subtract the 2 cm offset

  // If something is too close in front (<7.5 cm), spin right until it's clear
  if (distFront < 7.5f) {
    float d_front;
    do {
      // 1) Spin right in place: left wheels backward, right wheels forward
      mc1.setSpeed(1, -400);
      mc1.setSpeed(2,  400);
      mc2.setSpeed(1, -400);
      mc2.setSpeed(2,  400);

      // 2) Read front sensor again and update the average
      float mv_f = analogRead(A0) * (5000.0f / 1023.0f);
      mv_f = max(mv_f, 1.0f);
      float dF = 4600.5f * powf(mv_f, -0.94f);
      if (isfinite(dF)) {
        ave1.push(dF);
      }
      d_front = ave1.mean() - 2.0f;

      // 3) Small pause so motor movement and sensor reading stabilize
      delay(10);
    } while (d_front < 20.0f);

    // 4) Stop turning as soon as there’s enough clearance in front
    mc1.setSpeed(1, 0);
    mc1.setSpeed(2, 0);
    mc2.setSpeed(1, 0);
    mc2.setSpeed(2, 0);

    // Skip the rest of wall-following logic this cycle
    return;
  }

  // --- Otherwise, do PD-based wall-following using the left sensor ---
  float mv = analogRead(DIST_PIN) * (5000.0f / 1023.0f);
  mv = max(mv, 1.0f);
  float d = 4600.5f * powf(mv, -0.94f);
  if (isfinite(d)) {
    ave.push(d);
  }
  float dist = ave.mean() - 2.0f;  // Apply that same –2 cm offset

  // Calculate error and derivative for PD control
  float error  = dist - DESIRED_CM;
  float dError = error - prevError;
  prevError    = error;
  float correction = (Kp_wall * error) + (Kd_wall * dError);

  // Compute left and right wheel speeds
  float rawL = (float)BASE_SPEED - correction;
  float rawR = (float)BASE_SPEED + correction;

  int16_t speedL = (int16_t)constrain(rawL, -800, 800);
  int16_t speedR = (int16_t)constrain(rawR, -800, 800);

  mc1.setSpeed(1, speedL);
  mc1.setSpeed(2, speedR);
  mc2.setSpeed(1, speedL);
  mc2.setSpeed(2, speedR);

  // Optional: print out some debug info to Serial
  Serial.print(F("DistL="));  Serial.print(dist,  1);
  Serial.print(F("  Err="));  Serial.print(error,2);
  Serial.print(F("  Corr=")); Serial.print(correction,2);
  Serial.print(F("  L="));    Serial.println(speedL);
}
