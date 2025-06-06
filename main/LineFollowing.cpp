// LineFollowing.cpp - Implements the line-following behavior for the robot.
#include "LineFollowing.h"

// Sensor pin assignments for back and front arrays
const uint8_t backPins[9]   = {2,  3,  4,  5,  8,  9,  10, 11, 12};
const uint8_t frontPins[9]  = {41, 42, 43, 44, 45, 46, 47, 48, 49};

// LED indicator pin
const uint8_t LED_PIN       = 35;

// Timing constants for RC sensor readings
const uint16_t TIMEOUT_US   = 3000;   // Maximum time to wait for discharge
const uint16_t CALIB_SAMPLES = 3000;  // Number of samples to collect during calibration

// Arrays to store calibration minimums and maximums (populated in setup)
uint16_t minBack[9], maxBack[9];
uint16_t minFront[9], maxFront[9];

// Control gains and sensor geometry parameters
const float Kp           = 0.105;  // Proportional gain
const float Kd           = 0.005;  // Derivative gain
const float Ka           = 2.4;    // Angle correction gain
const float sensorPitch  = 10.0;   // Distance between sensors (mm)
const float rowSpacing   = 30.0;   // Distance between front and back sensor rows (mm)

// Pivoting parameters for 90° turns
const int16_t turnSpeed    = 400;   // Motor speed during pivot
const uint16_t turn90Time  = 1000;  // Approximate time (ms) to rotate 90°

// Motor controllers on the I²C bus (addresses 0x10 and 0x14)
MotoronI2C mc1(0x10), mc2(0x14);


// Read a single RC (reflectance) sensor by charging the pad, then timing how long it takes to discharge
uint16_t readRC(uint8_t pin) {
  // Charge the sensor capacitor
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);

  // Switch to input and measure how long until the pin goes LOW
  pinMode(pin, INPUT);
  uint32_t start = micros();
  while (digitalRead(pin)) {
    if (micros() - start > TIMEOUT_US) {
      // If it never discharges, return the timeout value
      return TIMEOUT_US;
    }
  }
  return micros() - start;
}


// Read all 9 sensors in one row, then normalize each raw reading to a 0–1000 scale using calibration data
void readRow(const uint8_t pins[], uint16_t raw[],
             uint16_t mn[], uint16_t mx[]) {
  // First, get the raw charge-time for each sensor
  for (uint8_t i = 0; i < 9; i++) {
    raw[i] = readRC(pins[i]);
  }

  // Now map each raw value into the range [0, 1000] based on the recorded min/max
  for (uint8_t i = 0; i < 9; i++) {
    int32_t span = mx[i] - mn[i];
    if (span <= 0) span = 1;  // Prevent division by zero
    int32_t d = raw[i] - mn[i];
    d = constrain(d, 0, span);
    raw[i] = (uint16_t)((d * 1000) / span);
  }
}


// Calibrate one row of 9 sensors by capturing the minimum and maximum discharge times over many samples
void calibrate(const uint8_t pins[], uint16_t mn[], uint16_t mx[]) {
  // Initialize min to max possible, max to zero
  for (uint8_t i = 0; i < 9; i++) {
    mn[i] = TIMEOUT_US;
    mx[i] = 0;
  }

  // Take a lot of readings so we see the full range of each sensor
  for (uint16_t t = 0; t < CALIB_SAMPLES; t++) {
    for (uint8_t i = 0; i < 9; i++) {
      uint16_t v = readRC(pins[i]);
      mn[i] = min(mn[i], v);
      mx[i] = max(mx[i], v);
    }
    delay(5);  // Short pause between samples
  }
}


// Spin in place by roughly 90°. If turnRight is true, rotate clockwise; otherwise, rotate counterclockwise.
void pivot(bool turnRight) {
  int16_t dir = turnRight ? +1 : -1;
  mc1.setSpeed(1,  dir * turnSpeed);
  mc1.setSpeed(2, -dir * turnSpeed);
  mc2.setSpeed(1,  dir * turnSpeed);
  mc2.setSpeed(2, -dir * turnSpeed);

  delay(turn90Time);  // Wait until roughly 90° is completed

  // Stop the motors once done
  mc1.setSpeed(1, 0);
  mc1.setSpeed(2, 0);
  mc2.setSpeed(1, 0);
  mc2.setSpeed(2, 0);
}


// Set up sensors, motor controllers, and run the calibration routine
void setupLineFollowing() {
  // Wait up to 2 seconds for the Serial interface to become available
  while (!Serial && millis() < 2000);

  // Turn on the LED so we know calibration is starting
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Initialize I²C and motor controllers
  Wire.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // Calibrate the back sensor row first
  Serial.println("Calibrating back row...");
  calibrate(backPins, minBack, maxBack);

  // Then calibrate the front sensor row
  Serial.println("Calibrating front row...");
  calibrate(frontPins, minFront, maxFront);

  Serial.println("Calibration complete.");
  delay(1000);  // Let the user see the final message before starting
}


// The main loop that continuously reads sensors, computes errors, and drives the motors
void loopLineFollowing() {
  static int16_t prevErr = 0;            // Remember the last error for derivative term
  const int16_t center    = 4000;        // Ideal centroid position when perfectly on line
  const int16_t baseSpeed = 400;         // Base motor speed while following

  // --- 1) Read both rows of sensors ---
  uint16_t rawB[9], rawF[9];
  readRow(backPins,  rawB,  minBack,  maxBack);
  readRow(frontPins, rawF,  minFront, maxFront);

  // --- 2) Compute the weighted centroid (position) of the line for each row ---
  uint32_t numB = 0, denB = 0;
  uint32_t numF = 0, denF = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (rawB[i] > 50) {
      numB += rawB[i] * i * 1000;
      denB += rawB[i];
    }
    if (rawF[i] > 50) {
      numF += rawF[i] * i * 1000;
      denF += rawF[i];
    }
  }
  // If no sensor sees the line, default to center
  uint16_t posB = denB ? (numB / denB) : center;
  uint16_t posF = denF ? (numF / denF) : center;

  // --- 3) Check for a corner or end-of-line: both rows see nothing → turn in place ---
  if (denB == 0 && denF == 0) {
    // Decide turn direction based on difference between front and back centroids
    bool turnRight = ((int32_t)posF - posB) > 0;
    pivot(turnRight);
    prevErr = 0;  // Reset derivative term
    return;
  }

  // --- 4) PD control plus angle correction based on front vs. back row ---
  int16_t err   = (int16_t)posB - center;     // How far off-center we are (back row)
  int16_t dErr  = err - prevErr;              // Change in error since last loop
  prevErr       = err;
  float pdCorr  = (Kp * err) + (Kd * dErr);   // Standard PD correction

  // Compute the angular offset between front and back row centroids
  float dx      = (float)(posF - posB) * (sensorPitch / 1000.0);  // Convert index difference to mm
  float theta   = atan2(dx, rowSpacing);  // Angle between rows

  float corr    = pdCorr + (Ka * theta);  // Total correction (PD + angle)

  // --- 5) Calculate individual wheel speeds and send to motors ---
  int16_t speedL = constrain(baseSpeed + corr, -800, 800);
  int16_t speedR = constrain(baseSpeed - corr, -800, 800);
  mc1.setSpeed(1, speedL);
  mc1.setSpeed(2, speedR);
  mc2.setSpeed(1, speedL);
  mc2.setSpeed(2, speedR);

  // --- 6) Optional: Print debug information to Serial monitor ---
  Serial.print("B:");    Serial.print(posB);
  Serial.print(" F:");   Serial.print(posF);
  Serial.print(" Err:"); Serial.print(err);
  Serial.print(" θ:");   Serial.print(theta * 57.3, 2);  // Convert radians to degrees
  Serial.print(" L:");   Serial.print(speedL);
  Serial.print(" R:");   Serial.println(speedR);

  delay(20);  // Small delay for loop timing
}
