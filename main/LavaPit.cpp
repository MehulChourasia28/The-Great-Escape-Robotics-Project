#include "LavaPit.h"

const int16_t BASE_SPEED = 400;  // Base motor speed (max ±800)

// Initialize the LavaPit hardware: set up I²C for the motor controllers and attach servos
void setupLava() {
  // Start I²C and reset both Motoron controllers
  Wire.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // Attach servo1 to pin 53 and servo2 to pin 52
  servo1.attach(53);
  servo2.attach(52);
}

// Primary loop for the LavaPit challenge: point both servos horizontally and run motors forward
void loopLava() {
  // Position both servos around 100° to hold them roughly horizontal
  servo1.write(100);
  servo2.write(100);

  // Spin all four motors at speed 500
  mc1.setSpeed(1, 500);
  mc1.setSpeed(2, 500);
  mc2.setSpeed(1, 500);
  mc2.setSpeed(2, 500);
}
