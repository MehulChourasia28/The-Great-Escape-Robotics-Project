#include "Steps.h"

// Two servo objects for controlling additional mechanisms
Servo servo1;
Servo servo2;

// Default speed for the DC motors (range is ±800)
const int16_t BASE_SPEED = 400;

// Called once inside Arduino’s setup()
void setupSteps() {
  // Start up I²C and reset both Motoron boards so they’re ready to go
  Wire.begin();
  mc1.reinitialize();
  mc1.disableCrc();
  mc1.clearResetFlag();
  mc2.reinitialize();
  mc2.disableCrc();
  mc2.clearResetFlag();

  // Hook up the servos: servo1 on pin 53, servo2 on pin 52
  servo1.attach(53);
  servo2.attach(52);
}

// Called over and over in Arduino’s loop()
// Replicates the same sequence of motor and servo moves as before
void loopSteps() {
  // --- Phase 1 ---
  // Run all DC motors at 300, tilt servo1 to 150°, and hold for 2 seconds
  mc1.setSpeed(1,  300);
  mc1.setSpeed(2,  300);
  mc2.setSpeed(1,  300);
  mc2.setSpeed(2,  300);
  servo1.write(150);
  delay(2000);

  // --- Phase 2 ---
  // Keep the motors at 300, move servo1 to 100°, servo2 to 150°, wait 2 seconds
  mc1.setSpeed(1,  300);
  mc1.setSpeed(2,  300);
  mc2.setSpeed(1,  300);
  mc2.setSpeed(2,  300);
  servo1.write(100);
  servo2.write(150);
  delay(2000);

  // --- Phase 3 ---
  // Same motor speed again, now set servo1 to 50° and servo2 to 70°, then pause for 2 seconds
  mc1.setSpeed(1,  300);
  mc1.setSpeed(2,  300);
  mc2.setSpeed(1,  300);
  mc2.setSpeed(2,  300);
  servo1.write(50);
  servo2.write(70);
  delay(2000);

  // After this, loopSteps() will run again from phase 1
}
