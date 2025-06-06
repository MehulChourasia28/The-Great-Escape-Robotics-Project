// -------- distance.h ------------
#ifndef DISTANCE_H
#define DISTANCE_H

#include <Arduino.h>
#include <Average.h>

/*
  Call this once in setup():
    - Starts Serial so you can see debug prints in your sketch
    - Sets up the rolling-average buffer for smoothing readings

  After that, in loop() just do:
    float d = getDistance(pin);
  to get an averaged distance in centimeters (with the same “−2 cm” offset as before).
*/
void setupDistance();

/*
  Reads the specified analog pin (e.g., A0), updates the rolling average,
  and returns the smoothed distance in cm. Formula used:
    4600.5 * mv^(−0.94)
  where mv = (analogRead(pin) * 5000.0 / 1023.0), and we clamp mv ≥ 1.0.
  Finally, we subtract 2 cm (just like the original sketch did) before returning.
*/
float getDistance(uint8_t pin);

#endif // DISTANCE_H
