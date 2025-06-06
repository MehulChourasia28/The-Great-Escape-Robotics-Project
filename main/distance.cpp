#include "distance.h"
#include <math.h>

// Create a floating-point rolling-average buffer with a window size of 10
static Average<float> ave(10);

/*
  Call this from setup():
    - Starts Serial at 9600 baud so you can print debug info
    - No additional setup needed for the analog sensor here
*/
void setupDistance() {
  Serial.begin(9600);
  // Wait briefly until Serial is ready (optional but helpful)
  while (!Serial) { /* just waiting */ }
  Serial.println(F("distance: Serial started, rolling-average buffer initialized."));
}

/*
  Read the analog pin (e.g., A0), convert to millivolts, clamp to at least 1.0,
  calculate distance with the formula 4600.5 * mv^(–0.94), feed it into the
  rolling-average, and return the smoothed distance (minus a 2 cm offset).

  Example usage inside loop():
    float d = getDistance(A0);
    // d is the averaged distance in centimeters
*/
float getDistance(uint8_t pin) {
  // 1) Read raw analog value and convert to millivolts
  float mv = analogRead(pin) * (5000.0f / 1023.0f);

  // 2) Ensure mv is at least 1.0 to avoid issues with powf()
  mv = max(mv, 1.0f);

  // 3) Compute distance (in cm) using the sensor’s characteristic curve
  float d = 4600.5f * powf(mv, -0.94f);

  // 4) If the result is a valid number, add it to our rolling-average buffer
  if (isfinite(d)) {
    ave.push(d);
  }

  // 5) Return the current average distance, then subtract 2.0 cm just like before
  return ave.mean() - 2.0f;
}
