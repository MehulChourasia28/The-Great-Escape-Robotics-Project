#include "gyro.h"

/* SPI pin assignments */
constexpr uint8_t MOSI_PIN = 11;  // Master Out, Slave In
constexpr uint8_t MISO_PIN = 12;  // Master In, Slave Out
constexpr uint8_t SCK_PIN  = 13;  // Serial Clock
constexpr uint8_t CS_PIN   = 10;  // Chip Select for the gyro

/* Gyro-specific constants */
static const float SENS = 0.07f;               // Sensitivity: 0.07 °/s per LSB (70 mdps/LSB)
static const float THRESH = 5.0f;               // If rotation > 5 °/s, consider it “not stationary”
static const uint32_t STATIC_US = 2'000'000UL;  // 2 million µs = 2 seconds

/* Calibration and integration state */
static int32_t xBias = 0, yBias = 0, zBias = 0;  // Raw LSB offsets for each axis
static float ax = 0.0f, ay = 0.0f, az = 0.0f;    // Integrated angles in degrees
static uint32_t tLast = 0;                      // Timestamp of last read, for Δt

/* Helper to write a byte to the gyro over SPI */
static void wr(uint8_t addr, uint8_t data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(addr & 0x7F);   // Ensure MSB=0 to indicate write
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

/* Helper to read a byte from the gyro over SPI */
static uint8_t rd(uint8_t addr) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(addr | 0x80);   // Set MSB=1 for a read operation
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

/*
  Read the raw X, Y, Z registers in one burst.
  For many 3-axis SPI gyros (e.g., L3GD20), the OUT_X_L register is at 0x28.
  Setting bit-6 (auto-increment) lets us read all 6 bytes consecutively.
*/
static void readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x28 | 0x80 | 0x40);  // 0x80 = read, 0x40 = auto-increment
  uint8_t xl = SPI.transfer(0x00);
  uint8_t xh = SPI.transfer(0x00);
  uint8_t yl = SPI.transfer(0x00);
  uint8_t yh = SPI.transfer(0x00);
  uint8_t zl = SPI.transfer(0x00);
  uint8_t zh = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
}

/*
  Wait until the gyro has been still for about 2 seconds, then calculate
  the average raw values for X, Y, Z to use as zero-bias. Resets integrated angles too.
*/
void gyroCalibrateIfStatic() {
  uint32_t startTime = micros();
  int64_t sx = 0, sy = 0, sz = 0;
  uint32_t n = 0;

  while (true) {
    int16_t xi, yi, zi;
    readXYZ(xi, yi, zi);
    float gx = xi * SENS;
    float gy = yi * SENS;
    float gz = zi * SENS;

    // If any axis is moving faster than THRESH, reset the wait
    if (fabsf(gx) > THRESH || fabsf(gy) > THRESH || fabsf(gz) > THRESH) {
      startTime = micros();
      sx = sy = sz = 0;
      n = 0;
    } else {
      sx += xi;
      sy += yi;
      sz += zi;
      ++n;
      // If we've been still for STATIC_US, break out
      if ((int32_t)(micros() - startTime) >= (int32_t)STATIC_US) {
        break;
      }
    }
    delay(5); // Sample around 200 Hz
  }

  // Compute integer biases from the sums
  xBias = (int32_t)(sx / n);
  yBias = (int32_t)(sy / n);
  zBias = (int32_t)(sz / n);

  // Zero out the integrated angles
  ax = ay = az = 0.0f;
  // Start timing for the next integration step
  tLast = micros();

  Serial.println(F("# static calibration done"));
}

/*
  Initialize the gyro over SPI, set up registers, and perform a one-shot calibration.
  Call this once from setup().
*/
void setupGyro() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Deactivate chip select

  SPI.begin();
  // SPI at 1 MHz, MSB first, MODE0
  SPI.beginTransaction(SPISettings(1'000'000UL, MSBFIRST, SPI_MODE0));

  // CTRL_REG1 (0x20): 0xCF sets 100 Hz output data rate, all axes enabled
  wr(0x20, 0xCF);

  // CTRL_REG4 (0x23): 0x30 sets full-scale ±2000 dps
  wr(0x23, 0x30);

  delay(100);  // Give the gyro time to power up

  Serial.println(F("ax,ay,az"));  // Print header for any logging

  // Do the one-time static calibration
  gyroCalibrateIfStatic();
}

/*
  Read the gyro, remove the zero-bias, convert to °/s, then integrate over Δt
  to update the running angles (ax, ay, az in degrees). You can call this
  at any interval; it uses micros() internally to calculate Δt.
*/
void gyroReadAngles(float& out_ax, float& out_ay, float& out_az) {
  // Grab raw readings and subtract biases
  int16_t xi, yi, zi;
  readXYZ(xi, yi, zi);

  float gx = ((float)xi - (float)xBias) * SENS;
  float gy = ((float)yi - (float)yBias) * SENS;
  float gz = ((float)zi - (float)zBias) * SENS;

  // Figure out elapsed time in seconds
  uint32_t now = micros();
  float dt = (now - tLast) * 1e-6f;
  tLast = now;

  // Integrate angular rates to update angles
  ax += gx * dt;
  ay += gy * dt;
  az += gz * dt;

  // Return the integrated angles
  out_ax = ax;
  out_ay = ay;
  out_az = az;
}
