# How-To Guide

This repository contains several independent “behavior modules” (Line Following, Wall Following, Lava Pit, Steps) plus a shared kill‐switch/remote‐stop utility. Each behavior is implemented in its own pair of `.h` and `.cpp` files. You can pick which behavior to run by simply calling its setup/loop functions from `Main.ino`.

> **Note:** All of these modules assume the same C++/Arduino toolchain. If you already have an Arduino (or ESP32, etc.) project, just drop these files into your sketch folder and then create a `Main.ino` that picks which behavior to run.

---

## Repository Layout

```
/
├── ControlLogic.h
├── ControlLogic.cpp
├── LineFollowing.h
├── LineFollowing.cpp
├── WallFollower.h
├── WallFollower.cpp
├── LavaPit.h
├── LavaPit.cpp
├── Steps.h
├── Steps.cpp
├── distance.h
├── distance.cpp
├── gyro.h
├── gyro.cpp
└── Main.ino      ← Example “master” sketch showing how to initialize and call each module
```

---

## Module-by-Module Explanation

Below is a brief summary of what each file does and how it fits into the system.

---

### 1. `ControlLogic.h` / `ControlLogic.cpp`

**Purpose:**

* Provides a **single, reusable “kill switch”/remote‐stop** abstraction that any behavior can call.
* Combines:

  1. A physical pushbutton (reads from a digital input) to toggle the motor(s) on/off.
  2. A simple UDP listener (“Stop” command) that, if received, will also cut motor output immediately.
* Exposes two functions:

  ```cpp
  void setupControl();
  void loopControl();
  ```

  These should be called (respectively) from your `setup()` and `loop()` in `Main.ino`.
* Internally manages a boolean flag (`motorRunning`) and a “wifiStopped” flag. If either route sets it “off,” all motor‐drive calls are zeroed until the next button press.

**Key Details:**

* Requires `<WiFi.h>` and `<WiFiUdp.h>` (ESP32/ESP8266 or similar).
* Instantiates a single `MotoronI2C mc(0x10);` and repeatedly (every `CMD_INTERVAL`) re‐issues `mc.setSpeed(…)` if `motorRunning == true && wifiStopped == false`.
* Physical‐button logic uses `pinMode(buttonPin, INPUT_PULLUP)` and edge‐detects a HIGH→LOW transition to toggle.
* UDP logic listens on a fixed port (55500) and stops motors whenever a packet containing exactly `"Stop"` arrives.

> If you do not need Wi-Fi/UDP on a given board, simply remove/comment out `#include <WiFi.h>` / `WiFi.begin(…)` lines in `ControlLogic.cpp`. The physical button will still function correctly.

---

### 2. `LineFollowing.h` / `LineFollowing.cpp`

**Purpose:**

* Implements a classic **two‐row reflectance‐sensor line‐follower** that:

  1. Calibrates two rows of 9 RC sensors each (`backPins[]` and `frontPins[]`).
  2. In each loop, reads both rows (maps raw discharge times into 0–1000), computes a weighted centroid for each row, applies a PD controller plus an angle correction (front vs. back row), and drives four motors via two `MotoronI2C` controllers (`mc1` at 0x10, `mc2` at 0x14).
  3. Detects when no line is under either row (both centroids’ denominators are zero) and, in that case, pivots 90° in place (right or left depending on which direction the front centroid is relative to the back centroid).

**Public API:**

```cpp
void setupLineFollowing();
void loopLineFollowing();
```

* **`setupLineFollowing()`**

  * Waits briefly for Serial, turns on an LED, resets/initializes both MotoronI2C boards, then runs a two‐stage calibration (`calibrate(backPins, …)` and `calibrate(frontPins, …)`), saving each sensor’s min/max discharge times.
  * Prints “Calibrating back row…”, “Calibrating front row…”, and “Calibration complete.” to Serial.
* **`loopLineFollowing()`**

  * Called continuously:

    1. Reads both rows’ raw values via `readRow(…)`.
    2. Calculates `posB` (back centroid) and `posF` (front centroid) in \[0..8000] scale.
    3. If both rows see nothing (denB == 0 && denF == 0), calls `pivot(turnRight)` to spin \~90° in place, then returns.
    4. Otherwise, runs a PD controller (Kp, Kd) based on `(posB – center)`, adds an angle term `theta = atan2(dx, rowSpacing)`, and drives left/right wheel speeds accordingly.
    5. Dumps debugging info to Serial.

**How to Use in Main:**

```cpp
#include "LineFollowing.h"

void setup() {
  setupLineFollowing();
}

void loop() {
  loopLineFollowing();
}
```

* If you want to preserve a kill‐switch abstraction, call `loopControl()` **before** `loopLineFollowing()` in your main `loop()`.

---

### 3. `WallFollower.h` / `WallFollower.cpp`

**Purpose:**

* Implements a **PD‐based “left‐wall follower”** with an obstacle‐detection pivot.
* Uses two analog distance sensors (front is analog pin A0, left is analog pin A1) through a rolling‐average filter (window size = 10, implemented in `distance.cpp`).
* If the **front sensor** reading (averaged) ever falls below a threshold (7.5 cm), it pivots right in place until the front reading exceeds 20 cm. Once clear, it immediately stops motors and returns (skipping that cycle’s wall‐following).
* Otherwise (front is clear), it reads the **left sensor**, computes `error = distLeft – DESIRED_CM`, runs a PD controller `(Kp_wall * error + Kd_wall * dError)`, and sets left/right motor speeds at `(BASE_SPEED ∓ correction)`.

**Public API:**

```cpp
void setupWall();
void loopWall();
```

* **`setupWall()`**

  * Initializes Serial at 9600 baud (prints “Wall-follower starting…”), resets both MotoronI2C boards, and returns.
* **`loopWall()`**

  * Every iteration:

    1. Read front‐sensor: convert `analogRead(A0)` → millivolts, clamp ≥ 1, compute distance via `4600.5 * mv^(–0.94)`, push into `ave1` (window=10), compute `distFront = ave1.mean() – 2.0`.
    2. If `distFront < 7.5 cm`, then:

       * Spin right (left wheels ←, right wheels →) until `ave1.mean() – 2.0 ≥ 20`, then stop all motors and `return`.
    3. Otherwise, read left sensor on A1 in the same way (push into `ave`, then `distLeft = ave.mean() – 2.0`).

       * Compute `error = distLeft – DESIRED_CM` (DESIRED\_CM=6.0 cm by default), `dError = error – prevError`, `correction = Kp_wall * error + Kd_wall * dError`.
       * Left wheel = `BASE_SPEED – correction`; Right wheel = `BASE_SPEED + correction`; clip each to \[–800..800] and call `mc1.setSpeed(1, …)`, `mc1.setSpeed(2, …)`, `mc2.setSpeed(1, …)`, `mc2.setSpeed(2, …)`.
    4. Serial‐print debug line: `DistL=… Err=… Corr=… L=…`.

**How to Use in Main:**

```cpp
#include "WallFollower.h"

void setup() {
  setupWall();
}

void loop() {
  loopWall();
}
```

* Again, if you want the physical/UDP kill-switch, call `loopControl()` before `loopWall()`.

---

### 4. `LavaPit.h` / `LavaPit.cpp`

**Purpose:**

* A **very minimal “lava‐pit” runner**. On each loop:

  1. Tells two servos (`servo1` on pin 53, `servo2` on pin 52) to move to 100° (roughly horizontal).
  2. Drives all four motors forward at a fixed speed of 500.

**Public API:**

```cpp
void setupLava();
void loopLava();
```

* **`setupLava()`**

  * Initializes I²C, resets Motoron boards (`mc1` & `mc2`), and calls `servo1.attach(53)` and `servo2.attach(52)`.
* **`loopLava()`**

  * `servo1.write(100)` and `servo2.write(100)`, then `mc1.setSpeed(1, 500)`, `mc1.setSpeed(2, 500)`, `mc2.setSpeed(1, 500)`, `mc2.setSpeed(2, 500)`.

No internal debug prints. Typically you leave it running until the kill‐switch or UDP “Stop” cuts the motors.

**How to Use in Main:**

```cpp
#include "LavaPit.h"

void setup() {
  setupLava();
}

void loop() {
  loopLava();
}
```

* Interleave with `loopControl()` if you want to preserve a kill‐switch.

---

### 5. `Steps.h` / `Steps.cpp`

**Purpose:**

* Implements a **3‐phase “sequence”** (called “Steps”) in which each phase lasts 2 s:

  1. Phase 1 (t= 0..2 s): Motors = 300, `servo1 = 150°`.
  2. Phase 2 (t= 2..4 s): Motors = 300, `servo1 = 100°`, `servo2 = 150°`.
  3. Phase 3 (t= 4..6 s): Motors = 300, `servo1 = 50°`, `servo2 = 70°`.
     Then go back to Phase 1, repeating indefinitely.

**Public API:**

```cpp
void setupSteps();
void loopSteps();
```

* **`setupSteps()`**

  * Resets both MotoronI2C boards, then `servo1.attach(53)` and `servo2.attach(52)`.
* **`loopSteps()`**

  * Runs the three phases in order, each ending in `delay(2000);`. After Phase 3, the function simply returns—so, on the next Arduino `loop()`, it runs Phase 1 again.

**How to Use in Main:**

```cpp
#include "Steps.h"

void setup() {
  setupSteps();
}

void loop() {
  loopSteps();
}
```

* Combine with `loopControl()` to allow the pushbutton/UDP to cut power mid‐sequence.

---

### 6. `distance.h` / `distance.cpp`

**Purpose:**

* Provides a **rolling‐average wrapper** for reading a single analog pin and converting to a distance in cm.
* Formula used:

  ```
  mv = analogRead(pin) * (5000 / 1023)
  distance = 4600.5 * mv^(−0.94)
  return (average(distance) − 2.0)
  ```
* Internally uses `Average<float> ave(10)` (window size 10) from the `Average` template.

**Public API:**

```cpp
void setupDistance();
float getDistance(uint8_t pin);
```

* **`setupDistance()`**

  * `Serial.begin(9600)` and prints a single startup message.
* **`getDistance(pin)`**

  * Reads `analogRead(pin)`, maps to millivolts, clamps ≥ 1.0, computes `d = 4600.5 * pow(mv, -0.94)`, pushes into `ave` if finite, returns `ave.mean() – 2.0`.

**How It’s Used:**

* Imported by `WallFollower.cpp`. That file calls `getDistance(A0)` for the front‐facing sensor and `getDistance(A1)` (via the rolling‐average) for the left‐facing sensor.

---

### 7. `gyro.h` / `gyro.cpp`

**Purpose:**

* Low‐level SPI driver for a 3‐axis gyroscope (e.g. L3GD20). Supports:

  1. **One‐shot “static” calibration**: waits until no rotation > 5 °/s for 2 s, then computes `(xBias, yBias, zBias)` from raw LSBs, zeros the integrated angles, and resets timestamp.
  2. **Continuous angle integration**: `gyroReadAngles(float& ax, float& ay, float& az)` reads raw X/Y/Z, subtracts the bias, multiplies by sensitivity (0.07 °/s per LSB), computes Δt via `micros()`, integrates into `ax/ay/az` (in degrees), and returns via reference.

**Public API:**

```cpp
void setupGyro();
void gyroCalibrateIfStatic();
void gyroReadAngles(float& ax, float& ay, float& az);
```

* **`setupGyro()`**

  * Initializes pins for SPI, configures registers (`CTRL_REG1 = 0xCF` for 100 Hz & all axes enabled, `CTRL_REG4 = 0x30` for ±2000 dps), then calls `gyroCalibrateIfStatic()`.
* **`gyroCalibrateIfStatic()`**

  * Repeatedly calls `readXYZ(xi, yi, zi)`, converts to °/s, resets as soon as any axis > 5 °/s. Once it has stayed under 5 °/s for 2 s, it averages raw LSBs to compute `xBias, yBias, zBias`; zeros out `ax/ay/az`; sets `tLast = micros()`.
* **`gyroReadAngles(ax, ay, az)`**

  * `readXYZ(xi, yi, zi)`, subtracts `(xBias, yBias, zBias)`, scales by sensitivity 0.07 °/s, computes Δt, integrates.

**How It’s Used:**

* Not automatically used by any other module in this repo—only provided for future expansion. If you want to incorporate gyro‐based heading corrections, call `setupGyro()` in your init and then call `gyroReadAngles(ax, ay, az)` once per loop to maintain `ax/ay/az`.

---

## How to Combine Everything in `Main.ino`

Below is an illustrative `Main.ino` that demonstrates:

1. Always running the **ControlLogic** kill‐switch/UDP listener.
2. Presenting a “menu” on Serial so the user can choose one of the four behaviors.
3. Calling the appropriate `setupX()` exactly once, then continuously calling `loopX()`.

```cpp
// -------- Main.ino ------------

#include <Arduino.h>

// Step 1: Include each module’s header so we can pick them at runtime
#include "ControlLogic.h"
#include "LineFollowing.h"
#include "WallFollower.h"
#include "LavaPit.h"
#include "Steps.h"

enum BehaviorMode {
  NONE  = 0,
  LINE  = 1,
  WALL  = 2,
  LAVA  = 3,
  STEPS = 4
};

static BehaviorMode selectedMode = NONE;
static bool         hasInitialized = false;

// Print a simple menu over Serial
void printMenu() {
  Serial.println(F("================================"));
  Serial.println(F("  Select Behavior Mode:"));
  Serial.println(F("   1 → Line Following"));
  Serial.println(F("   2 → Wall Following"));
  Serial.println(F("   3 → Lava Pit"));
  Serial.println(F("   4 → Steps"));
  Serial.println(F("  (Type 1/2/3/4 and press Enter)"));
  Serial.println(F("================================"));
}

void setup() {
  // 1) Initialize Serial at 115200 so we can show the menu
  Serial.begin(115200);
  delay(300);

  Serial.println(F("\n\n### Main: Starting Setup() ###"));

  // 2) Always initialize the kill-switch & UDP “Stop” logic first
  setupControl();
  Serial.println(F("Main: ControlLogic initialized."));

  // 3) Print the menu over Serial
  printMenu();
  Serial.println(F("Awaiting user selection..."));
}

void loop() {
  // A) Continuously run the kill-switch + UDP “Stop” checks
  loopControl();

  // B) If we haven’t picked a mode yet, wait for Serial input
  if (!hasInitialized) {
    if (Serial.available()) {
      char c = Serial.read();
      switch (c) {
        case '1': selectedMode = LINE;  break;
        case '2': selectedMode = WALL;  break;
        case '3': selectedMode = LAVA;  break;
        case '4': selectedMode = STEPS; break;
        default:  /* ignore anything else */ break;
      }
      if (selectedMode != NONE) {
        Serial.print(F("Main: You selected mode "));
        Serial.println((int)selectedMode);
      }
    }

    // C) Once a valid key is pressed, call the corresponding setupX()
    if (selectedMode == LINE) {
      Serial.println(F("Main: Initializing Line Following..."));
      setupLineFollowing();
      hasInitialized = true;
      Serial.println(F("Main: Entering Line Following loop()..."));
    }
    else if (selectedMode == WALL) {
      Serial.println(F("Main: Initializing Wall Following..."));
      setupWall();
      hasInitialized = true;
      Serial.println(F("Main: Entering Wall Following loop()..."));
    }
    else if (selectedMode == LAVA) {
      Serial.println(F("Main: Initializing Lava Pit..."));
      setupLava();
      hasInitialized = true;
      Serial.println(F("Main: Entering Lava Pit loop()..."));
    }
    else if (selectedMode == STEPS) {
      Serial.println(F("Main: Initializing Steps..."));
      setupSteps();
      hasInitialized = true;
      Serial.println(F("Main: Entering Steps loop()..."));
    }

    // If still NONE, keep waiting
    return;
  }

  // D) Once `hasInitialized == true`, simply run the chosen behavior’s loop
  switch (selectedMode) {
    case LINE:
      loopLineFollowing();
      break;
    case WALL:
      loopWall();
      break;
    case LAVA:
      loopLava();
      break;
    case STEPS:
      loopSteps();
      break;
    default:
      // Should never happen, because we set `hasInitialized` only after a valid mode
      break;
  }
}
```

### How `Main.ino` Integrates Each Module

1. **`setupControl()` / `loopControl()`**

   * Always called first. Ensures your physical pushbutton and remote UDP “Stop” are active no matter which behavior is running.

2. **Serial Menu**

   * On boot, you see a menu (via `Serial.println`) asking for `1`, `2`, `3`, or `4`.
   * As soon as you type a valid character and press Enter, `selectedMode` is set, and the corresponding `setupX()` is invoked exactly once.

3. **Switching to the Chosen Behavior**

   * After `setupX()`, we set `hasInitialized = true`, so future calls to `loop()` simply call `loopX()` every iteration (always preceded by `loopControl()`).
   * This pattern avoids reinitializing the chosen module on every iteration.

4. **Kill‐Switch & Remote “Stop”**

   * Because `loopControl()` always runs at the top of `loop()`, any button press or incoming UDP “Stop” will cut all motor outputs—even if your chosen behavior’s `loopX()` is trying to drive the motors.
   * Once you hit the button again, the motors resume (unless they were stopped by UDP after that).

---

## How to Add or Update a Behavior Module

1. **Create New `MyBehavior.h` / `MyBehavior.cpp`.**

   * Follow the same pattern as the existing modules: define `void setupMyBehavior();` and `void loopMyBehavior();`.
   * If you need helper files (like `distance.cpp` for range sensors), add them to the same folder.
2. **Include in `Main.ino`.**

   * Add `#include "MyBehavior.h"` at the top.
   * Extend the `BehaviorMode` enum and the menu‐printing function to list your new mode with a new number (e.g. change `enum BehaviorMode { …, STEPS = 4, MYBEHAVIOR = 5 };`).
   * In `loop()`, add a new `else if (selectedMode == MYBEHAVIOR) { setupMyBehavior(); … }` and a `case MYBEHAVIOR: loopMyBehavior(); break;` in the final `switch` block.

That’s it—**no need to change any of the existing modules**. Each module only cares about its own pins, sensors, PD gains, etc. The `Main.ino` is the only place you need to “wire together” which behavior is active at runtime.

---

## Quick Start Checklist

1. **Clone/Download this repository** into your Arduino sketch folder.
2. **Open `Main.ino`** in the Arduino IDE (or VSCode/PlatformIO). You should see all the `.h` and `.cpp` files in the same project.
3. **Select the Correct Board** (ESP32 if you want UDP, or any Arduino if you only need the physical button).
4. **Compile & Upload**.
5. **Open Serial Monitor** at **115200 baud**. You’ll see:

   ```
   ========================================
     Select Behavior Mode:
      1 → Line Following
      2 → Wall Following
      3 → Lava Pit
      4 → Steps
   (Type 1/2/3/4 and press Enter)
   ========================================
   ```
6. **Type `1`, `2`, `3`, or `4`** + Enter.

   * For **1 (Line Following)** or **3 (Lava Pit)** or **4 (Steps)**: leave the Serial Monitor at **115200**.
   * For **2 (Wall Following)**: once you see “Wall-follower starting…”, switch your Serial Monitor to **9600** to see its debug prints.
7. **Use the Physical Button** or send a UDP packet to port 55500 containing exactly the ASCII text `Stop` (no newline) to pause/resume motors at any time, regardless of which behavior is running.
