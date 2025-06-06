#include "ControlLogic.h"

// UDP object and buffer
static WiFiUDP udp;
static char udpBuffer[BUF_SIZE];

// Button state tracking
static bool lastButtonState = HIGH;
static bool motorRunning    = true;
static bool wifiStopped     = false;

// Timing for UDP command refresh
static unsigned long lastCommandTime = 0;


void setupControl() {
  // Serial for debug
  Serial.begin(115200);
  delay(1000);
  Serial.println("ControlLogic: setup started");

  // Configure physical button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initially run motors at MOTOR_SPEED
  mc1.setSpeed(1, MOTOR_SPEED);
  mc1.setSpeed(2, MOTOR_SPEED);
  Serial.println("ControlLogic: motor started");

  // Connect to Wi-Fi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("ControlLogic: connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nControlLogic: Wi-Fi connected");
  Serial.print("ControlLogic: IP Address: ");
  Serial.println(WiFi.localIP());

  // Start listening on UDP port
  udp.begin(UDP_PORT);
  Serial.print("ControlLogic: UDP listening on port ");
  Serial.println(UDP_PORT);
}

void loopControl() {
  // Handle physical button toggles
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    // If Wi-Fi previously forced a stop, clearing wifiStopped resumes motors
    if (wifiStopped) {
      wifiStopped  = false;
      motorRunning = true;
      Serial.println("ControlLogic: motor resumed after Wi-Fi stop");
    } else {
      motorRunning = !motorRunning;
      Serial.println(motorRunning ? "ControlLogic: Motor ON" : "ControlLogic: Motor OFF");
      if (!motorRunning) {
        mc1.setSpeed(1, 0);
        mc1.setSpeed(2, 0);
      }
    }
    delay(200); // debounce
  }
  lastButtonState = currentButtonState;

  // Handle incoming UDP “Stop” command
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(udpBuffer, BUF_SIZE - 1);
    if (len > 0) {
      udpBuffer[len] = '\0';
      Serial.print("ControlLogic: UDP Received → ");
      Serial.println(udpBuffer);
      if (strcmp(udpBuffer, "Stop") == 0) {
        motorRunning = false;
        wifiStopped  = true;
        mc1.setSpeed(1, 0);
        mc1.setSpeed(2, 0);
        Serial.println("ControlLogic: Motor STOPPED by Wi-Fi");
      }
    }
  }

  // Periodically refresh motor command if still running
  unsigned long now = millis();
  if ((now - lastCommandTime) >= CMD_INTERVAL) {
    lastCommandTime = now;
    if (motorRunning && !wifiStopped) {
      mc1.setSpeed(1, MOTOR_SPEED);
      mc1.setSpeed(2, MOTOR_SPEED);
    }
  }
}
