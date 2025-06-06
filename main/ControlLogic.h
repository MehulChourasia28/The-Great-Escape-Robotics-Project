#ifndef CONTROLLOGIC_H
#define CONTROLLOGIC_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Motoron.h>
#include "LineFollowing.h"

// Wi-Fi credentials (adjust as needed)
static const char* SSID     = "eric";
static const char* PASSWORD = "password";

// UDP settings
static const int UDP_PORT    = 55500;
static const size_t BUF_SIZE = 64;

// Physical kill-switch pin
static const int BUTTON_PIN = 2;

// Desired motor speed when running
static const int MOTOR_SPEED = 300;

// How often (ms) to refresh motor speed command
static const unsigned long CMD_INTERVAL = 100;

// Public functions – call these from your sketch’s setup() and loop()
void setupControl();
void loopControl();

#endif // CONTROLLOGIC_H
