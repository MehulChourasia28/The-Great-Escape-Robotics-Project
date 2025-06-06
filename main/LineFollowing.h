// LineFollowing.h
#ifndef LINEFOLLOWING_H
#define LINEFOLLOWING_H

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <math.h>

extern const uint8_t backPins[9];
extern const uint8_t frontPins[9];
extern const uint8_t LED_PIN;
extern const uint16_t TIMEOUT_US;
extern const uint16_t CALIB_SAMPLES;
extern uint16_t minBack[9], maxBack[9];
extern uint16_t minFront[9], maxFront[9];

typedef enum { MODE_LINE, MODE_CORNER } Mode;
extern const float Kp, Kd, Ka;
extern const float sensorPitch, rowSpacing;
extern const int16_t turnSpeed;
extern const uint16_t turn90Time;

extern MotoronI2C mc1;
extern MotoronI2C mc2;

uint16_t readRC(uint8_t pin);
void readRow(const uint8_t pins[], uint16_t raw[], uint16_t mn[], uint16_t mx[]);
void calibrate(const uint8_t pins[], uint16_t mn[], uint16_t mx[]);
void pivot(bool turnRight);

void setupLineFollowing();
void loopLineFollowing();

#endif // LINEFOLLOWING_H
