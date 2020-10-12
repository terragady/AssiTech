// LIBs

#include <Arduino.h>

// Constants
const int buttonPin = 0;
const int driverPwmPin = 0;
const int driverDirPin = 0;
const int driverSleepPin = 0;
const int driverCSPin = 0;
const int currentGain = 20;

// Vars

// x = 1 - X

int buttonState;

unsigned long currentTime = 0;
unsigned long previousButtonTime = 0;
unsigned long runningMotorTime = 0;

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  currentTime = millis();
}
