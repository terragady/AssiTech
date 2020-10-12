// LIBs

#include <Arduino.h>

// Constants
const int motorForwardSpeed = 255;
const int motorReverseSpeed = 255;



const int buttonPin = 0;
const int driverPwmPin = 0;
const int driverDirPin = 0;
const int driverSleepPin = 0;
const int driverCSPin = 0;
const int currentGain = 20;

// Vars

// x = 1 - X

int buttonState;
int motorDirection;
int currentPWM = 0;

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
  if (currentPWM < motorForwardSpeed)
  {
    softStart();
  }
  
}

void softStart()
{
  currentPWM += 10;
  analogWrite(driverPwmPin, currentPWM);
  delay(10)
}