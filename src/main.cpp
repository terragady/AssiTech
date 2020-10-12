// LIBs

#include <Arduino.h>

// Constants
const int motorForwardSpeed = 155;
const int motorReverseSpeed = 155;
const int motorForwardTime = 10;
const int motorReverseTime = 10;
const int repsNumber = 10;
const int currentGain = 20;

const int buttonPin = 0;
const int driverPwmPin = 0;
const int driverDirPin = 0;
const int driverSleepPin = 0;
const int driverCSPin = 0;
const int driverFaultPin = 0;

// Vars

// x = 1 - X

int buttonState = 0;
int motorDirection;
int currentPWM = 0;

unsigned long currentTime = 0;
unsigned long previousButtonTime = 0;
unsigned long runningMotorTime = 0;
unsigned int currentOffset = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(driverFaultPin, INPUT_PULLUP);
  pinMode(driverPwmPin, OUTPUT);
  pinMode(driverDirPin, OUTPUT);
  pinMode(driverSleepPin, OUTPUT);
  pinMode(driverCSPin, INPUT);

  calibrateOffset();
  delay(100);

  Serial.println('Setup completed');
  Serial.print('Offset calibrated as: ');
  Serial.println(currentOffset);
}

void loop()
{
  currentTime = millis();
  if (currentPWM < motorForwardSpeed)
  {
    softStart(motorForwardSpeed);
  }
}

void softStart(int speed)
{
  while (currentPWM < speed)
  {
    currentPWM += 10;
    analogWrite(driverPwmPin, currentPWM);
    delay(10);
  }
}

void enableDriver()
{
  digitalWrite(driverSleepPin, HIGH);
}

void calibrateOffset()
{
  analogWrite(driverPwmPin, 0);
  enableDriver();
  currentOffset = analogRead(driverCSPin);
}

unsigned int getCurrentReading()
{
  int reading = analogRead(driverCSPin) - currentOffset;
  if (reading > 0)
  {
    return reading * 5000000 / 1024 / currentGain;
  }
  return 0;
}