// LIBs

#include <Arduino.h>

// Constants

// set startType 0 for hardStart and 1 for softStart of motor every turn
const int motorStartType = 1;
const int motorSpeed = 150;
const int motorTime = 5000;
const int currentGain = 20;
const unsigned int repsNumber = 10;
const unsigned int buttonDelay = 500;

const int buttonPin = 12;
const int driverPwmPin = 3;
const int driverDirPin = 4;
const int driverSleepPin = 7;
const int driverCSPin = A0;
const int driverFaultPin = 2;

// Vars

// x = 1 - X

int buttonState = 0;
int motorDirection = 0;
int currentPWM = 0;
int running = 0;
unsigned int currentRep = 0;

unsigned long currentTime = 0;
unsigned long endTime = 0;
unsigned long previousButtonStateChange = 0;
unsigned long runningMotorTime = 0;
unsigned int currentOffset = 0;

void softStart(int speed)
{
  currentPWM = 0;
  while (currentPWM <= speed)
  {
    currentPWM += 10;
    analogWrite(driverPwmPin, currentPWM);
    digitalWrite(13, HIGH);

    delay(2);
  }
  running = 1;
  analogWrite(driverPwmPin, speed);
}

void hardStart(int speed)
{
  analogWrite(driverPwmPin, speed);
  digitalWrite(13, HIGH);
  running = 1;
}

void enableDriver()
{
  digitalWrite(driverSleepPin, HIGH);
  delay(1);
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
  return 1;
}

void checkFault()
{
  if (!digitalRead(driverFaultPin))
  {
    digitalWrite(driverSleepPin, LOW);
    Serial.println(digitalRead(driverFaultPin));
  };
}

void readButton()
{
  if (digitalRead(buttonPin) == LOW)
  {
    if (millis() - previousButtonStateChange >= buttonDelay)
    {
      buttonState = !buttonState;
      previousButtonStateChange = millis();
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(driverFaultPin, INPUT_PULLUP);
  pinMode(driverPwmPin, OUTPUT);
  pinMode(driverDirPin, OUTPUT);
  pinMode(driverSleepPin, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(driverCSPin, INPUT);

  calibrateOffset();
  delay(100);
  

  Serial.println("Setup completed!");
  Serial.print("Current offset calibrated as: ");
  Serial.println(currentOffset);
}

void loop()
{
  readButton();
  if (buttonState == 1 && currentRep < repsNumber)
  {
    currentTime = millis();
    if (currentTime - runningMotorTime >= motorTime)
    {
      runningMotorTime = currentTime;
      motorDirection = !motorDirection;
      running = 0;
      Serial.println("I am here");
    }
    else
    {
      // digitalWrite(driverSleepPin, HIGH);
      digitalWrite(driverDirPin, motorDirection);
      if (running == 0)
      {
        enableDriver();
        softStart(motorSpeed);
      }
    }
  }
  else
  {
    analogWrite(driverPwmPin, LOW);
    Serial.println("I am here NOW");

    // digitalWrite(driverSleepPin, LOW);
    running = 0;
  }

  // Serial.print(buttonState);

  // if(currentRep <= repsNumber)
  // {

  // }

  // checkFault();
  // digitalWrite(driverSleepPin, HIGH);
  // delay(1);

  // currentTime = millis();
  // digitalWrite(driverDirPin, LOW);
  // softStart(200);
  // analogWrite(driverPwmPin, 200);
  // digitalWrite(13, HIGH);
  // delay(1000);
  // Serial.println(analogRead(driverCSPin));
  // Serial.println(digitalRead(driverFaultPin));
  // delay(2000);
  // analogWrite(driverPwmPin, 0);
  // digitalWrite(driverDirPin, HIGH);
  // softStart(200);

  // analogWrite(driverPwmPin, 200);
  // digitalWrite(13, LOW);
  // delay(1000);
  // Serial.println(analogRead(driverCSPin));
  // delay(2000);
  // analogWrite(driverPwmPin, 0);
  // digitalWrite(driverSleepPin, LOW);
}
