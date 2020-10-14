
// LIBs

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants

/////////////////////////// USER CONFIG ///////////////////////////
// set startType 0 for hardStart and 1 for softStart of motor every turn
const int motorStartType = 1;
// set motor speed from 0 to 255
const int motorSpeed = 255;
// set motor time it will be running forward in ms
const unsigned int motorForwardTime = 1000;
// set motor time it will be running reverse in ms
const unsigned int motorReverseTime = 1000;
// set number of cycles (back and forth)
const unsigned int repsNumber = 5;
// set 1 if you want the cycles to be reseted after button press during working phase
const int repReset = 1;
// set 0 if first movement should be forward or 1 for reverse
int motorDirection = 0;

/////////////////////////////// END ///////////////////////////////

const int currentGain = 20;
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
int currentPWM = 0;
int running = 0;
unsigned int currentRep = 0;

unsigned long currentTime = 0;
unsigned long endTime = 0;
unsigned long previousButtonStateChange = 0;
unsigned long runningMotorTime = 8.64e+7;
unsigned long previousCurrentReadingTime = 0;
unsigned int currentOffset = 0;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// functions

void softStart(int speed)
{
  currentPWM = 0;
  while (currentPWM <= speed)
  {
    currentPWM += 10;
    analogWrite(driverPwmPin, currentPWM);
    delay(2);
  }
  running = 1;
  Serial.println("SOFT");
  analogWrite(driverPwmPin, speed);
}

void hardStart(int speed)
{
  analogWrite(driverPwmPin, speed);
  running = 1;
  Serial.println("SOFT");
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
  int rawReading = analogRead(driverCSPin);
  int reading = rawReading - currentOffset;
  if (reading > 0)
  {
    return reading * 5000000 / 1024 / currentGain;
  }
  return 0;
}

void checkFault()
{
  if (!digitalRead(driverFaultPin))
  {
    Serial.println("There was a fault with the driver");
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
// CUSTOM CHARS

byte FW[] = {
    0x04,
    0x0E,
    0x1F,
    0x04,
    0x04,
    0x04,
    0x1F,
    0x1F};

byte RW[] = {
    0x04,
    0x04,
    0x04,
    0x1F,
    0x0E,
    0x04,
    0x1F,
    0x1F};

byte time[] = {
    0x1F,
    0x1F,
    0x0E,
    0x04,
    0x04,
    0x0E,
    0x11,
    0x1F};
byte cycles[] = {
    0x04,
    0x0E,
    0x1F,
    0x04,
    0x04,
    0x1F,
    0x0E,
    0x04};
// SETUP

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

  lcd.init(); // initialize the lcd
  lcd.createChar(0, FW);
  lcd.createChar(1, RW);
  lcd.createChar(2, time);
  lcd.createChar(3, cycles);

  lcd.backlight();
  lcd.home();
  lcd.print("   Ready to start");
  lcd.setCursor(0, 1);
  lcd.write(0);
  lcd.print(" ");
  lcd.print(motorForwardTime);
  lcd.print("ms ");
  lcd.write(1);
  lcd.print(" ");
  lcd.print(motorReverseTime);
  lcd.print("ms");
  lcd.setCursor(0, 2);
  lcd.write(3);
  lcd.print(" ");
  lcd.print(repsNumber);
  lcd.print("x");
  lcd.setCursor(0, 3);
  lcd.write(2);
  lcd.print(" ");
  lcd.print((motorForwardTime + motorReverseTime) / 1000 * repsNumber / 60 + 1);
  lcd.print("min");
}

// LOOP

void loop()
{
  readButton();
  currentTime = millis();
  if (currentTime - previousCurrentReadingTime >= 1000)
  {
    previousCurrentReadingTime = currentTime;

    // Serial.println(getCurrentReading());
  }
  if (buttonState == 1 && currentRep/2 < repsNumber)
  {
    if (currentTime - runningMotorTime >= (motorDirection ? motorForwardTime : motorReverseTime))
    {
      runningMotorTime = currentTime;
      motorDirection = !motorDirection;
      running = 0;
    }
    else
    {
      digitalWrite(driverDirPin, motorDirection);
      if (running == 0)
      {
        lcd.clear();
        enableDriver();
        currentRep += 1;
        lcd.home();
        lcd.print("Current cycle:");
        lcd.print(currentRep/2);
        motorDirection ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        motorStartType ? softStart(motorSpeed) : hardStart(motorSpeed);
      }
    }
  }
  else
  {
    analogWrite(driverPwmPin, LOW);
    running = 0;
    repReset ? currentRep = 0 : currentRep = currentRep;
  }
}
