///////////////////////////////////////////////////////////////////////////////////////////
//                        _   ___ ___ ___ _____ ___ ___ _  _                             //
//                       /_\ / __/ __|_ _|_   _| __/ __| || |                            //
//                      / _ \\__ \__ \| |  | | | _| (__| __ |                            //
//                     /_/ \_\___/___/___| |_| |___\___|_||_|                            //
//                                                                                       //
/////////////////////////////////////// USER CONFIG ///////////////////////////////////////
// set startType 0 for hardStart and 1 for softStart of motor every turn                 //
const int motorStartType = 1;                                                            //
// set motor speed from 0 to 255                                                         //
const int motorSpeed = 255;                                                              //
// set motor time it will be running forward in ms                                       //
const unsigned int motorForwardTime = 2500;                                              //
// set motor time it will be running reverse in ms                                       //
const unsigned int motorReverseTime = 2500;                                              //
// set number of cycles (back and forth)                                                 //
const unsigned int repsNumber = 3;                                                       //
// set 1 if you want the cycles to be reseted after button press during working phase    //
const int repReset = 0;                                                                  //
// set 0 if first movement should be forward or 1 for reverse                            //
int motorDirection = 1;                                                                  //
// refresh time for current measurements during move in ms                               //
unsigned int currentRefreshTime = 250;                                                   //
// Delay for steps in motor soft start, higher the value, lower the current draw         //
// on every start but higher the start time (for speed 255 and delay 10 it is 250ms)     //
unsigned int softStartDelayCoef = 5;                                                     //
// Uncomment if needed                                                                   //
 #define CHECKFAULT                                                                    //
 #define STOPATFAULT                                                                   //
/////////////////////////////////////////// END ///////////////////////////////////////////


// #define CHECKFAULT

// LIBs

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// CONSTANTS

const int currentGain = 40;
const unsigned int buttonDelay = 500;
const int buttonPin = 12;
const int driverPwmPin = 3;
const int driverDirPin = 4;
const int driverSleepPin = 7;
const int driverCSPin = A0;
const int driverFaultPin = 2;

// VARS

int buttonState = 0;
int currentPWM = 0;
int running = 0;
unsigned int currentRep = 1;

unsigned long currentTime = 0;
unsigned long endTime = 0;
unsigned long previousButtonStateChange = 0;
unsigned long runningMotorTime = 8.64e+7;
unsigned long previousCurrentReadingTime = 0;
unsigned int currentOffset = 0;

// FUNCS

LiquidCrystal_I2C lcd(0x27, 20, 4);

void softStart(int speed)
{
  currentPWM = 0;
  while (currentPWM + 10 <= speed)
  {
    currentPWM += 10;
    analogWrite(driverPwmPin, currentPWM);
    delay(softStartDelayCoef);
  }
  running = 1;
  analogWrite(driverPwmPin, speed);
}

void hardStart(int speed)
{
  analogWrite(driverPwmPin, speed);
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
  return 0;
}

void checkFault()
{
  if (!digitalRead(driverFaultPin))
  {
    #if defined STOPATFAULT
    digitalWrite(driverSleepPin, LOW);
    while(1)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" FAULT FROM DRIVER  ");
      lcd.setCursor(0,1);
      lcd.print("   PLEASE RESTART   ");
      lcd.setCursor(0,2);
      lcd.print(" SYSTEM TO CONTINUE ");
      delay(10000);
    }
    #endif
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

  #if defined (CHECKFAULT)
    checkFault();
  #endif

  readButton();
  currentTime = millis();
  if (currentTime - previousCurrentReadingTime >= currentRefreshTime)
  {
    previousCurrentReadingTime = currentTime;
    if (running == 1)
    {
      lcd.setCursor(0, 3);
      lcd.print("Current: ");
      lcd.print(getCurrentReading() / 1000.0, 2);
      lcd.print("A");
    }
  }

  if (buttonState == 1 && currentRep / 2 <= repsNumber)
  {
    // Serial.println(currentTime - runningMotorTime);
    if (currentTime - runningMotorTime >= (motorDirection ? motorForwardTime : motorReverseTime))
    {
      analogWrite(driverPwmPin, 0);
      runningMotorTime = currentTime;
      motorDirection = !motorDirection;
      running = 0;
      currentRep += 1;
    }
    else
    {
      if (running == 0)
      {
        digitalWrite(driverDirPin, motorDirection);
        motorDirection ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        motorStartType ? softStart(motorSpeed) : hardStart(motorSpeed);
        runningMotorTime = currentTime;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Curr. cycle: ");
        lcd.print(currentRep / 2);
        lcd.setCursor(19, 0);
        lcd.write(motorDirection ? 0 : 1);
        lcd.setCursor(0, 1);
        lcd.print("Time left: ");
        lcd.print((motorForwardTime + motorReverseTime) / 1000 * (repsNumber - currentRep / 2) / 60 + 1);
        lcd.print("min");
      }
    }
  }
  else
  {
    analogWrite(driverPwmPin, LOW);
    running = 0;
    repReset ? currentRep = 0 : currentRep = currentRep;
    if (currentRep / 2 > repsNumber)
    {
      lcd.clear();
      lcd.home();
      lcd.print("      FINISHED      ");
      lcd.setCursor(0, 2);
      lcd.print("    press button    ");
      lcd.setCursor(0, 3);
      lcd.print("    to run again    ");
      buttonState = 0;
      currentRep = 1;
      runningMotorTime = 8.64e+7;
      motorDirection = !motorDirection;
    }
  }
}

//********************************************************************************************//
//                                                                                            //
//   Code prepared with happiness by:                                                         //
//                                                                                            //
//    &@@@@@@@@@@@@/         ,(%%%%%%%%%%%%%%%#.                                              //
//    .#@@@@@@@@@@@@#,          /#%%%%%%%%%%#*                                                //
//      *@@@@@@@@@@@@&/          .(%%%%%%%%/.                                                 // 
//       .%@@@@@@@@@@@@#.          *#%%%%(.                                                   //
//         *@@@@@@@@@@@@&*          *#%#*          ___ ___                                    //
//          .%@@@@@@@@@@@@#          **.           `MM `MM                                    //
//            /&@@@@@@@@@@@&,                       MM  MM                                    //
//             ,%@@@@@@@@@@@@(              ___     MM  MM   _____  ____    _    ___          //
//               (&@@@@@@@@@@@&.          6MMMMb    MM  MM  6MMMMMb `MM(   ,M.   )M'          // 
//                *%@@@@@@@@@@@@/.       8M'  `Mb   MM  MM 6M'   `Mb `Mb   dMb   d'           // 
//                 .(@@@@@@@@@@&,            ,oMM   MM  MM MM     MM  YM. ,PYM. ,P            //
//                   *%@@@@@@@/          ,6MM9'MM   MM  MM MM     MM  `Mb d'`Mb d'            //
//                    .(@@@@%.           MM'   MM   MM  MM MM     MM   YM,P  YM,P             //
//                      *%&*             MM.  ,MM   MM  MM YM.   ,M9   `MM'  `MM'             //
//     .*&@@@&(.                         `YMMM9'Yb._MM__MM_ YMMMMM9     YP    YP              //
//    *@@@@@@@@@(.                                                                            //
//    #@@@@@@@@@@&#,.                                                                         //
//    ,@@@@@@@@@@@@&/.          ___________________________________________________           //
//      ,#@@@@@&(,             | www.yallow.no | +47 41 35 06 08 | hello@yallow.no |          //
//                              ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯           // 
//                                                                                            //
//********************************************************************************************//
