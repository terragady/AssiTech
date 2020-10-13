// LIBs

#include <Arduino.h>

// Constants
const int motorForwardSpeed = 155;
const int motorReverseSpeed = 155;
const int motorForwardTime = 10;
const int motorReverseTime = 10;
const int repsNumber = 10;
const int currentGain = 20;

const int buttonPin = 12;
const int driverPwmPin = 3;
const int driverDirPin = 4;
const int driverSleepPin = 7;
const int driverCSPin = A0;
const int driverFaultPin = 2;

// Vars

// x = 1 - X

int buttonState = 0;
int motorDirection;
int currentPWM = 0;
unsigned int currentRep = 0;

unsigned long currentTime = 0;
unsigned long previousButtonTime = 0;
unsigned long runningMotorTime = 0;
unsigned int currentOffset = 0;

void softStart(int speed)
{
  currentPWM = 0;
  while (currentPWM < speed)
  {
    currentPWM += 10;
    analogWrite(driverPwmPin, currentPWM);
    delay(5);
  }
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
    Serial.println("Driver is sending a fault!!!");
  };
}

void readButton() {

      // this only reads the button state after the button interval has elapsed
      //  this avoids multiple flashes if the button bounces
      // every time the button is pressed it changes buttonLed_State causing the Led to go on or off
      // Notice that there is no need to synchronize this use of millis() with the flashing Leds
  
  if (millis() - previousButtonTime >= 2000) {

    if (digitalRead(buttonPin) == LOW) {
      buttonState = ! buttonState; 
      previousButtonTime += 2000;
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
Serial.print(buttonState);

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
