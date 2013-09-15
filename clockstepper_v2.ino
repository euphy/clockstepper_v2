#include <AccelStepper.h>
#include <Wire.h>
#include "RTClib.h"
#include <Button.h>

/*
Linear Clock driver
Written by Sandy Noble (sandy.noble@gmail.com)
This version 27th August 2013

Uses DS1307 Real time clock module wired:
Arduino pin A4 to DS1307 SDA pin.
Arduino pin A5 to DS1307 SCL pin.

Uses 2x L293D H bridge drivers wired to arduino pins 4, 5, 6 & 7 driving minute hand motor,
and pins 8, 9, 10 & 11 driving hour hand motor.  This is an extremely simple circuit.

Home switches are on pins D2 and D3, they are the AVR interrupts 0 and 1.  They are
electrically wired to switches at both ends of each rail.  


This should be fairly simple self-explanatory code.

@TODO  Reserve pins for LDR and lighting controller (PWM).
@TODO  Write clock size into EEPROM when it changes so it doesn't have to recalibrate every time it resets.
@TODO  Reserve pins for physical buttons.
@TODO  

*/

/* Clock setup */
RTC_DS1307 RTC;


// These set up the motors.  The values here depend on how they've been wired up.
const byte motoraEnablePin = 3;
const byte motoraStepPin = 4;
const byte motoraDirPin = 5;

const byte motorbEnablePin = 6;
const byte motorbStepPin = 7;
const byte motorbDirPin = 8;

AccelStepper minuteHand(1,motoraStepPin, motoraDirPin); // minutes
AccelStepper hourHand(1,motorbStepPin, motorbDirPin); // hours

float maxSpeed = 2000.0;
float acceleration = 1000.0;
byte stepSize = 8;

boolean motorsEnabled = false;

// Minutes setup
int startMinutePos = 0;
int currentMinutePos = startMinutePos;

// Hours setup
int startHourPos = 0;
int currentHourPos = startHourPos;

// These are the actual time, in seconds minutes and hours.  
// findTimeToDisplay() writes these values, and renderTime() reads them.
int currentSeconds = 0;
int currentMinutes = 0;
int currentHours = 0;

/* This is the number of steps to move the indicator from one end to the other. */
static int stepsPerClockMinute = 2105;
static int stepsPerClockHour = 2105;

float stepsPerMinute = stepsPerClockMinute/60.0;
float stepsPerHourMinute = stepsPerClockHour/720.0;
float stepsPerHour = stepsPerClockHour/12.0;

int const END_MARGIN = 4;

/* User interface */

long lastDebugMessageTime = 0L;

/* Limits and interrupts */
volatile boolean mLimitTriggered = false;

int minuteLimitPin = 19;
int mInt = 4;

const byte BACKWARD = 0;
const byte FORWARD = 1;

static byte mDir = BACKWARD;

static boolean minuteWinding = true;

boolean debugToSerial = false;


void mLimit()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50)
    mLimitTriggered = true;
   lastInterrupt = interruptTime;
}


void setup() 
{
  Serial.begin(9600);
  Serial.println("LINEAR CLOCK.");

  // attach limit interrupts
  attachInterrupt(mInt, mLimit, FALLING);
  pinMode(minuteLimitPin, INPUT);
  digitalWrite(minuteLimitPin, HIGH);

  pinMode(motoraEnablePin, OUTPUT);
  digitalWrite(motoraEnablePin, HIGH);
  minuteHand.setEnablePin(motoraEnablePin);
  minuteHand.setPinsInverted(false, false, true);

  minuteHand.setMaxSpeed(maxSpeed);
  hourHand.setMaxSpeed(maxSpeed);
  
  // setup RTC stuff
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  } else {
    Serial.println("RTC is running.");
  }

  minuteHand.setMaxSpeed(maxSpeed);
  minuteHand.setAcceleration(acceleration);
  
  recalculateStepsPerUnits();
  homeHands();
}

void recalculateStepsPerUnits() {
  stepsPerMinute = stepsPerClockMinute/60.0;
  stepsPerHourMinute = stepsPerClockHour/720.0;
  stepsPerHour = stepsPerClockHour/12.0;
}



void loop() 
{
}

void homeHands()
{
  // run backwards, slowly until sensor is triggered.
  Serial.println("Minute hand not home.");
  mDir = BACKWARD;
  minuteWinding = true;
  minuteHand.enableOutputs();
  
  Serial.println("Winding backwards.");

  while (minuteWinding) {
    if (mLimitTriggered) {
      minuteWinding = false;
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      // creep until trigger cleared
      minuteHand.move(100*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        Serial.print("minute creeping... ");
        Serial.print(minuteHand.distanceToGo());
        Serial.print(", speed: ");
        Serial.println(minuteHand.speed());
        minuteHand.run();
        if (digitalRead(minuteLimitPin) != LOW) {
          Serial.println("Limit switch released (a).");
          minuteHand.moveTo(minuteHand.currentPosition());
          minuteHand.setSpeed(0);
          break;
        }
      }
      minuteHand.move(END_MARGIN*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        Serial.print("Adding margin ");
        Serial.print(minuteHand.distanceToGo());
        Serial.print(", speedd: ");
        Serial.println(minuteHand.speed());
        minuteHand.run();
      }
      
      mLimitTriggered = false;
      minuteHand.disableOutputs();
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteHand.setSpeed(0);
      Serial.println("Minute rewind finished.");
    }
    if (minuteWinding) {
      minuteHand.move(-stepSize);
      minuteHand.setSpeed(-maxSpeed);
      minuteHand.runSpeed();
      if (debugToSerial) {
        Serial.print("rewind minutes pos ");
        Serial.println(minuteHand.currentPosition());
      }
    }
  }


  // so now test the full length
  Serial.println("Winding forward to detect length of clock...");
  detectLengthOfClock();

  Serial.println("HOMED!!");
  Serial.print("Minute axis is ");
  Serial.print(stepsPerClockMinute);
  Serial.println(" true steps long.");
}

void detectLengthOfClock()
{
  Serial.print("Minute current position: ");
  Serial.print(minuteHand.currentPosition());
  Serial.print(", distance to go: ");
  Serial.print(minuteHand.distanceToGo());
  Serial.print(", speed: ");
  Serial.print(minuteHand.speed());
  Serial.print(", target:" );
  Serial.println(minuteHand.targetPosition());


  minuteWinding = true;
  minuteHand.enableOutputs();
  
  minuteHand.setMaxSpeed(maxSpeed);
 
  while (minuteWinding) {
    if (mLimitTriggered && minuteWinding) {
      Serial.println("Minute limit triggered.");
      minuteWinding = false;
      Serial.print("Current position at trigger ");
      Serial.println(minuteHand.currentPosition());
      minuteHand.moveTo(minuteHand.currentPosition());

      // creep until trigger cleared
      minuteHand.move(-(100*stepSize));
      // not sure why this speed needs to be manually reversed, but it does.
      while (minuteHand.distanceToGo() != 0) {
        Serial.print("min reverse creeping... ");
        Serial.print(minuteHand.distanceToGo());
        Serial.print(", speedd: ");
        Serial.println(minuteHand.speed());
        
        minuteHand.run();
        if (digitalRead(minuteLimitPin) != LOW) {
          Serial.println("Limit switch released (c).");
          break;
        }
      }

      minuteHand.move(-(END_MARGIN*stepSize*10));
      while (minuteHand.distanceToGo() != 0) {
        Serial.print("Adding minute margin ");
        Serial.print(minuteHand.distanceToGo());
        Serial.print(", speedd: ");
        Serial.println(minuteHand.speed());
        minuteHand.run();
      }
      stepsPerClockMinute = minuteHand.currentPosition();
      recalculateStepsPerUnits();
      mLimitTriggered = false;
      Serial.println("Minute size detection finished.");
    }

    if (minuteWinding) {
      minuteHand.move(stepSize);
      minuteHand.setSpeed(maxSpeed);
      minuteHand.runSpeed();
//      Serial.println("forward wind minutes...");
    }
  }


  minuteHand.setMaxSpeed(maxSpeed);
  mDir = FORWARD;

  minuteHand.disableOutputs();
}

