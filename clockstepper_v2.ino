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

*/

/* Clock setup */
RTC_DS1307 RTC;


// These set up the motors.  The values here depend on how they've been wired up.
AccelStepper minuteHand(4, 7,6,5,4); // minutes
AccelStepper hourHand(4, 8,9,10,11); // hours

float maxSpeed = 1000.0;
float acceleration = 800.0;

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

int const END_MARGIN = 8;

/* User interface */

long lastDebugMessageTime = 0L;

/* Limits and interrupts */
volatile boolean mLimitTriggered = false;
volatile boolean hLimitTriggered = false;

int mInt = 0;
int hInt = 1;

const byte BACKWARD = 0;
const byte FORWARD = 1;

static byte mDir = BACKWARD;
static byte hDir = BACKWARD;

static boolean minuteWinding = true;
static boolean hourWinding = true;


void mLimit()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50)
    mLimitTriggered = true;
   lastInterrupt = interruptTime;
}

void hLimit()
{
  static unsigned long lastInterrupt = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterrupt > 50)
    hLimitTriggered = true;
   lastInterrupt = interruptTime;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("LINEAR CLOCK.");

  // attach limit interrupts
  attachInterrupt(mInt, mLimit, FALLING);
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);

  attachInterrupt(hInt, hLimit, FALLING);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);

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
  hourHand.setMaxSpeed(maxSpeed);
  hourHand.setAcceleration(acceleration);
  
  recalculateStepsPerUnits();
  homeHands();
  Serial.println("HOMED!!");
}

void recalculateStepsPerUnits() {
  stepsPerMinute = stepsPerClockMinute/60.0;
  stepsPerHourMinute = stepsPerClockHour/720.0;
  stepsPerHour = stepsPerClockHour/12.0;
}
  

void loop() 
{
  findTimeToDisplay();
  setHandPositions();
  moveHands();
  dealWithLimits();
  debug();
}

void dealWithLimits()
{
  if (mLimitTriggered) {
    if (mDir == BACKWARD) {
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(0);
      minuteHand.disableOutputs();
    }
    else if (mDir == FORWARD) {
      // recalibrate length of machine
      stepsPerClockMinute = minuteHand.currentPosition()-END_MARGIN;
      recalculateStepsPerUnits();
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteHand.disableOutputs();
    }
    mLimitTriggered = false;
  }

  if (hLimitTriggered) {
    if (hDir == BACKWARD) {
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      hourHand.disableOutputs();
    }
    else if (hDir == FORWARD) {
      // recalibrate length of machine
      stepsPerClockHour = hourHand.currentPosition()-END_MARGIN;
      recalculateStepsPerUnits();
      hourHand.moveTo(hourHand.currentPosition());
      hourHand.disableOutputs();
    }
    hLimitTriggered = false;
  }
}

void debug()
{
  long now = millis();
  if ((now - lastDebugMessageTime) > 2000)
  {
    reportPosition();
    lastDebugMessageTime = now;
  }
}

void homeHands()
{
  // run backwards, slowly until sensor is triggered.
  Serial.println("Minute hand not home.");
  mDir = BACKWARD;
  minuteWinding = true;
  
  Serial.println("Hour hand not at home.");
  hDir = BACKWARD;
  hourWinding = true;

  while (minuteWinding || hourWinding) {
    if (mLimitTriggered) {
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(0);
      minuteWinding = false;
      mLimitTriggered = false;
    }
    if (minuteWinding) {
      minuteHand.move(-2);
      minuteHand.setSpeed(-200);
      minuteHand.runSpeed();
//      Serial.print("rewind minutes pos ");
//      Serial.println(minuteHand.currentPosition());
    }
    
    if (hLimitTriggered) {
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      hourWinding = false;
      hLimitTriggered = false;
    }
    if (hourWinding) {
      hourHand.move(-2);
      hourHand.setSpeed(-200);
      hourHand.runSpeed();
//      Serial.print("rewind hours pos ");
//      Serial.println(hourHand.currentPosition());
    }
  }


  // so now test the full length
  minuteWinding = true;
  hourWinding = true;
  
  while (minuteWinding || hourWinding) {
    if (mLimitTriggered) {
      stepsPerClockMinute = minuteHand.currentPosition()-END_MARGIN;
      recalculateStepsPerUnits();
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteWinding = false;
      mLimitTriggered = false;
    }
    if (minuteWinding) {
      minuteHand.move(2);
      minuteHand.setSpeed(200);
      minuteHand.runSpeed();
//      Serial.println("forward wind minutes...");
    }
    if (hLimitTriggered && hourWinding) {
      stepsPerClockHour = hourHand.currentPosition()-END_MARGIN;
      recalculateStepsPerUnits();
      hourHand.moveTo(hourHand.currentPosition());
      hourWinding = false;
      hLimitTriggered = false;
    }
    if (hourWinding) {
      hourHand.move(2);
      hourHand.setSpeed(200);
      hourHand.runSpeed();
//      Serial.println("forward wind hours...");
    }      
  }


  minuteHand.setMaxSpeed(maxSpeed);
  mDir = FORWARD;
  hourHand.setMaxSpeed(maxSpeed);
  hDir = FORWARD;
}

void moveHands()
{
  if (hourHand.distanceToGo() != 0 || minuteHand.distanceToGo() != 0)
  {
//    Serial.print("Moving to ");
//    Serial.print(currentHourPos);
//    Serial.print(":");
//    Serial.print(currentMinutePos);
//    Serial.print(" (");
//    Serial.print(hourHand.distanceToGo());
//    Serial.print(":");
//    Serial.print(minuteHand.distanceToGo());
//    Serial.println(")");
    
    hourHand.enableOutputs();
    minuteHand.enableOutputs();

    if (hLimitTriggered)
      Serial.println("hlimit triggered.");
    else {
//      Serial.println("running hour!");
      hourHand.run();
    }
      
    if (mLimitTriggered)
      Serial.println("mlimit triggered.");
    else {
//      Serial.println("running hour!");
      minuteHand.run();
    }
  }
  else
  {
    hourHand.disableOutputs();
    minuteHand.disableOutputs();
//    Serial.print("After moving: ");
//    reportPosition();
  }
  
}


String reportPosition()
{
  Serial.print("Position: ");
  Serial.print(hourHand.currentPosition());
  Serial.print(":");
  Serial.print(minuteHand.currentPosition());
  Serial.print(" (");
  Serial.print(currentHours);
  Serial.print(":");
  Serial.print(currentMinutes);
  Serial.print(":");
  Serial.print(currentSeconds);
  Serial.println(")");
}  

void findTimeToDisplay()
{
  DateTime now = RTC.now();
   
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  currentHours = now.hour();
  if (currentHours >= 12)
    currentHours = currentHours - 12;
  currentMinutes = now.minute();
  currentSeconds = now.second();
}

void setHandPositions()
{
  displayHour(currentHours, currentMinutes);
  displayMinute(currentMinutes);
}

void displayHour(int hour, int minute)
{
  // work out the new position and set it globally eg time 4:25.
  // first hours
  // eg 0.2055 * 4 * 60 = 49.333
  float justHours = stepsPerHourMinute * hour * 60;
  // eg 0.2055 * 25 = 5.13888
  float justMinutes = stepsPerHourMinute * minute;
  
  // stick em together: position is 54.472 (4:25)
  currentHourPos = justHours + justMinutes;

  // round to integer
  // eg 54
  int newPos = currentHourPos;
  hourHand.moveTo(newPos);
}

void displayMinute(int minute)
{
  // work out the new position and set it globally
  // eg 2.467 * 25 = 61.675
  currentMinutePos = stepsPerMinute * minute;

  // round to integer
  // eg 61
  int newPos = currentMinutePos;
  minuteHand.moveTo(newPos);

}

