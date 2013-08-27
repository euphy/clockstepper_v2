#include <AccelStepper.h>
#include <Wire.h>
#include "RTClib.h"
#include <Button.h>

/*
Linear Clock driver
Written by Sandy Noble (sandy.noble@gmail.com)
This version 16th July 2011

Uses DS1307 Real time clock module wired:
Arduino pin A4 to DS1307 SDA pin.
Arduino pin A5 to DS1307 SCL pin.

Uses 2x L293D H bridge drivers wired to arduino pins 4, 5, 6 & 7 driving minute hand motor,
and pins 8, 9, 10 & 11 driving hour hand motor.  This is an extremely simple circuit.

A "home" switch is attached on pin 2 which will fire an interrupt when homing the clock.


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

int const millisPerSecond = 1000;

/* This is the number of steps to move the indicator from one end to the other. */
int const stepsPerClock = 2105;

float const stepsPerMinute = stepsPerClock/60.0;
float const stepsPerHourMinute = stepsPerClock/720.0;
float const stepsPerHour = stepsPerClock/12.0;

/* User interface */

boolean currentlyRunning = false;

Button hourSetButton = Button(16, BUTTON_PULLUP);
Button minuteSetButton = Button(17, BUTTON_PULLUP);

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
  Serial.begin(57600);
  Serial.println("LINEAR CLOCK.");

  // attach limit interrupts
  attachInterrupt(mInt, mLimit, FALLING);
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);

  attachInterrupt(hInt, hLimit, FALLING);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);


  Wire.begin();
  RTC.begin();


  if (! RTC.isrunning()) 
  {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  else
  {
    Serial.println("RTC is running.");
  }

  minuteHand.setMaxSpeed(maxSpeed);
  minuteHand.setAcceleration(acceleration);
  
  hourHand.setMaxSpeed(maxSpeed);
  hourHand.setAcceleration(acceleration);
  
  moveHandsToHome();
  
}

void loop() 
{
  findTimeToDisplay();
  setHandPositions();
  moveHands();
  readButtons();
  debug();
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

void readButtons()
{
  hourSetButton.isPressed();
  if (hourSetButton.stateChanged() && hourSetButton.wasPressed())
  {
    Serial.println("Incrementing Hour hand.");
    manualIncrementHour();
  }

  minuteSetButton.isPressed();
  if (minuteSetButton.stateChanged() && minuteSetButton.wasPressed())
  {
    Serial.println("Incrementing Minute hand.");
    manualIncrementMinute();
  }
}

void manualIncrementHour()
{
  DateTime now = RTC.now();
  int nowHour = now.hour();

  if (nowHour >= 23)
    nowHour = 0;
  else
    nowHour++;
  
  DateTime adjTime = DateTime(now.year(), now.month(), now.day(), nowHour, now.minute(), now.second());
  
  RTC.adjust(adjTime);
}

void manualIncrementMinute()
{
  DateTime now = RTC.now();
  int nowMin = now.minute();

  if (nowMin >= 59)
    nowMin = 0;
  else
    nowMin++;
  
  DateTime adjTime = DateTime(now.year(), now.month(), now.day(), now.hour(), nowMin, now.second());
  
  RTC.adjust(adjTime);
  
}


void moveHands()
{
  if (hourHand.distanceToGo() != 0 || minuteHand.distanceToGo() != 0)
  {
    Serial.print("Moving to ");
    Serial.print(currentHourPos);
    Serial.print(":");
    Serial.print(currentMinutePos);
    Serial.print(" (");
    Serial.print(hourHand.distanceToGo());
    Serial.print(":");
    Serial.print(minuteHand.distanceToGo());
    Serial.println(")");
    
    hourHand.enableOutputs();
    minuteHand.enableOutputs();
    
    hourHand.run();
    minuteHand.run();
  }
  else
  {
    hourHand.disableOutputs();
    minuteHand.disableOutputs();
    Serial.print("After moving: ");
    reportPosition();
  }
}

void moveHandsToHome()
{
  Serial.println("Rewinding hands");
  rewindHands();
  Serial.println("Hands are rewound.");
  reportPosition();
  delay(4000);
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

void rewindHands()
{
  if (!minuteHandIsHome())
  { 
    // run backwards, slowly until sensor is triggered.
    Serial.println("Minute hand not home.");
    minuteHand.setMaxSpeed(200.0);
    minuteHand.moveTo(0-stepsPerClock);
  }
  
  if (!hourHandIsHome())
  {
    Serial.println("Hour hand not at home.");
    hourHand.setMaxSpeed(200.0);
    hourHand.moveTo(0-stepsPerClock);
  }

  while (minuteHand.distanceToGo() != 0 || hourHand.distanceToGo() != 0)
  {
    Serial.println("About to run backwards.");
    minuteHand.run();
    Serial.println("run minutes...");
    hourHand.run();
    Serial.println("run hours...");
    
    if (minuteHandIsHome())
    {
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(0);
      minuteHand.disableOutputs();
    }
    if (hourHandIsHome())
    {
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(0);
      hourHand.disableOutputs();
    }
  }

  minuteHand.setMaxSpeed(maxSpeed);
  hourHand.setMaxSpeed(maxSpeed);
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

