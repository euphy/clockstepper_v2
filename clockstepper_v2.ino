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

AccelStepper hourHand(1,motoraStepPin, motoraDirPin); // minutes
AccelStepper minuteHand(1,motorbStepPin, motorbDirPin); // hours

float maxSpeed = 1000.0;
float acceleration = 4000.0;
byte stepSize = 8;

boolean motorsEnabled = false;

// Minutes setup
long startMinutePos = 0L;
long currentMinutePos = startMinutePos;

// Hours setup
long startHourPos = 0L;
long currentHourPos = startHourPos;

// These are the actual time, in seconds minutes and hours.  
// findTimeToDisplay() writes these values, and renderTime() reads them.
long currentSeconds = 0L;
long currentMinutes = 0L;
long currentHours = 0L;

/* This is the number of steps to move the indicator from one end to the other. */
static long stepsPerClockMinute = 2105L;
static long stepsPerClockHour = 2105L;

float stepsPerMinute = stepsPerClockMinute/60.0;
float stepsPerHourMinute = stepsPerClockHour/720.0;
float stepsPerHour = stepsPerClockHour/12.0;

int const END_MARGIN = 10;

/* User interface */

long lastDebugMessageTime = 0L;
long timeChecked = millis();

/* Limits and interrupts */
volatile boolean mLimitTriggered = false;
volatile boolean hLimitTriggered = false;
byte mLimitPulled = HIGH;
byte hLimitPulled = LOW;
byte hIntTrigger = RISING;
byte mIntTrigger = FALLING;

int minuteLimitPin = 19;
int hourLimitPin = 18;
int mInt = 4;
int hInt = 5;

const byte BACKWARD = 0;
const byte FORWARD = 1;

static byte mDir = BACKWARD;
static byte hDir = BACKWARD;

static boolean minuteWinding = true;
static boolean hourWinding = true;

boolean debugToSerial = false;
boolean debugTriggers = false;


void mLimit()
{
  static unsigned long mlastInterrupt = 0;
  unsigned long minterruptTime = millis();
  if (minterruptTime - mlastInterrupt > 50) {
    mLimitTriggered = true;
  }
  mlastInterrupt = minterruptTime;
}

void hLimit()
{
  static unsigned long hlastInterrupt = 0;
  unsigned long hinterruptTime = millis();
  if (hinterruptTime - hlastInterrupt > 50) {
    hLimitTriggered = true;
  }
  hlastInterrupt = hinterruptTime;
}

void setup() 
{
  Serial.begin(57600);
  Serial.println("LINEAR CLOCK.");

  // attach limit interrupts
  attachInterrupt(mInt, mLimit, mIntTrigger);
  pinMode(minuteLimitPin, INPUT);
  digitalWrite(minuteLimitPin, mLimitPulled);

  attachInterrupt(hInt, hLimit, hIntTrigger);
  pinMode(hourLimitPin, INPUT);
  digitalWrite(hourLimitPin, hLimitPulled);

  pinMode(motoraEnablePin, OUTPUT);
  digitalWrite(motoraEnablePin, HIGH);
  pinMode(motorbEnablePin, OUTPUT);
  digitalWrite(motorbEnablePin, HIGH);
  minuteHand.setEnablePin(motoraEnablePin);
  minuteHand.setPinsInverted(false, false, true);
  hourHand.setEnablePin(motorbEnablePin);
  hourHand.setPinsInverted(true, false, true); // this one turns the opposite direction to A, hence inverted.

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
  hourHand.setMaxSpeed(maxSpeed);
  hourHand.setAcceleration(acceleration);
  
  recalculateStepsPerUnits();
  homeHands();
  hourHand.enableOutputs();
  minuteHand.enableOutputs();
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
//  dealWithLimits();
//  debug();
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
      stepsPerClockMinute = stepsPerClockMinute / stepSize;
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
      stepsPerClockHour = stepsPerClockHour / stepSize;
      recalculateStepsPerUnits();
      hourHand.moveTo(hourHand.currentPosition());
      hourHand.disableOutputs();
    }
    hLimitTriggered = false;
  }
}

void debug()
{
  if ((millis() - lastDebugMessageTime) > 2000)
  {
    reportPosition();
    lastDebugMessageTime = millis();
  }
}

void clearEndStops(int &limitPin, AccelStepper &hand, int closedState)
{
  int distanceToBackOff = -(10*stepSize);
  hand.enableOutputs();
  Serial.println(digitalRead(limitPin));
  if (digitalRead(limitPin) == closedState) {
    Serial.println("limit pin is closed.");
    for (int range=0; range<(200); range++) {
      Serial.print("wiggling ");
      Serial.println(range);
      // try winding backwards a little bit
      Serial.println("Testing in backwards direction.");
      hand.move(-(range*stepSize));
      while (hand.distanceToGo() != 0) {
        Serial.println("Running backwards...");
        hand.run();
        if (digitalRead(limitPin) != closedState) {
          Serial.println("Limit switch released (1).");
          break;
        }
      }
      // check if that worked,
      if (digitalRead(limitPin) == closedState) {
        // looks like backwards didn't work, lets try forwards the same distance
        Serial.println("Testing in forwards direction.");
        hand.move(2*(range*stepSize));
        while (hand.distanceToGo() != 0) {
          Serial.println("Running forwards...");
          hand.run();
          if (digitalRead(limitPin) != closedState) {
            Serial.println("Limit switch released (2).");
            distanceToBackOff = abs(distanceToBackOff);
            break;
          }
        }
      }
      // if either of these worked, then break out this iterating loop
      if (digitalRead(limitPin) != closedState) {
        Serial.println("Limit switch released (3).");
        break;
      }
    }
    // and back off a bit further
    Serial.print("Backing off ");
    Serial.println(distanceToBackOff);
    hand.move(distanceToBackOff);
    while (hand.distanceToGo() != 0) {
      hand.run();
    }
  }
  
  
  hand.disableOutputs();
}
void homeHands()
{
  // check to see if any sensors are already triggered and move the carriage if it is
  clearEndStops(minuteLimitPin, minuteHand, LOW);
  mLimitTriggered = false;
  clearEndStops(hourLimitPin, hourHand, HIGH);
  hLimitTriggered = false;
  
  rewindHands();
  // so now test the full length
  Serial.println("Winding forward to detect length of clock...");
  detectLengthOfClock();

  mDir = FORWARD;
  hDir = FORWARD;
  
  minuteHand.moveTo(minuteHand.currentPosition());
  hourHand.moveTo(hourHand.currentPosition());
  minuteHand.setSpeed(0);
  hourHand.setSpeed(0);

  Serial.println("HOMED!!");
  Serial.print("Minute axis is ");
  Serial.print(stepsPerClockMinute);
  Serial.print(" steps long (");
  Serial.print(stepsPerClockMinute*stepSize);
  Serial.println(" microsteps).");
  Serial.print("Hour axis is ");
  Serial.print(stepsPerClockHour);
  Serial.print(" steps long (");
  Serial.print(stepsPerClockHour*stepSize);
  Serial.println(" microsteps).");
}

void rewindHands()
{
  // run backwards, until sensor is triggered.
  Serial.println("Minute hand not home.");
  minuteWinding = true;
  minuteHand.enableOutputs();
  
  Serial.println("Hour hand not at home.");
  hourWinding = true;
  hourHand.enableOutputs();

  minuteHand.setCurrentPosition(0);
  minuteHand.move(-2147483648L);
  hourHand.setCurrentPosition(0);
  hourHand.move(-2147483648L);
  
  Serial.println("Winding backwards.");

  while (minuteWinding || hourWinding) {
    if (debugTriggers) {
      Serial.print("A Triggered M:");
      Serial.print(mLimitTriggered);
      Serial.print(", H:");
      Serial.println(hLimitTriggered);
    }

    if (mLimitTriggered) {
      Serial.println("M limit triggered.");
      minuteWinding = false;
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      // creep until trigger cleared
      minuteHand.move(100*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        if (debugToSerial) {
          Serial.print("minute creeping... ");
          Serial.print(minuteHand.distanceToGo());
          Serial.print(", speed: ");
          Serial.println(minuteHand.speed());
        }
        minuteHand.run();
        if (digitalRead(minuteLimitPin) == mLimitPulled) {
          Serial.println("Limit switch minute released (a).");
          minuteHand.moveTo(minuteHand.currentPosition());
          minuteHand.setSpeed(0);
          break;
        }
      }
      minuteHand.move(END_MARGIN*stepSize);
      while (minuteHand.distanceToGo() != 0) {
        if (debugTriggers) {
          Serial.print("Adding margin ");
          Serial.print(minuteHand.distanceToGo());
          Serial.print(", speed: ");
          Serial.println(minuteHand.speed());
        }
        minuteHand.run();
      }
      
      mLimitTriggered = false;
//      minuteHand.disableOutputs();
      minuteHand.setCurrentPosition(0);
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteHand.setSpeed(0);
      Serial.println("Minute rewind finished.");
    }
    if (minuteWinding) {
      minuteHand.run();
      if (debugToSerial) {
        Serial.print("rewind minutes pos ");
        Serial.println(minuteHand.currentPosition());
      }
    }
    if (debugTriggers) {
      Serial.print("B Triggered M:");
      Serial.print(mLimitTriggered);
      Serial.print(", H:");
      Serial.println(hLimitTriggered);
    }
    
    if (hLimitTriggered) {
      hourWinding = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(hourHand.currentPosition());
      // creep until trigger cleared
      hourHand.move(100*stepSize);
      while (hourHand.distanceToGo() != 0) {
        hourHand.run();
        if (digitalRead(hourLimitPin) == hLimitPulled) {
          hourHand.moveTo(hourHand.currentPosition());
          Serial.println("Limit switch hour released (b).");
          break;
        }
      }
      hourHand.move(END_MARGIN*stepSize);
      while (hourHand.distanceToGo() != 0) hourHand.run();

//      hourHand.disableOutputs();
      hLimitTriggered = false;
      hourHand.setCurrentPosition(0);
      hourHand.moveTo(hourHand.currentPosition());
      hourHand.setSpeed(0);
      
      Serial.println("Hour rewind finished.");
    }
    
    if (hourWinding) {
      hourHand.run();
      if (debugToSerial) {
        Serial.print("rewind hours pos ");
        Serial.println(hourHand.currentPosition());
      }
    }
  }
}

void detectLengthOfClock()
{
  if (debugToSerial) {
    Serial.print("Minute current position: ");
    Serial.print(minuteHand.currentPosition());
    Serial.print(", distance to go: ");
    Serial.print(minuteHand.distanceToGo());
    Serial.print(", speed: ");
    Serial.print(minuteHand.speed());
    Serial.print(", target:" );
    Serial.println(minuteHand.targetPosition());
    Serial.print("Hour current position: ");
    Serial.print(hourHand.currentPosition());
    Serial.print(", distance to go: ");
    Serial.print(hourHand.distanceToGo());
    Serial.print(", speed: ");
    Serial.print(hourHand.speed());
    Serial.print(", target:" );
    Serial.println(hourHand.targetPosition());
  }


  minuteWinding = true;
  hourWinding = true;
  minuteHand.enableOutputs();
  hourHand.enableOutputs();
  
  minuteHand.setMaxSpeed(maxSpeed);
  hourHand.setMaxSpeed(maxSpeed);

  minuteHand.moveTo(2147483647L);
  hourHand.moveTo(2147483647L);
 
  while (minuteWinding || hourWinding) {
    if (mLimitTriggered && minuteWinding) {
      Serial.println("Minute limit triggered.");
      minuteWinding = false;
      Serial.print("Current position at trigger ");
      Serial.println(minuteHand.currentPosition());
      minuteHand.moveTo(minuteHand.currentPosition());
      minuteHand.setSpeed(0);

      // creep until trigger cleared
      minuteHand.move(-(100*stepSize));
      while (minuteHand.distanceToGo() != 0) {
        if (debugToSerial) {
          Serial.print("min reverse creeping... ");
          Serial.print(minuteHand.distanceToGo());
          Serial.print(", speed: ");
          Serial.println(minuteHand.speed());
        }
        minuteHand.run();
        if (digitalRead(minuteLimitPin) == mLimitPulled) {
          Serial.println("Limit switch released (c).");
          minuteHand.moveTo(minuteHand.currentPosition());
          break;
        }
      }

      minuteHand.move(-(END_MARGIN*stepSize));
      while (minuteHand.distanceToGo() != 0) {
        if (debugToSerial) {
          Serial.print("Adding minute margin ");
          Serial.print(minuteHand.distanceToGo());
          Serial.print(", speed: ");
          Serial.println(minuteHand.speed());
        }
        minuteHand.run();
      }
      stepsPerClockMinute = minuteHand.currentPosition()/stepSize;
      recalculateStepsPerUnits();
      mLimitTriggered = false;
      Serial.println("Minute size detection finished.");
    }

    if (hLimitTriggered && hourWinding) {
      hourWinding = false;
      hourHand.moveTo(hourHand.currentPosition());
      hourHand.setSpeed(0);
      
      // creep until trigger cleared
      hourHand.move(-(100*stepSize));
      while (hourHand.distanceToGo() != 0) {
        if (debugToSerial) {
          Serial.print("hour reverse creeping... ");
          Serial.print(hourHand.distanceToGo());
          Serial.print(", speedd: ");
          Serial.println(hourHand.speed());
        }
        hourHand.run();
        if (digitalRead(hourLimitPin) == hLimitPulled) {
          Serial.println("Limit switch released (d).");
          hourHand.moveTo(hourHand.currentPosition());
          break;
        }
      }

      hourHand.move(-(END_MARGIN*stepSize));
      while (hourHand.distanceToGo() != 0) {
        if (debugToSerial) {
          Serial.print("Adding hour margin ");
          Serial.print(hourHand.distanceToGo());
          Serial.print(", speedd: ");
          Serial.println(hourHand.speed());
        }
        hourHand.run();
      }
      stepsPerClockHour = hourHand.currentPosition()/stepSize;
      recalculateStepsPerUnits();
      hLimitTriggered = false;
      Serial.println("Hour size detection finished.");
    }
    
    if (hourWinding) {
      hourHand.run();
    }      
    if (minuteWinding) {
      minuteHand.run();
    }
  }
  minuteHand.disableOutputs();
  hourHand.disableOutputs();
}

void moveHands()
{
  if (abs(hourHand.distanceToGo()) >= stepSize || abs(minuteHand.distanceToGo()) >= stepSize)
  {
    if (!motorsEnabled) {
      hourHand.enableOutputs();
      minuteHand.enableOutputs();
      motorsEnabled = true;
    }

    if (false) {
      Serial.print("Moving to ");
      Serial.print(currentHourPos);
      Serial.print(":");
      Serial.print(currentMinutePos);
      Serial.print(" (to go ");
      Serial.print(hourHand.distanceToGo());
      Serial.print(":");
      Serial.print(minuteHand.distanceToGo());
      Serial.print("), speed: ");
      Serial.print(hourHand.speed());
      Serial.print(":");
      Serial.println(minuteHand.speed());
    }
    

    if (hLimitTriggered)
      Serial.println("hlimit triggered.");
    else {
      if (debugToSerial) Serial.println("running hour!");
      hourHand.run();
    }
      
    if (mLimitTriggered)
      Serial.println("mlimit triggered.");
    else {
      if (debugToSerial) Serial.println("running hour!");
      minuteHand.run();
    }
    if (abs(hourHand.distanceToGo()) < stepSize && abs(minuteHand.distanceToGo()) < stepSize) {
      Serial.println("Disabling motors until there's enough steps to take");
      hourHand.disableOutputs();
      minuteHand.disableOutputs();
      motorsEnabled = false;
    }
  }
}


String reportPosition()
{
  Serial.println("rp");
  Serial.print(F("Current Position: "));
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
  Serial.print(F("Current target: "));
  Serial.print(hourHand.targetPosition());
  Serial.print(":");
  Serial.print(minuteHand.targetPosition());
  Serial.print(", (togo: ");
  Serial.print(hourHand.distanceToGo());
  Serial.print(":");
  Serial.print(minuteHand.distanceToGo());
  Serial.print("), speed: ");
  Serial.print(hourHand.speed());
  Serial.print(":");
  Serial.println(minuteHand.speed());
  Serial.println("rpend");
}  

void findTimeToDisplay()
{
  if (millis() - timeChecked > 500)
  {
    DateTime now = RTC.now();
    if (debugToSerial) {
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
    }
  
    currentHours = now.hour();
    if (currentHours >= 12)
      currentHours = currentHours - 12;
    currentMinutes = now.minute();
    currentSeconds = now.second();
    timeChecked = millis();
  }
}

void setHandPositions()
{
  displayHour(currentHours, currentMinutes);
  displayMinute(currentMinutes);
}

void displayHour(long hour, long minute)
{
  // work out the new position and set it globally eg time 4:25.
  // first hours
  // eg 0.2055 * 4 * 60 = 49.333
  float justHours = stepsPerHourMinute * hour * 60;
  // eg 0.2055 * 25 = 5.13888
  float justMinutes = stepsPerHourMinute * minute;
  
  // stick em together: position is 54.472 (4:25)
  long oldHourPos = currentHourPos;
  currentHourPos = justHours + justMinutes;

  if (currentHourPos != oldHourPos)
  {
    Serial.print("Moving hour hand to ");
    Serial.print(currentHourPos);
    Serial.print(" (");
    Serial.print(currentHourPos*stepSize);
    Serial.print(")");
    Serial.print(", to represent ");
    Serial.println(hour);
    hourHand.moveTo(currentHourPos*stepSize);
  }
}

void displayMinute(long minute)
{
  //Serial.println("dm");
  long oldMinutePos = currentMinutePos;
  // work out the new position and set it globally
  // eg 2.467 * 25 = 61.675
  currentMinutePos = stepsPerMinute * minute;

  if (currentMinutePos != oldMinutePos)
  {
    Serial.print("Moving minute hand to position ");
    Serial.print(currentMinutePos);
    Serial.print(" (");
    Serial.print(currentMinutePos*stepSize);
    Serial.print(")");
    Serial.print(", to represent ");
    Serial.println(minute);
    
    minuteHand.moveTo(currentMinutePos*stepSize);
  }
}

