//4 Motor Arduino Roaming Bot by Ryan Cole.
#include "LowPower.h"
#include "millisDelay.h"


//Variable Setup =======================================================================
const byte dirLeft = 8;       //direction of Left motors
const byte leftPWM = 10;           //speed pin for Left motors (PWM mapped to Channel 1 and 3 of motor driver)
const byte rightPWM = 11;           //speed pin for Right motors (PWM mapped to Channel 2 and 4 of motor driver)
const byte dirRight = 12;      //direction of Right motors
const byte buttonPin = 4;
const byte onBoardLED = 13;
const byte frontPing = 7;
const byte bottomPing = 2;

bool fwdTimerStarted = false;

byte pingPin = frontPing; 
byte maxSpeed = 65; //0-255, 60 too low?

int accRate = 50;  //corresponds to delay time. Was 5, 100 too slow?

long cm;

millisDelay fwdTimer;

enum movementStatus {movingFWD, movingBCK, movingLFT, movingRGT, stopped};
movementStatus currentMovement = stopped;


//================================================================================
void setup()
{
  Serial.begin(9600);    // initialize serial communication

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(onBoardLED, OUTPUT);

  pinMode(dirLeft, OUTPUT);      // set Motor pins as output
  pinMode(dirRight, OUTPUT);

  randomSeed(analogRead(0));    //on unconnected pin to pick up noise

  fwdTimer.start(3000);
}



//Functions =================================================================
long MicrosecToCM(long microsec)
{
  /* The speed of sound is 34431 cm/s @ 22 C or ~29 microseconds per centimeter.
  The ping travels out and back, so to find the distance of the object we
  take half of the distance travelled. */ 
  return microsec / 29 / 2;
}

bool IsStalled(int botSpeed)
{
  int stallThreshold = 2; //slowest acceptable speed for stall status in cm per sec. Measurement error +/- 2cm.
  
  if (botSpeed <= stallThreshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*bool InCorner()
{
  if (AvoidanceRoutine used x times in y seconds)
  {
    return true;
  }
  else
  {
    return false;
  }
}*/

int getSpeed() // Get speed in cm per ms
{
    Ping();
    long posA = cm;
    delay(1000); //1000 creates 1ms for cm/ms
    Ping();
    long posB = cm;
    return abs(posA - posB);
    //Serial.println("speed is ");
    //Serial.print(botSpeed);
}

void Ping()
{
     /* The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
     Give a short LOW pulse beforehand to ensure a clean HIGH pulse: */
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    /* The same pin is used to read the signal from the PING))): a HIGH pulse
     whose duration is the time (in microseconds) from the sending of the ping
     to the reception of its echo off of an object. */
    pinMode(pingPin, INPUT);
    long duration = pulseIn(pingPin, HIGH);

    cm = MicrosecToCM(duration); // convert the time from Ping into a distance

    //Serial.println(cm);
    //Serial.print("cm");
}

void SwitchPingSensor() //Switch front and bottom sensors
{
    if (pingPin == frontPing)
    {
      pingPin = bottomPing;
    }
    else
    {
      pingPin = frontPing;
    }
}

void RightOrLeft(long randNumber, int maxSpeed)
{
  if (randNumber % 2 == 0)
  {
    TurnRight();
    Accelerate(5,100); 
    //AccelerateLog(maxSpeed);
  }
  else
  {
    TurnLeft();
    Accelerate(5,100);
    //AccelerateLog(maxSpeed); 
  }
}

//Movement Section
void MoveForward()
{
  //Serial.println("Moving Forward");
  
  if (currentMovement != movingFWD)
  {
    digitalWrite(dirRight, LOW);
    digitalWrite(dirLeft, LOW);
    currentMovement = movingFWD;
  }
  else //maintain forward top speed
  {
    analogWrite(leftPWM, maxSpeed);
    analogWrite(rightPWM, maxSpeed);
  }
}

void StopMovement()
{
  //Serial.println("STOP");

  if (currentMovement != stopped)
  {
    analogWrite(leftPWM, 0);
    analogWrite(rightPWM, 0);
    currentMovement = stopped;
  }
}

void MoveBackward()
{
  //Serial.println("Moving Backward");
  digitalWrite(dirRight, HIGH);
  digitalWrite(dirLeft, HIGH);
  currentMovement = movingBCK;
}

void TurnLeft()
{
  //Serial.println("Moving Left");
  digitalWrite(dirRight, LOW);
  digitalWrite(dirLeft, HIGH);
  currentMovement = movingLFT;
}

void TurnRight()
{
  //Serial.println("Moving Right");
  digitalWrite(dirRight, HIGH);
  digitalWrite(dirLeft, LOW);
  currentMovement = movingRGT;
}

//Accelerate and decelerate. In the future change these linear functions to logarithmic?
void Accelerate(int accRate, int maxSpeed) 
{
  //Serial.println("Accelerating");
  for(int motorValue = 0 ; motorValue <= maxSpeed; motorValue +=1)
  {
    analogWrite(leftPWM, motorValue);
    analogWrite(rightPWM, motorValue);
    delay(accRate);
  } 
}

//log test. 
void AccelerateLog(int maxSpeed)
{
  long accRate = 0;
  for(int motorValue = 0 ; motorValue <= maxSpeed; motorValue +=1)
  {
    analogWrite(leftPWM, motorValue);
    analogWrite(rightPWM, motorValue);
    delay(accRate);
    accRate = 17*log(motorValue + 1); //17ln(x) too slow for maxSpeed = 65 and accRate = 50
  } 
} 

void Decelerate(int accRate, int maxSpeed)
{
  //Serial.println("Decelerating");
  for(int motorValue = maxSpeed ; motorValue > 0; motorValue -=1)
  {
    analogWrite(leftPWM, motorValue);
    analogWrite(rightPWM, motorValue);
    delay(accRate);      
 }
}

//Checks and Routines
void AvoidanceRoutine()
{
  //Serial.println("Avoidance Routine");
  if (cm < 15 && cm > 14 && pingPin == frontPing) //slow down for approaching obstacles
  {
    Decelerate(5, maxSpeed); 
  }
  
  StopMovement();
  MoveBackward();
  // random number from 1 to 45 to increase chance of getting out of corners.
  long randNumber = random(1,46);
  Accelerate(randNumber, maxSpeed); 
  //AccelerateLog(maxSpeed);
  Decelerate(45, maxSpeed); 
  StopMovement();
  delay(100);

  //random 1 or 2 to randomize left and right directions.
  randNumber = random(1,3); 
  RightOrLeft(randNumber, maxSpeed);
  
  // random number from 1400 to 4200 to increase chance for getting out of corners.
  randNumber = random(1400,4200);
  delay(randNumber); //1400 gives close to 90 degrees for my motors/acceleration rate/max speed. 
}

void ObstacleCheck()
{
    if (cm < 15 && pingPin == frontPing)
    {
      Serial.println("WARNING: Collision imminent!");
      AvoidanceRoutine();
    }
    else if (cm > 15 && pingPin == bottomPing)
    {
      Serial.println("WARNING: Fall imminent!");
      AvoidanceRoutine();
    }
    else
    {
      if (currentMovement == stopped)
      {
        MoveForward();
        Accelerate(accRate,maxSpeed);
        //AccelerateLog(maxSpeed);
      }
      else
      {
        MoveForward();
      }  
    }
}

void StallCheck()
{
    if (pingPin == frontPing && cm >= 15 && currentMovement == movingFWD)
    {
      if (!fwdTimerStarted)
      {
        fwdTimer.repeat(); //start three second non-blocking timer
        fwdTimerStarted = true;
      }
      
      if (fwdTimer.justFinished())
      {
        int botSpeed = getSpeed();
        bool stallDetected = IsStalled(botSpeed);
        fwdTimerStarted = false;
        
        if (stallDetected)
        {
          AvoidanceRoutine();
          stallDetected = false;
        }
       } 
     }
}



//===================================================================================
void loop()
{
  int sensorVal = digitalRead(buttonPin);
  //Serial.println(sensorVal);
  //Serial.println("currentMovement is ");
  //Serial.print(currentMovement);

  if (sensorVal == HIGH)
  { //Continue program (Logic is inverted with INPUT_PULLUP)
    digitalWrite(onBoardLED, LOW);

    Ping();

    ObstacleCheck();

    StallCheck();

    SwitchPingSensor();

    //delay(8);
  }
  else //Stop Program
  {
    digitalWrite(onBoardLED, HIGH);
    StopMovement();
    //delay(500);
    LowPower.idle(SLEEP_500MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  }
}
