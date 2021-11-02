// The following tutorials were used to create this library:
// https://www.arduino.cc/en/Hacking/LibraryTutorial
// https://www.arduino.cc/en/guide/libraries
#include "Arduino.h"
#include "Servo.h"
#include "EECS473Servo.h"

/*
----------------------------------
---Variable Defines-------
----------------------------------
*/
#define SERVO_MAX_TRAVEL 150
#define SERVO_PULSE_RANGE 1400
/*
----------------------------------
---Function Initializations-------
----------------------------------
*/

// long constrain(long x, long in_min, long in_max) 
// {
//   if (x < in_min) return in_min;
//   else if (x > out_min) return in_max;
//   return x;
// }

// long map(long x, long in_min, long in_max, long out_min, long out_max) 
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

void ArmServo::servoInit()
{
  ZServ.attach(z_servo_pin);
  pinMode(z_MOSFET_pin,OUTPUT);
  digitalWrite(z_MOSFET_pin,HIGH);
  servoMove(z_servo_micro_open);

  if(z_servo_micro_closed < z_servo_micro_min)
    z_servo_micro_closed = z_servo_micro_min;
  else if(z_servo_micro_closed > z_servo_micro_max)
    z_servo_micro_closed = z_servo_micro_max;

  if(z_servo_micro_open < z_servo_micro_min)
    z_servo_micro_open = z_servo_micro_min;
  else if(z_servo_micro_open > z_servo_micro_max)
    z_servo_micro_open = z_servo_micro_max;
}

/*DONT USE servoMove in the main loop, ONLY USE servoOpen and servoClosed. They have MOSFET control code*/
void ArmServo::servoMove(unsigned int desiredMicro)
{
  unsigned int newMicro;
  /* Limits micro Values between the 'open' and 'closed' values*/
  if(desiredMicro >= z_servo_micro_max)
  {
    newMicro = z_servo_micro_max;
  }
  else if (desiredMicro <= z_servo_micro_min)
  {
    newMicro = z_servo_micro_min;
  }
  else
  {
    newMicro = desiredMicro;
  }

  /* Creates new current value and writes it to the pin */
  z_servo_micro_current = newMicro;
  ZServ.writeMicroseconds(newMicro);
}

void ArmServo::servoMoveSegmented(unsigned int desiredMicro, unsigned int seg)
{
  /* Calculates signed difference between starting and final Micro*/
  int diff = (int)z_servo_micro_current - (int)desiredMicro;

  int segDiff;
  /*Based on the sign of diff, this mitigates the possibility of integer division truncation*/
  if(diff > 0)
  {
    segDiff = (diff / (int)seg) + 1;
  }
  else if(diff < 0)
  {
    segDiff = (diff / (int)seg) - 1;
  }
  else
  {
    segDiff = (diff / (int)seg);
  }

  unsigned int i;
  /* Moves Servo to desired endpoint using segmented points in between the starting and final micro */
  for(i = 0; i < seg; i++)
  {
    //-------------------TEST CODE START------------------
    /*Prevents code from writing beyond the desiredMicro value*/
    if(segDiff > 0)
    {
      if(z_servo_micro_current - (segDiff) < desiredMicro)
      {
        z_servo_micro_current = desiredMicro;
        segDiff = 0;
      }
    }
    else if(segDiff < 0)
    {
      if(z_servo_micro_current - (segDiff) > desiredMicro)
      {
        z_servo_micro_current = desiredMicro;
        segDiff = 0;
      }
    }
    //-------------------TEST CODE END--------------------
    Serial.println(z_servo_micro_current - (segDiff)); // Debug Code

    servoMove(z_servo_micro_current - (segDiff));

    //ADDED DELAY MATH
    unsigned int dlay = 0;

    if(segDiff < 0)
      dlay = (unsigned int)(((map(segDiff, 0, -SERVO_PULSE_RANGE, 0 , SERVO_MAX_TRAVEL) / 60.0) * z_servo_speed) + 1);
    else
      dlay = (unsigned int)(((map(segDiff, 0, SERVO_PULSE_RANGE, 0 , SERVO_MAX_TRAVEL) / 60.0) * z_servo_speed) + 1);
    
    //Serial.println(dlay);
      
    delay(dlay);
  }

}

void ArmServo::servoOpen()
{
  digitalWrite(z_MOSFET_pin,HIGH);
  servoMoveSegmented(z_servo_micro_open,20);
  digitalWrite(z_MOSFET_pin,LOW);
}

void ArmServo::servoClosed()
{
  digitalWrite(z_MOSFET_pin,HIGH);
  servoMoveSegmented(z_servo_micro_closed,20);
}