// The following tutorials were used to create this library:
// https://www.arduino.cc/en/Hacking/LibraryTutorial
// https://www.arduino.cc/en/guide/libraries
#include "Arduino.h"
#include "Servo.h"
#include "EECS473Servo.h"

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

void servoInit(ServoType *Serv)
{
  Serv->ZServ.attach(Serv->z_servo_pin);
  pinMode(Serv->z_MOSFET_pin,OUTPUT);
  digitalWrite(Serv->z_MOSFET_pin,HIGH);
  servoMove(Serv,Serv->z_servo_micro_open);

  if(Serv->z_servo_micro_closed < Serv->z_servo_micro_min)
    Serv->z_servo_micro_closed = Serv->z_servo_micro_min;
  else if(Serv->z_servo_micro_closed > Serv->z_servo_micro_max)
    Serv->z_servo_micro_closed = Serv->z_servo_micro_max;

  if(Serv->z_servo_micro_open < Serv->z_servo_micro_min)
    Serv->z_servo_micro_open = Serv->z_servo_micro_min;
  else if(Serv->z_servo_micro_open > Serv->z_servo_micro_max)
    Serv->z_servo_micro_open = Serv->z_servo_micro_max;
}

/*DONT USE servoMove in the main loop, ONLY USE servoOpen and servoClosed. They have MOSFET control code*/
void servoMove(ServoType* Serv, unsigned int desiredMicro)
{
  unsigned int newMicro;
  /* Limits micro Values between the 'open' and 'closed' values*/
  if(desiredMicro >= Serv->z_servo_micro_max)
  {
    newMicro = Serv->z_servo_micro_max;
  }
  else if (desiredMicro <= Serv->z_servo_micro_min)
  {
    newMicro = Serv->z_servo_micro_min;
  }
  else
  {
    newMicro = desiredMicro;
  }

  /* Creates new current value and writes it to the pin */
  Serv->z_servo_micro_current = newMicro;
  Serv->ZServ.writeMicroseconds(newMicro);
}

void servoMoveSegmented(ServoType *Serv, unsigned int desiredMicro, unsigned int seg)
{
  /* Calculates signed difference between starting and final Micro*/
  int diff = (int)Serv->z_servo_micro_current - (int)desiredMicro;

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
      if(Serv->z_servo_micro_current - (segDiff) < desiredMicro)
      {
        Serv->z_servo_micro_current = desiredMicro;
        segDiff = 0;
      }
    }
    else if(segDiff < 0)
    {
      if(Serv->z_servo_micro_current - (segDiff) > desiredMicro)
      {
        Serv->z_servo_micro_current = desiredMicro;
        segDiff = 0;
      }
    }
    //-------------------TEST CODE END--------------------
    Serial.println(Serv->z_servo_micro_current - (segDiff)); // Debug Code

    servoMove(Serv,Serv->z_servo_micro_current - (segDiff));

    //ADDED DELAY MATH
    unsigned int dlay = 0;

    if(segDiff < 0)
      dlay = (unsigned int)(((map(segDiff, 0, -2000, 0 , 180) / 60.0) * Serv->z_servo_speed) + 1);
    else
      dlay = (unsigned int)(((map(segDiff, 0, 2000, 0 , 180) / 60.0) * Serv->z_servo_speed) + 1);
    
    //Serial.println(dlay);
      
    delay(dlay);
  }

}

void servoOpen(ServoType *Serv)
{
  digitalWrite(Serv->z_MOSFET_pin,HIGH);
  servoMoveSegmented(Serv,Serv->z_servo_micro_open,20);
  digitalWrite(Serv->z_MOSFET_pin,LOW);
}

void servoClosed(ServoType *Serv)
{
  digitalWrite(Serv->z_MOSFET_pin,HIGH);
  servoMoveSegmented(Serv,Serv->z_servo_micro_closed,20);
}