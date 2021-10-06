// The following tutorials were used to create this library:
// https://www.arduino.cc/en/Hacking/LibraryTutorial
// https://www.arduino.cc/en/guide/libraries
#ifndef Morse_h
#define Morse_h
#include "Servo.h"
#include "Arduino.h"
/*
----------------------------------
-------------Type Definitions-----
----------------------------------
*/

/*Type definition for a structure regarding servos.*/
typedef struct ServoType
{
  Servo ZServ; // Uses <Servo.h>'s class in order to init and control the servo
  unsigned int z_servo_pin;
  /* 'micro' defines microseconds of high time within a PWM signal */
  unsigned int z_servo_micro_open; 
  unsigned int z_servo_micro_closed;  
  unsigned int z_servo_micro_current;
  unsigned int z_servo_micro_max;
  unsigned int z_servo_micro_min;
  unsigned int z_MOSFET_pin;
  /* Defines speed in degrees per second */
  unsigned int z_servo_speed;
};

/*
----------------------------------
-----Function Prototypes-----
----------------------------------
*/
/*
  Function Name: servoInit
  Input: ServoType Structure (A Servo)
  Purpose: Initializes the pins needed to drive the servo. Sets servo to open position
*/
void servoInit(ServoType *Serv);
/*
  Function Name: servoMove
  Input: 
    ServoType Structure (A Servo)
    Desired micro to move to
  Purpose: 
    Move servo to the desiredMicro's corresponding position(within the acceptable range).
    Update the current micro variable of the servo.
    Clamps open and closed micro values to min and max values if they exceed min and max values.
*/
void servoMove(ServoType* Serv, unsigned int desiredMicro);
/*
  Function Name: servoMoveSegmented
  Input: 
    ServoType Structure (A Servo)
    Desired micro to move to
  Purpose: Move servo to the desiredMicro position using incremental micro values between the current angle and the desiredMicro
*/
void servoMoveSegmented(ServoType *Serv, unsigned int desiredMicro, unsigned int seg);
/*
  Function Name: servoOpen
  Input: 
    ServoType Structure (A Servo)
  Purpose: Turn on MOSFET, open hand, turn off MOSFET
*/
void servoOpen(ServoType *Serv);
/*
  Function Name: servoOpen
  Input: 
    ServoType Structure (A Servo)
  Purpose: Turn on MOSFET, close hand, leave MOSFET on
*/
void servoClosed(ServoType *Serv);


/*Max's Math Code. Comment this stuff later*/
// long constrain(long x, long in_min, long in_max) 
// long map(long x, long in_min, long in_max, long out_min, long out_max) 

#endif