/*
----------------------------------
-----Includes-----
----------------------------------
*/
#include <Servo.h>
/*
----------------------------------
-------------Type Definitions-----
----------------------------------
*/

//Speed of the servo as ms/180 deg
#define SERVO_SPEED 390
#define FAST_MOVE 1
#define SLOW_MOVE 6
#define REDUCTION_STEPS 17

/*Type definition for a structure regarding servos.*/
typedef struct ServoType
{
  Servo ZServ; // Uses <Servo.h>'s class in order to init and control the servo
  unsigned int z_servo_pin;
  unsigned int z_servo_micro_open; 
  unsigned int z_servo_micro_closed;   // Note: Open and Closed values represent the limits of the micro
  unsigned int z_servo_micro_current;
  unsigned int z_servo_micro_max;
  unsigned int z_servo_micro_min;
  unsigned int z_MOSFET_pin;
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

/*
  Function Name: Servo_Invert_At_Speed
  Input: 
    ServoType Structure (A Servo)
    Speed will be moving fast or slow (1 or 0)
    
  Purpose: Invert the position of the servo (close to open and vice-versa) at an arbitrary fast or slow speed*/
  
void Servo_Invert_At_Speed(ServoType *Serv, uint8_t isFast);

/*
----------------------------------
-------------Main Code------------
----------------------------------
*/
ServoType testServo;

void setup() 
{
  /*Init Serial Line for Arduino debugging purposes*/
  Serial.begin(9600);
  /*Init servo with only the micro pin, and open/closed pulse widths*/
  testServo.z_servo_pin = 3;
  testServo.z_servo_micro_open = 2500;
  testServo.z_servo_micro_closed = 500;
  testServo.z_servo_micro_max = 2500;
  testServo.z_servo_micro_min = 500;
  testServo.z_MOSFET_pin = 8;
  testServo.z_servo_speed = 210;
  servoInit(&testServo);
  Serial.println("Init Complete");  // Debug Code

}

void loop() 
{
  servoOpen(&testServo);
  Serial.print("Move 1 to open: ");
  Serial.println(testServo.z_servo_micro_current);   // Debug Code
  delay(1000);
  servoClosed(&testServo);
  Serial.print("Move 2 to closed: ");
  Serial.println(testServo.z_servo_micro_current);   // Debug Code
  delay(1000);
}


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
  else if(Serv->z_servo_micro_closed < Serv->z_servo_micro_max)
    Serv->z_servo_micro_closed = Serv->z_servo_micro_max;
  if(Serv->z_servo_micro_open < Serv->z_servo_micro_min)
    Serv->z_servo_micro_open = Serv->z_servo_micro_min;
  else if(Serv->z_servo_micro_open < Serv->z_servo_micro_max)
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
      dlay = (unsigned int)((map(segDiff, 0, -2000, 0 , 60) / 60.0) * Serv->z_servo_speed);
    else
      dlay = (unsigned int)((map(segDiff, 0, 2000, 0 , 60) / 60.0) * Serv->z_servo_speed);
    
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

void Servo_Invert_At_Speed(ServoType *Serv, uint8_t isFast)
{
  uint8_t openServo = 0;
  
  //Determine if opening or closing
  if(Serv->z_servo_micro_current == Serv->z_servo_micro_closed)
  {
    openServo = 1;
  }
  
  //Determine delay needed at each position 
  int totalTime = SERVO_SPEED;

  //Multiples total time for inversion based on speed chosen
  if(!isFast)
  {
    totalTime *= SLOW_MOVE; 
  }

  //Calculates time delay after each segment of the inversion
  int timeDelay = (totalTime - SERVO_SPEED) / REDUCTION_STEPS;
  
  unsigned int microStep = (Serv->z_servo_micro_open - Serv->z_servo_micro_closed) / REDUCTION_STEPS;

  //move servo step by step
  for(int i = 0; i < REDUCTION_STEPS; ++i)
  {
    if(openServo)
    {
      servoMove(Serv,Serv->z_servo_micro_current + microStep);
    }
    else
    {
      servoMove(Serv,Serv->z_servo_micro_current - microStep);
    }
    
    delay(timeDelay);
  }
}