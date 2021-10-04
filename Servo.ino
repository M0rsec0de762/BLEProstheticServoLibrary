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
  // unsigned int zServoPeriod;
  // unsigned int zServoDutyCycle;
  unsigned int z_servo_PWM_open; 
  unsigned int z_servo_PWM_closed;   // Note: Open and Closed values represent the limits of the PWM
  unsigned int z_servo_PWM_current;
};

/*
----------------------------------
-----Function Prototypes-----
----------------------------------
*/
/*
  Function Name: servoInit
  Input: ServoType Structure (A Servo)
  Purpose: Initializes the pins needed to drive the servo.
*/
void servoInit(ServoType *Serv);
/*
  Function Name: servoMove
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: 
    Move servo to the desiredPWM's corresponding position(within the acceptable range).
    Update the current PWM variable of the servo.
*/
void servoMove(ServoType* Serv, unsigned int desiredPWM);
/*
  Function Name: servoMoveSegmented
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: Move servo to the desiredPWM position using incremental PWM values between the current angle and the desiredPWM
*/
void servoMoveSegmented(ServoType *Serv, unsigned int desiredPWM, unsigned int seg);

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
  /*Init servo with only the pwm pin, and open/closed pulse widths*/
  testServo.z_servo_pin = 3;
  testServo.z_servo_PWM_open = 2500;
  testServo.z_servo_PWM_closed = 500;
  servoInit(&testServo);
  Serial.println("Init Complete");  // Debug Code
}
//https://docs.google.com/document/d/1P_BeJH5wWfhm9kize7KvitGLEQI_--hjxDYvU9wtM60/edit
void loop() 
{
  // servoMoveSegmented(&testServo, 2500,20);
  // Serial.print("Move 1 to ");
  // Serial.println(testServo.z_servo_PWM_current);   // Debug Code
  // delay(1000);
  // servoMoveSegmented(&testServo, 500,20);
  // Serial.print("Move 2 to ");
  // Serial.println(testServo.z_servo_PWM_current);   // Debug Code
  // delay(1000);
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
  servoMove(Serv,Serv->z_servo_PWM_open);
}

void servoMove(ServoType* Serv, unsigned int desiredPWM)
{
  unsigned int newPWM;
  /* Limits PWM Values between the 'open' and 'closed' values*/
  // Serv->z_servo_PWM_current = constrain(desiredPWM, Serv->z_servo_PWM_closed, Serv->z_servo_PWM_open);
  if(desiredPWM >= Serv->z_servo_PWM_open)
  {
    newPWM = Serv->z_servo_PWM_open;
  }
  else if (desiredPWM <= Serv->z_servo_PWM_closed)
  {
    newPWM = Serv->z_servo_PWM_closed;
  }
  else
  {
    newPWM = desiredPWM;
  }

  /* Creates new current value and writes it to the pin */
  Serv->z_servo_PWM_current = newPWM;
  Serv->ZServ.writeMicroseconds(newPWM);
}

void servoMoveSegmented(ServoType *Serv, unsigned int desiredPWM, unsigned int seg)
{
  /* Calculates signed difference between starting and final PWM*/
  int diff = (int)Serv->z_servo_PWM_current - (int)desiredPWM;

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
  /* Moves Servo to desired endpoint using segmented points in between the starting and final PWM */
  for(i = 0; i < seg; i++)
  {
    //-------------------TEST CODE START------------------
    /*Prevents code from writing beyond the desiredPWM value*/
    if(segDiff > 0)
    {
      if(Serv->z_servo_PWM_current - (segDiff) < desiredPWM)
      {
        Serv->z_servo_PWM_current = desiredPWM;
        segDiff = 0;
      }
    }
    else if(segDiff < 0)
    {
      if(Serv->z_servo_PWM_current - (segDiff) > desiredPWM)
      {
        Serv->z_servo_PWM_current = desiredPWM;
        segDiff = 0;
      }
    }
    //-------------------TEST CODE END--------------------
    Serial.println(Serv->z_servo_PWM_current - (segDiff)); // Debug Code

    servoMove(Serv,Serv->z_servo_PWM_current - (segDiff));

    delay(90);
  }
}

void Servo_Invert_At_Speed(ServoType *Serv, uint8_t isFast)
{
  uint8_t openServo = 0;
  
  //Determine if opening or closing
  if(Serv->z_servo_PWM_current == Serv->z_servo_PWM_closed)
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
  
  unsigned int pwmStep = (Serv->z_servo_PWM_open - Serv->z_servo_PWM_closed) / REDUCTION_STEPS;

  //move servo step by step
  for(int i = 0; i < REDUCTION_STEPS; ++i)
  {
    if(openServo)
    {
      servoMove(Serv,Serv->z_servo_PWM_current + pwmStep);
    }
    else
    {
      servoMove(Serv,Serv->z_servo_PWM_current - pwmStep);
    }
    
    delay(timeDelay);
  }
}