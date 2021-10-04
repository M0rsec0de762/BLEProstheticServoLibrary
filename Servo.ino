/*
----------------------------------
-----Includes-----
----------------------------------
*/

/*
----------------------------------
-------------Type Definitions-----
----------------------------------
*/

//Speed of the servo as ms/180 deg
#define SERVOSPEED 390
#define FASTMOVE 1
#define SLOWMOVE 6
#define REDUCTIONSTEPS 17

/*Type definition for a structure regarding servos.*/
typedef struct ServoType
{
  unsigned int zServoPin;
  // unsigned int zServoPeriod;
  // unsigned int zServoDutyCycle;
  unsigned int zServoPWMOpen; 
  unsigned int zServoPWMClosed;   // Note: Open and Closed values represent the limits of the PWM
  unsigned int zServoPWMCurrent;
};

/*
----------------------------------
-----Function Prototypes-----
----------------------------------
*/
/*
  Function Name: Servo_init
  Input: ServoType Structure (A Servo)
  Purpose: Initializes the pins needed to drive the servo.
*/
void servoInit(ServoType *Serv);
/*
  Function Name: Servo_Move
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: 
    Move servo to the desiredPWM's corresponding position(within the acceptable range).
    Update the current PWM variable of the servo.
*/
void servoMove(ServoType* Serv, unsigned int desiredPWM);
/*
  Function Name: Servo_Move_Segmented
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: Move servo to the desiredPWM position using incremental PWM values between the current angle and the desiredPWM
*/
void servoMoveSegmented(ServoType *Serv, unsigned int desiredPWM, unsigned int seg);

/*Max's Math Code. Comment this stuff later*/
long constrain(long x, long in_min, long in_max) 
long map(long x, long in_min, long in_max, long out_min, long out_max) 

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
  /*Init servo with only the pwm pin, and  */
  testServo.zServoPin = 3;
  testServo.zServoPWMOpen = 250;
  testServo.zServoPWMClosed = 65;
  servoInit(&testServo);
  delay(1000);
  Serial.println("Init Complete");  // Debug Code
  servoMove(&testServo, 250);
  delay(1000);
  servoMove(&testServo, 65);
}
https://docs.google.com/document/d/1P_BeJH5wWfhm9kize7KvitGLEQI_--hjxDYvU9wtM60/edit
void loop() 
{
  servoMoveSegmented(&testServo, 65,7);
  Serial.print("Move 1 to ");
  Serial.println(testServo.zServoPWMCurrent);   // Debug Code
  delay(1000);
  Servo_Move_Segmented(&testServo, 250,20);
  Serial.print("Move 2 to ");
  Serial.println(testServo.zServoPWMCurrent);   // Debug Code
  delay(1000);
}


/*
----------------------------------
---Function Initializations-------
----------------------------------
*/

long constrain(long x, long in_min, long in_max) 
{
  if (x < in_min) return in_min;
  else if (x > out_min) return in_max;
  return x;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servoInit(ServoType *Serv)
{
  pinMode(Serv->zServoPin,OUTPUT);
}

void servoMove(ServoType* Serv, unsigned int desiredPWM)
{
  //unsigned int newPWM;
  /* Limits PWM Values between the 'open' and 'closed' values*/
  Serv->zServoPWMCurrent = constrain(desiredPWM, Serv->zServoPWMClosed, Serv->zServoPWMOpen);
  /*if(desiredPWM >= Serv->zServoPWMOpen)
  {
    newPWM = Serv->zServoPWMOpen;
  }
  else if (desiredPWM <= Serv->zServoPWMClosed)
  {
    newPWM = Serv->zServoPWMClosed;
  }
  else
  {
    newPWM = desiredPWM;
  }*/

  /* Creates new current value and writes it to the pin */
  //Serv->zServoPWMCurrent = newPWM;
  analogWrite(Serv->zServoPin,newPWM);
}

void Servo_Move_Segmented(ServoType *Serv, unsigned int desiredPWM, unsigned int seg)
{
  /* Calculates signed difference between starting and final PWM*/
  int diff = (int)Serv->zServoPWMCurrent - (int)desiredPWM;

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
      if(Serv->zServoPWMCurrent - (segDiff) < desiredPWM)
      {
        Serv->zServoPWMCurrent = desiredPWM;
        segDiff = 0;
      }
    }
    else if(segDiff < 0)
    {
      if(Serv->zServoPWMCurrent - (segDiff) > desiredPWM)
      {
        Serv->zServoPWMCurrent = desiredPWM;
        segDiff = 0;
      }
    }
    //-------------------TEST CODE END--------------------
    Serial.println(Serv->zServoPWMCurrent - (segDiff)); // Debug Code

    Servo_Move(Serv,Serv->zServoPWMCurrent - (segDiff));

    delay(90);
  }
}

void Servo_Invert_At_Speed(ServoType *Serv, uint8_t isFast)
{
  uint8_t openServo = 0;
  
  //Determine if opening or closing
  if(Serv->zServoPWMCurrent == Serv->zServoPWMClosed)
  {
    openServo = 1;
  }
  
  //Determine delay needed at each position 
  int totalTime = SERVOSPEED;

  //Multiples total time for inversion based on speed chosen
  if(!isFast)
  {
    totalTime *= SLOWMOVE; 
  }

  //Calculates time delay after each segment of the inversion
  int timeDelay = (totalTime - SERVOSPEED) / REDUCTIONSTEPS;
  
  unsigned int pwmStep = (Serv->zServoPWMOpen - Serv->zServoPWMClosed) / REDUCTIONSTEPS;

  //move servo step by step
  for(int i = 0; i < REDUCTIONSTEPS; ++i)
  {
    if(openServo)
    {
      Servo_Move(Serv,Serv->zServoPWMCurrent + pwmStep);
    }
    else
    {
      Servo_Move(Serv,Serv->zServoPWMCurrent - pwmStep);
    }
    
    delay(timeDelay);
  }
}