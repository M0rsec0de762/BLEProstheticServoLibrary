/*
  TODO:
  1. DETERMINE HOW SPEED OF HAND OPENING AND CLOSING SHOULD BE CONTROLLED
  2. DETERMINE "LEVELS" OF EACH API FUNCTION
*/
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
void Servo_init(ServoType *Serv);
/*
  Function Name: Servo_Move
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: 
    Move servo to the desiredPWM's corresponding position(within the acceptable range).
    Update the current PWM variable of the servo.
*/
void Servo_Move(ServoType* Serv, unsigned int desiredPWM);
/*
  Function Name: Servo_Move_Segmented
  Input: 
    ServoType Structure (A Servo)
    Desired PWM to move to
  Purpose: Move servo to the desiredPWM position using incremental PWM values between the current angle and the desiredPWM
*/
void Servo_Move_Segmented(ServoType *Serv, unsigned int desiredPWM, unsigned int seg);
/*
----------------------------------
-------------Main Code------------
----------------------------------
*/
ServoType testServo;

void setup() 
{
  /*InitSerial Line for Arduino debugging purposes*/
  //Serial.begin(9600);
  /*Init servo with only the pwm pin, and  */
  testServo.zServoPin = 3;
  testServo.zServoPWMOpen = 200;
  testServo.zServoPWMClosed = 110;
  Servo_init(&testServo);
  delay(1000);
  //Serial.println("Init Complete");  // Debug Code
  Servo_Move(&testServo, 200);
  delay(1000);
}

void loop() 
{
  Servo_Move_Segmented(&testServo, 110,7);
  //Serial.print("Move 1 to ");
  //Serial.println(testServo.zServoPWMCurrent);   // Debug Code
  delay(1000);
  Servo_Move_Segmented(&testServo, 200,20);
  //Serial.print("Move 2 to ");
  //Serial.println(testServo.zServoPWMCurrent);   // Debug Code
  delay(1000);
}


/*
----------------------------------
---Function Initializations-------
----------------------------------
*/

void Servo_init(ServoType *Serv)
{
  pinMode(Serv->zServoPin,OUTPUT);
}

void Servo_Move(ServoType* Serv, unsigned int desiredPWM)
{
  unsigned int newPWM;
  /* Limits PWM Values between the 'open' and 'closed' values*/
  if(desiredPWM >= Serv->zServoPWMOpen)
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
  }

  /* Creates new current value and writes it to the pin */
  Serv->zServoPWMCurrent = newPWM;
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
   //Serial.println(Serv->zServoPWMCurrent - (segDiff)); // Debug Code

    Servo_Move(Serv,Serv->zServoPWMCurrent - (segDiff));

    delay(90);
  }
}
