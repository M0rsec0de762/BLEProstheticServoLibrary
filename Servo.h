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

/*
  Function Name: Servo_Invert_At_Speed
  Input: 
    ServoType Structure (A Servo)
    Speed will be moving fast or slow (1 or 0)
    
  Purpose: Invert the position of the servo (close to open and vice-versa) at an arbitrary fast or slow speed*/
  
void servoInvertAtSpeed(ServoType *Serv,  unsigned int isFast);