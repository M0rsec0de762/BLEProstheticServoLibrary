/*
----------------------------------
-----Includes-----
----------------------------------
*/
#include <Servo.h>
/*
----------------------------------
---Variable Defines-------
----------------------------------
*/
#define SERVO_MAX_TRAVEL 150
#define SERVO_PULSE_RANGE 1400
/*
/*
----------------------------------
-------------Type Definitions-----
----------------------------------
*/

/*Type definition for a structure regarding servos.*/
class ArmServo
{
  public:
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
    void servoInit();
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
    void servoMove(unsigned int desiredMicro);
    /*
      Function Name: servoMoveSegmented
      Input: 
        ServoType Structure (A Servo)
        Desired micro to move to
      Purpose: Move servo to the desiredMicro position using incremental micro values between the current angle and the desiredMicro
    */
    void servoMoveSegmented(unsigned int desiredMicro, unsigned int seg);
    /*
      Function Name: servoOpen
      Input: 
        ServoType Structure (A Servo)
      Purpose: Turn on MOSFET, open hand, turn off MOSFET
    */
    void servoOpen();
    /*
      Function Name: servoOpen
      Input: 
        ServoType Structure (A Servo)
      Purpose: Turn on MOSFET, close hand, leave MOSFET on
    */
    void servoClosed();


    /*Max's Math Code. Comment this stuff later*/
    // long constrain(long x, long in_min, long in_max) 
    // long map(long x, long in_min, long in_max, long out_min, long out_max) 

};

/*
----------------------------------
-------------Main Code------------
----------------------------------
*/
ArmServo testServo1;
ArmServo testServo2;

void setup() 
{
  /*Init Serial Line for Arduino debugging purposes*/
  Serial.begin(9600);
  /*Init servo 1 with only the micro pin, and open/closed pulse widths*/
  testServo1.z_servo_pin = 14;
  testServo1.z_servo_micro_open = 2500;
  testServo1.z_servo_micro_closed = 500;
  testServo1.z_servo_micro_max = 2500;
  testServo1.z_servo_micro_min = 500;
  testServo1.z_MOSFET_pin = 22;
  testServo1.z_servo_speed = 210;
  testServo1.servoInit();
  /*Init servo 2 with only the micro pin, and open/closed pulse widths*/
  testServo2.z_servo_pin = 5;
  testServo2.z_servo_micro_open = 2500;
  testServo2.z_servo_micro_closed = 500;
  testServo2.z_servo_micro_max = 2500;
  testServo2.z_servo_micro_min = 500;
  testServo2.z_MOSFET_pin = 23;
  testServo2.z_servo_speed = 210;
  testServo2.servoInit();
  Serial.println("Init Complete");  // Debug Code

}

void loop() 
{
  /* Debug Code Start*/
  Serial.print("Move Servo 1 to closed: ");
  Serial.println(testServo1.z_servo_micro_current);  
  /* Debug Code End */
  testServo1.servoClosed();
  delay(1000);
  /* Debug Code Start*/
  Serial.print("Move Servo 1 to open: ");
  Serial.println(testServo1.z_servo_micro_current);   
  /* Debug Code End */
  testServo1.servoOpen();
  delay(1000);
  /* Debug Code Start*/
  Serial.print("Move Servo 2 to closed: ");
  Serial.println(testServo2.z_servo_micro_current);  
  /* Debug Code End */
  testServo2.servoClosed();
  delay(1000);
  /* Debug Code Start*/
  Serial.print("Move Servo 2 to open: ");
  Serial.println(testServo2.z_servo_micro_current);   
  /* Debug Code End */
  testServo2.servoOpen();
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