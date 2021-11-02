/*
----------------------------------
-----Includes-----
----------------------------------
*/
#include <EECS473Servo.h>

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
