// Modified from:
// Author: Sung Jik Cha
// Credits:
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>


/*
Connections:
Motor 1
- Pin 5 ---> PWM1
- Pin 7 ---> A1
- Pin 8 ---> B1

Motor 2
- Pin 6 ---> PWM2
- Pin 4 ---> A2
- Pin 9 ---> B2

- Motor 1 (Left) : A01 and A02
- Motor 2 (Right): B01 and B02
*/

#define LOOPTIME        100 

//Constants to help remember the parameters
#define LEFT_MOTOR   0 //for motorDrive, motorStop, motorBrake functions
#define RIGHT_MOTOR  1 //for motorDrive, motorStop, motorBrake functions

#define MOTOR_CW     0 //for motorDrive function
#define MOTOR_CCW    1 //for motorDrive function

//Define the Pins
//Motor 1 (Left)
#define pinAIN1   7 //Direction
#define pinAIN2   8 //Direction
#define pinPWMA   5 //Speed (Connect to Timer1)

//Motor 2 (Right)
#define pinBIN1  4 //Direction
#define pinBIN2  9 //Direction
#define pinPWMB  6 //Speed (Connect to Timer1)

#define EN_PIN_1 A0
#define EN_PIN_2 A1


//#define sign(x) (x > 0) - (x < 0)

unsigned long lastMilli = 0;       // loop timing 

int direction1 = MOTOR_CW;
int direction2 = MOTOR_CW;

//int prev_direction1 = RELEASE;
//int prev_direction2 = RELEASE;

int PWM_val1 = 0;
int PWM_val2 = 0;


void motorDrive(boolean motorNumber, boolean motorDirection){
  /*
  This Drives a specified motor, in a specific direction, at a specified speed:
    - motorNumber: LEFT_MOTOR or RIGHT_MOTOR ---> Motor 1 or Motor 2
    - motorDirection: MOTOR_CW or MOTOR_CCW ---> clockwise or counter-clockwise
  */

  boolean pinIn1;  //Relates to AIN1 or BIN1 (depending on the motor number specified)
 
  //Specify the Direction to turn the motor
  //Clockwise: AIN1/BIN1 = HIGH and AIN2/BIN2 = LOW
  //Counter-Clockwise: AIN1/BIN1 = LOW and AIN2/BIN2 = HIGH
  if (motorDirection == MOTOR_CW)
    pinIn1 = HIGH;
  else
    pinIn1 = LOW;

  //Select the motor to turn, and set the direction and the speed
  if(motorNumber == LEFT_MOTOR){
    digitalWrite(pinAIN1, pinIn1);
    digitalWrite(pinAIN2, !pinIn1);  //This is the opposite of the AIN1
  }
  else{
    digitalWrite(pinBIN1, pinIn1);
    digitalWrite(pinBIN2, !pinIn1);  //This is the opposite of the BIN1
  }

}

void motorSpeed(boolean motorNumber, int motorSpeed){
  /*
    - motorSpeed: 0 to 255 ---> 0 = stop / 255 = fast
  */

  //Select the motor to turn, and set the direction and the speed
  if(motorNumber == LEFT_MOTOR){
    analogWrite(pinPWMA, motorSpeed);
  }
  else{
    analogWrite(pinPWMB, motorSpeed);
  }

}

void motorBrake(boolean motorNumber){
  //This "Short Brake"s the specified motor, by setting speed to zero
  if (motorNumber == LEFT_MOTOR)
    analogWrite(pinPWMA, 0);
  else
    analogWrite(pinPWMB, 0);
}

void motorStop(boolean motorNumber){
  //This stops the specified motor by setting both IN pins to LOW
  if (motorNumber == LEFT_MOTOR){
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
  }
  else{
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
  } 
}

void testMotor(int dir1, int pwm1, int dir2, int pwm2){
  //tune PID
  //int val = analogRead(3);    // read the input pin
  //float volt = (float)val*(3.3/1023.);
  //Kp = volt * (30.); Ki = 0.; Kd = 0;
  

  //motorSpeed(LEFT_MOTOR, 64);
  //motorSpeed(RIGHT_MOTOR, 64);
  //motorDrive(LEFT_MOTOR, MOTOR_CW);
  //motorDrive(RIGHT_MOTOR, MOTOR_CW);

  motorDrive(LEFT_MOTOR, dir1);
  motorDrive(RIGHT_MOTOR, dir2);
  
  motorSpeed(LEFT_MOTOR, pwm1);
  motorSpeed(RIGHT_MOTOR, pwm2);

  /*
  delay(3000);
  
  motorStop(LEFT_MOTOR);
  motorStop(RIGHT_MOTOR);
  
  motorDrive(LEFT_MOTOR, MOTOR_CCW);
  motorDrive(RIGHT_MOTOR, MOTOR_CCW);
  
  delay(3000);
  
  motorStop(LEFT_MOTOR);
  motorStop(RIGHT_MOTOR);
  */
 
}

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  
  if (z == 0) {     // go straight
    // convert m/s to rpm
    //rpm_req1 = x*60/(pi*wheel_diameter);
    //rpm_req2 = rpm_req1;
    PWM_val1 = x*128;
    PWM_val2 = PWM_val1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    //rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    //rpm_req1 = -rpm_req2;
    PWM_val2 = z*128;
    PWM_val1 = -PWM_val2;
  }
  else {
    //rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    //rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);


void setup() {

 PWM_val1 = 0;
 PWM_val2 = 0;

 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 
  //Set the PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH); 
 
 motorSpeed(LEFT_MOTOR, 0);
 motorSpeed(RIGHT_MOTOR, 0);
 
 motorDrive(LEFT_MOTOR, MOTOR_CW);
 motorDrive(RIGHT_MOTOR, MOTOR_CW);
 
 //motorStop(LEFT_MOTOR);
 //motorStop(RIGHT_MOTOR);

 //test
 //Serial.begin(57600);
 
}

void loop() {
  nh.spinOnce();
  
  unsigned long time = millis();
  
  if(time-lastMilli >= LOOPTIME)   {      // enter tmed loop

    if(PWM_val1 > 0) direction1 = MOTOR_CW;
    else if(PWM_val1 < 0) direction1 = MOTOR_CCW;
        
    if(PWM_val2 > 0) direction2 = MOTOR_CW;
    else if(PWM_val2 < 0) direction2 = MOTOR_CCW;
    
    //testMotor(direction1, abs(PWM_val1), direction2, abs(PWM_val2));
    
    if (PWM_val1 == 0)
      motorStop(LEFT_MOTOR);
    else{
       motorDrive(LEFT_MOTOR, direction1);
       delay(100);
       motorStop(LEFT_MOTOR);
       PWM_val1 == 0;
    }

    if (PWM_val2 == 0)
      motorStop(RIGHT_MOTOR);
    else{
       motorDrive(RIGHT_MOTOR, direction2);
       delay(100);
       motorStop(RIGHT_MOTOR);
       PWM_val2 == 0;
    }

    motorSpeed(LEFT_MOTOR, abs(PWM_val1));
    motorSpeed(RIGHT_MOTOR, abs(PWM_val2));
    
    //publishRPM(time-lastMilli);
    lastMilli = time;
  }
  
}
