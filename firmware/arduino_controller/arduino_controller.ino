#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
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

unsigned long lastMilli = 0;       // loop timing 
unsigned long last_cmd_time = 0;       // loop timing 

int PWM_val1 = 0;
int PWM_val2 = 0;

void control_motor(int speed, int pwmPin, int INaPin, int INbPin){
    if(speed > 0){
        analogWrite(pwmPin, speed);
        digitalWrite(INaPin, HIGH);
        digitalWrite(INbPin, LOW);
    }
    else if(speed < 0){
        analogWrite(pwmPin, -speed);
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, HIGH);
    }
    else{
        digitalWrite(INaPin, LOW);
        digitalWrite(INbPin, LOW);
    }
}

void handle_cmd(const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  
  if (z == 0) {     // go straight
    // convert m/s to rpm
    //rpm_req1 = x*60/(pi*wheel_diameter);
    //rpm_req2 = rpm_req1;
    PWM_val1 = x*500;
    PWM_val2 = PWM_val1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    //rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    //rpm_req1 = -rpm_req2;
    PWM_val1 = z*1500;
    PWM_val2 = -PWM_val1;
  }
  else {
    //rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    //rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
    PWM_val1 = x*500 + z*1500;
    PWM_val2 = x*500 - z*1500;
  }

  PWM_val1 = constrain(PWM_val1, -255, 255);
  PWM_val2 = constrain(PWM_val2, -255, 255);

  last_cmd_time = millis();
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);

void setup() {
  
  PWM_val1 = 0;
  PWM_val2 = 0;
 
  //Set the PIN Modes
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);

  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  //enable motors
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH); 
 
  control_motor(PWM_val1, pinPWMA, pinAIN1, pinAIN2);
  control_motor(PWM_val2, pinPWMB, pinBIN1, pinBIN2);

  //start ROS node
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);

  while(!nh.connected()){
    nh.spinOnce();
  }

  nh.loginfo("** Mobile base connected! **");
  delay(1);
}

void loop() {
  nh.spinOnce();
  
  unsigned long time = millis();
  
  if(time-lastMilli >= LOOPTIME){ // enter timed loop
    control_motor(PWM_val1, pinPWMA, pinAIN1, pinAIN2);
    control_motor(PWM_val2, pinPWMB, pinBIN1, pinBIN2);
    
    lastMilli = time;
  }

  if(time-last_cmd_time >= 250){
    PWM_val1 = 0;
    PWM_val2 = 0;
  }
  
}
