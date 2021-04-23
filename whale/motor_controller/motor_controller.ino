// Author: Maria E. Delgado
// Credits:
//   Sung Jik Cha https://github.com/sungjik/my_personal_robotic_companion
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

// --------------------------------------------------------------//
// -------------------------- DEFINITIONS -----------------------//

/* ROS */
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

/* Servomotors */ // !!!!!!!! change depending on servo
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

/* Arduino PINS */ // !!!!!!!! change depending on servo
#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)

/* Variable initialization */
// Timing
unsigned long lastMilli = 0;
unsigned long lastMilliPub = 0;

// RPM
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;

// Directions
int direction1 = FORWARD;
int direction2 = FORWARD;
int prev_direction1 = RELEASE;
int prev_direction2 = RELEASE;
int PWM_val1 = 0;
int PWM_val2 = 0;

// PID control
float Kp =   0.5;
float Kd =   0;
float Ki =   0;

// Other
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
ros::NodeHandle nh;                // Handles publishing and subscribing to topics

// --------------------------------------------------------------//
// --------------------------- FUNCTIONS ------------------------//

/* ROS message handling */

void handle_cmd(const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;

  if(z==0) {        // no angular info: wheelchair goes straight. Convert from m/s to rpm
    rpm_req1 = x*(60/(2*pi*wheel_diameter));
    rpm_req2 = rpm_req1;
  }
  else if(x == 0) { // no linear info: wheelchair is changing of angle. Convert rad/s to rpm
    rpm_req1 = z*track_width*(60/(2*pi*wheel_diameter));
    rpm_req2 = -rpm_req2;
  }
  else {            // both linear and angular info
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = ng.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce
}

/* -------------------- */
/* Motor data functions */

void getMotorData(unsigned long time) {
  rpm_act1 = double( (count1 - countAnt1)*60*1000 )/double( (time*encoder_pulse*gear_ratio) );
  rpm_act2 = double( (count2 - countAnt2)*60*1000 )/double( (time*encoder_pulse*gear_ratio) );
  
  countAnt1 = count1; // revolutions counter
  countAnt2 = count2;
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}

void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}

/* --------------------- */
/* PID control functions */

int updatePid(int id, int command, double targetValue, double currentValue) {
  // Initialization of variables
  double pidTerm = 0;
  double error = 0;
  double new_pwn = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  error = targetValue - currentValue;

  // If servo 1 or 2
  if(id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error - last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error - last_error2) + Ki*int_error2;
    last_error2 = error;
  }

  new_pwm = constrain( double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM );
  NEW_CMD = 4095.0*new_pwm/MAX_RPM;

  return int(new_cmd);
} 

/* -------------- */ 
/* Main functions */

void setup() {
  // AFMS.begin();  // inits servomotor

  // Init variables
  count1 = 0;
  count2 = 0;
  countAnt1 = 0;
  countAnt2 = 0;
  rpm_req1 = 0;
  rpm_req2 = 0;
  rpm_act1 = 0;
  rpm_act2 = 0;
  PWM_val1 = 0;
  PWM_val2 = 0;

  // Init Node Handle
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);                  // Subscribes to geometry_msgs::Twist topic
  nh.advertise(rpm_pub);

  // Set Arduino pins
  // Servo 1
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  digitalWrite(encodPinA1, HIGH);
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, encoder1, RISING);
  //Servo 2
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(0, encoder2, RISING);

  // Init motors
  // !!!!!!!!!!!!!!!!!!!!!!!!!!! falta
}

void loop() {
  nh.spinOnce();                  // Asks ROS to process the upcoming messages

  unsigned long time = millis();  // Time since machine started
  if(time - lastMilli >= LOOPTIME) {
    // If 100 s have passed -> new loop iteration
    // Get data from motors -> revolutions number
    getMotorData(time - lastMilli);
    
    // PID control
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if(PWM_val1 > 0) direction1 = FORWARD;
    else if(PWM_val1 < 0) direction1 = BACKWARD;
    if(rpm_req1 == 0) direction = RELEASE;
    
    if(PWM_val2 > 0) direction2 = FORWARD;
    else if(PWM_val2 < 0) direction2 = BACKWARD;
    if(rpm_re12 ==) direction2 = RELEASE;

    // Send commands to the wheels servos
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! run + set speed

    // Publish the actual RPM of the wheels
    publishRPM(time - lastMilli);
    lastMilli = time;
  }

  if(time - lastMilliPub >= LOOPTIME) {
    lastMilliPub = time;
  }
}
