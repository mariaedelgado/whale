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

/* Arduino PINS */
#define encodPinONOFF      1
#define encodPinFORWARD    2     
#define encodPinREVERSE    3
#define encodPinTURNLEFT   4
#define encodPinTURNRIGHT  5

#define LOOPTIME           100
#define sign(x) (x > 0) - (x < 0)

/* Variable initialization */
ros::NodeHandle nh;                // Handles publishing and subscribing to topics

// Timing
unsigned long lastMilli = 0;
unsigned long lastMilliPub = 0;

// Velocities (actual and required) for each wheel
double vel_actual_right = 0;
double vel_actual_left = 0;
double vel_required_right = 0;
double vel_required_left = 0;

// --------------------------------------------------------------//
// --------------------------- FUNCTIONS ------------------------//

/* ROS message handling */

void handle_cmd(const geometry_msgs::Twist& cmd_msg) { // FIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;

  if(z==0) {        // no angular info: wheelchair goes straight. Convert from m/s to rpm
    vel_required_right = x*(60/(2*pi*wheel_diameter));
    vel_required_left = vel_required_right;
  }
  else if(x == 0) { // no linear info: wheelchair is changing of angle. Convert rad/s to rpm
    vel_required_right = z*track_width*(60/(2*pi*wheel_diameter));
    vel_required_left = -vel_required_left;
  }
  else {            // both linear and angular info
    vel_required_right = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    vel_required_left = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped vel_msg;
ros::Publisher vel_publisher("vel", &vel_msg);
ros::Time current_time;
ros::Time last_time;

void publish_velocity(unsigned long time) {
  vel_msg.vector.x = vel_actual_right;
  vel_msg.vector.y = vel_actual_left;
  vel_msg.vector.z = double(time)/1000;
  vel_msg.header.stamp = ng.now();

  vel_publisher.publish(&vel_msg);
  nh.spinOnce;
}

/* -------------- */ 
/* Main functions */

void setup() {
  // Init variables
  vel_actual_right = 0;
  vel_actual_left = 0;
  vel_required_right = 0;
  vel_required_left = 0;

  // Init Node Handle
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);                  // Subscribes to geometry_msgs::Twist topic
  nh.advertise(vel_pub);

  // Set Arduino pins
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  digitalWrite(encodPinA1, HIGH);
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, encoder1, RISING);

}

void loop() {
  nh.spinOnce();                  // Asks ROS to process the upcoming messages

  unsigned long time = millis();  // Time since machine started
  if(time - lastMilli >= LOOPTIME) {
    // Set new command to wheels:
    // Forward, reverse or stop
    if( (vel_required_right == vel_required_left) && (vel_required_right > 0) ) digitalWrite(encodPinFORWARD, HIGH);
    else if( (vel_required_right == vel_required_left) && (vel_required_right < 0) ) digitalWrite(encodPinREVERSE, HIGH);
    else if( (vel_required_right == vel_required_left) && (vel_required_right == 0) ) digitalWrite(encodPinONOFF, LOW);
    // Turn
    else if( (vel_required_right != vel_required_left) && (vel_required_right < vel_required_left) ) digitalWrite(encodPinRIGHT, HIGH);
    else if( (vel_required_right != vel_required_left) && (vel_required_right > vel_required_left) ) digitalWrite(encodPinLEFT, HIGH);

    // Publish the actual velocity of the wheels
    vel_actual_right = vel_required_right;
    vel_actual_left = vel_required_left;
    publish_velocity(time - lastMilli);
    lastMilli = time;
  }

  if(time - lastMilliPub >= LOOPTIME) {
    lastMilliPub = time;
  }
}
