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

/* Variable initialization */
ros::NodeHandle nh;                // Handles publishing and subscribing to topics

// Timing
unsigned long lastMilli = 0;
unsigned long lastMilliPub = 0;

// Velocities (actual and required) for each wheel
#define ACTUAL_VELOCITY_RIGHT 3
#define ACTUAL_VELOCITY_LEFT 3

double vel_actual_right = 0;
double vel_actual_left = 0;

// --------------------------------------------------------------//
// --------------------------- FUNCTIONS ------------------------//

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

void get_actual_velocity() {  // HACER
  vel_actual_right = ACTUAL_VELOCITY_RIGHT;
  vel_actual_left = ACTUAL_VELOCITY_LEFT;
}

/* -------------- */ 
/* Main functions */

void setup() {
  // Init variables
  vel_actual_right = 0;
  vel_actual_left = 0;

  // Init Node Handle
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(vel_publisher);

  // Set Arduino pins
  pinMode(encodPinONOFF, OUTPUT);
  pinMode(encodPinFORWARD, OUTPUT);
  pinMode(encodPinREVERSE, OUTPUT);
  pinMode(encodPinTURNLEFT, OUTPUT);
  pinMode(encodPinTURNRIGHT, OUTPUT);
}

void loop() {
  unsigned long time = millis();
  if(time - lastMilli >= LOOPTIME) {
    // Publish the actual velocity of the wheels
    get_actual_velocity();
    publish_velocity(time - lastMilli);
    lastMilli = time;
  }

  // if(time - lastMilliPub >= LOOPTIME) {
  //   lastMilliPub = time;
  // }
}
