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
#define pinJoyX            6
#define pinJoyY            7
#define pinJoyButton       8

#define LOOPTIME           1000

/* Variable initialization */
ros::NodeHandle nh;                // Handles publishing and subscribing to topics

// Timing
unsigned long lastMilli = 0;
unsigned long lastMilliPub = 0;

// Velocities (actual and required) for each wheel
#define VELOCITY_FORWARD 1.6  // m/s
#define ACCELERATION_FORWARD    1.35 // m/ss

#define VELOCITY_REVERSE 1.35
#define ACCELERATION_REVERSE    1.35

double vel_actual_right = 0;
double vel_actual_left = 0;

// Commands
int command;
bool is_new_command = true;

int joystick_x = 0;
int joystick_y = 0;
bool joystick_button = false;

int prev_joystick_x = 0;
int prev_joystick_y = 0;
bool prev_joystick_button = false;

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

int get_actual_directions(int joystick_x, int joystick_y) {
  if( (joystick_x == 0) && (joystick_y > 0) ) { // if FORDWARD, return 1
    digitalWrite(encodPinFORWARD, HIGH);
    digitalWrite(encodPinREVERSE, LOW);
    return(1);
  } 
  else if( (joystick_x == 0) && (joystick_y < 0) ) { // if REVERSE, return -1
    digitalWrite(encodPinREVERSE, HIGH);
    digitalWrite(encodPinFORWARD, LOW);
    return(-1);
  }
}

void get_actual_velocity(unsigned long time_since_last_command, int command) {  // HACER
  if( command == 1) { // FORWARD
    int acceleration = ACCELERATION_FORWARD;
    int velocity = VELOCITY_FORWARD;
  else if( command == -1 ) { // REVERSE
    int acceleration = ACCELERATION_REVERSE;
    int velocity = VELOCITY_REVERSE;
  }

  // Set speed taking into account acceleration curve
  if( acceleration*time_since_last_command >= velocity) { // already reached constant velocity
    vel_actual_right = velocity;
    vel_actual_left = vel_actual_right;
  } else {                                                // still on acceleration curve
    vel_actual_right = acceleration*time_since_last_command;
    vel_actual_left = vel_actual_right;
  }
    
}

/* -------------- */ 
/* Main functions */

void setup() {
  // Init variables
  vel_actual_right = 0;
  vel_actual_left = 0;

  joystick_x = 0;
  joystick_y = 0;
  joystick_button = false;
  prev_joystick_x = 0;
  prev_joystick_y = 0;
  prev_joystick_button = false;

  command = 0;
  is_new_command = true;

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
  pinMode(pinJoyX, INPUT);
  pinMode(pinJoyY, INPUT);
  pinMode(pinJoyButton, INPUT_PULLUP);
  Serial.begin(9600);

}

void loop() {
  unsigned long time = millis();

  if(time - lastMilli >= LOOPTIME) {
    joystick_button = digitalRead(pinJoyButton);

    // If new ONOFF command, change to ON or OFF
    if(joystick_button != prev_joystick_button) {
      if(joystick_button == true) // ON
        digitalWrite(encodPinONOFF, HIGH);
      else // OFF
        digitalWrite(encodPinONOFF, LOW);

      prev_joystick_button = joystick_button;
    }

    // Read x, y data from joystick and convert into directions
    joystick_x = analogRead(pinJoyX);
    delay(100);
    joystick_y = analogRead(pinJoyY);

    if( (joystick_x != prev_joystick_x) || (joystick_y != prev_joystick_y) ) {
      command = get_actual_directions(joystick_x, joystick_y);
      is_new_command = true;
    }

    Serial.print("Actual command: ");
    Serial.print(command);

    prev_joystick_x = joystick_x;
    prev_joystick_y = joystick_y;

    // Get velocity from the commands and publish the actual velocity of the wheels
    if( is_new_command )
      time_since_last_command = 0;
      is_new_command = false;
    else
      time_since_last_command = time_since_last_command + LOOPTIME;

    get_actual_velocity(time_since_last_command, command);
    publish_velocity(time - lastMilli);
    lastMilli = time;
  }

}
