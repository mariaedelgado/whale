// Author: Maria E. Delgado
// Credits:
//   Sung Jik Cha https://github.com/sungjik/my_personal_robotic_companion
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

// --------------------------------------------------------------//
// -------------------------- DEFINITIONS -----------------------//

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>

#define USE_USBCON

#define pinJoyX       A0
#define pinJoyY       A1

#define pinFORWARD    13
#define pinREVERSE    12
#define pinLEFT       11
#define pinRIGHT      10

#define LOOPTIME      100

// Timing
unsigned long time_since_last_command;
unsigned long lastMilli;

// Velocities (actual and required) for each wheel
#define VELOCITY_FORWARD        1.6
#define ACCELERATION_FORWARD    1.35

#define VELOCITY_REVERSE        1.35
#define ACCELERATION_REVERSE    1.35

#define VELOCITY_TURN           2.5
#define ACCELERATION_TURN       1.4

double vel_actual_right;
double vel_actual_left;

// Commands
int command;
bool is_new_command;
bool is_wheelchair_on;

int joystick_x;
int joystick_y;

int prev_joystick_x;
int prev_joystick_y;

// --------------------------------------------------------------//
// --------------------------- FUNCTIONS ------------------------//
ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped vel_msg;
ros::Publisher vel_publisher("vel", &vel_msg);
ros::Time current_time;
ros::Time last_time;

void publish_velocity(unsigned long time) {
  vel_msg.vector.x = vel_actual_right;
  vel_msg.vector.y = vel_actual_left;
  vel_msg.vector.z = double(time)/1000;
  vel_msg.header.stamp = nh.now();
  Serial.print(" ** Vel right: ");
  Serial.print(vel_actual_right);
  Serial.print(" / Vel left: ");
  Serial.print(vel_actual_left);
  Serial.print(" **");
  vel_publisher.publish(&vel_msg);
  nh.spinOnce();
}

int get_actual_directions(int joystick_x, int joystick_y) {
  if( (joystick_x < 5) && (joystick_y > 502) ) { // if FORDWARD, return 1
    return(1);
  } 
  else if( (joystick_x > 1020) && (joystick_y > 502) ) { // if REVERSE, return 2
    return(2);
  }
  else if( (joystick_x > 502) && (joystick_y > 1020) ) { // if LEFT, return 3
    return(3);
  }
  else if( (joystick_x > 502) && (joystick_y < 5) ) { // if RIGHT, return 4
    return(4);
  }
  else {
    return(0);
  }
}

void get_actual_velocity(unsigned long time_since_last_command, int command) {  // HACER
  double acceleration;
  double velocity;
  if( command == 1) { // FORWARD
    acceleration = ACCELERATION_FORWARD;
    velocity = VELOCITY_FORWARD;
  } else if( command == 2 ) { // REVERSE
    acceleration = - ACCELERATION_REVERSE;
    velocity = - VELOCITY_REVERSE;
  } else if( command == 3 || command == 4) { // LEFT OR RIGHT
    acceleration = ACCELERATION_TURN;
    velocity = VELOCITY_TURN;
  } else {
    acceleration = 0;
    velocity = 0;
  }

  // Set speed taking into account acceleration curve
  float current_velocity;
  if( abs(acceleration*time_since_last_command) >= abs(velocity) ) { // already reached constant velocity
    current_velocity = velocity;
  } else {                                                // still on acceleration curve
    current_velocity = acceleration*time_since_last_command;
  }

  if(command == 0 || command == 1 || command == 2){
      vel_actual_right = current_velocity;
      vel_actual_left = vel_actual_right;
  } else if( command == 3){
    vel_actual_right = current_velocity;
    vel_actual_left = - vel_actual_right;
  } else if( command == 4){
    vel_actual_left = current_velocity;
    vel_actual_right = - vel_actual_left;
  }
}

void send_wheelchair_command(int command) {
  switch(command) { // LOGICA INVERSA RELÉ
    case 0:
      digitalWrite(pinFORWARD, HIGH);
      digitalWrite(pinREVERSE, HIGH);
      digitalWrite(pinLEFT, HIGH);
      digitalWrite(pinRIGHT, HIGH);
      break;

    case 1:
      digitalWrite(pinREVERSE, HIGH);
      digitalWrite(pinLEFT, HIGH);
      digitalWrite(pinRIGHT, HIGH);
      digitalWrite(pinFORWARD, LOW);
      break;

    case 2:
      digitalWrite(pinFORWARD, HIGH);
      digitalWrite(pinLEFT, HIGH);
      digitalWrite(pinRIGHT, HIGH);
      digitalWrite(pinREVERSE, LOW);
      break;

    case 3:
      digitalWrite(pinFORWARD, HIGH);
      digitalWrite(pinREVERSE, HIGH);
      digitalWrite(pinRIGHT, HIGH);
      digitalWrite(pinLEFT, LOW);
      break;

    case 4:
      digitalWrite(pinFORWARD, HIGH);
      digitalWrite(pinREVERSE, HIGH);
      digitalWrite(pinLEFT, HIGH);
      digitalWrite(pinRIGHT, LOW);
      break;
      
  }
}

/* -------------- */ 
/* Main functions */

void setup() {
  // Init variables
  time_since_last_command = 0;
  
  vel_actual_right = 0;
  vel_actual_left = 0;

  joystick_x = 0;
  joystick_y = 0;
  prev_joystick_x = 0;
  prev_joystick_y = 0;

  command = 0;
  is_new_command = true;

  // Init Node Handle
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(vel_publisher);

  // Set Arduino pins
  pinMode(pinJoyX, INPUT);
  pinMode(pinJoyY, INPUT);

  pinMode(pinFORWARD, OUTPUT);
  pinMode(pinREVERSE, OUTPUT);
  pinMode(pinLEFT, OUTPUT);
  pinMode(pinRIGHT, OUTPUT);
  
  Serial.begin(57600);

}

void loop() {
  unsigned long time = millis();

  if(time - lastMilli >= LOOPTIME) {
    // If wheelchair ON, read x, y data from joystick and convert into directions
    joystick_x = analogRead(pinJoyX);
    delay(100);
    joystick_y = analogRead(pinJoyY);

    if( (abs(joystick_x - prev_joystick_x) > 20 ) || (abs(joystick_y - prev_joystick_y) > 20) ) {
      command = get_actual_directions(joystick_x, joystick_y);
      is_new_command = true;
    }
    
    prev_joystick_x = joystick_x;
    prev_joystick_y = joystick_y;

    // Send command to the chair
    send_wheelchair_command(command);
    
    // Get velocity from the commands and publish the actual velocity of the wheels
    if( is_new_command ){
      time_since_last_command = 0;
      is_new_command = false;
    }
    else
      time_since_last_command = time_since_last_command + LOOPTIME;

    get_actual_velocity(time_since_last_command, command);
    publish_velocity(time - lastMilli);
    lastMilli = time;
  }

}
