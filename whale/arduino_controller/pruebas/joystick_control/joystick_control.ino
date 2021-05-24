/* 
 * rosserial joystick control Publisher
 */

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>

#define pinJoyX       A0
#define pinJoyY       A1
#define pinJoyButton  10 

#define LOOPTIME      1000

unsigned long lastMilli;

int joystick_x;
int joystick_y;
bool joystick_button;

int prev_joystick_x;
int prev_joystick_y;
bool prev_joystick_button;

ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped joy_msg;
ros::Publisher joy_publisher("joy", &joy_msg);
ros::Time current_time;
ros::Time last_time;

void publish_joystick_commands(unsigned long time) {
  joy_msg.vector.x = joystick_x;
  joy_msg.vector.y = joystick_y;
  joy_msg.vector.z = joystick_button;
  joy_msg.header.stamp = nh.now();
  Serial.print("Publish");
  joy_publisher.publish(&joy_msg);
  nh.spinOnce();
}

void setup()
{
  /* Variable initialization */
  lastMilli = 0;
  joystick_x = 0;
  joystick_y = 0;
  joystick_button = false;
  prev_joystick_x = 0;
  prev_joystick_y = 0;
  prev_joystick_button = false;
  
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(joy_publisher); 
  
  // Set Arduino pins
  pinMode(pinJoyX, INPUT);
  pinMode(pinJoyY, INPUT);
  pinMode(pinJoyButton, INPUT_PULLUP);
  Serial.begin(57600);
}

void loop()
{  
  unsigned long time = millis();

  if(time - lastMilli >= LOOPTIME) {
    joystick_button = digitalRead(pinJoyButton);
    joystick_x = analogRead(pinJoyX);
    delay(100);
    joystick_y = analogRead(pinJoyY);

    publish_joystick_commands(time - lastMilli);
    lastMilli = time;
  }
}
