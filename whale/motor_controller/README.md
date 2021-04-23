# README for whale/motor_controller
The main objective of motor_controller.ino is to control the Arduino board in order to:
- Convert the motor encoder tick counts into rpm.
- Control the speed of the motors using PID control.

## Functions

### Main functions
##### void setup()
The setup function initializes the proper functions and variables to start working. Specifically:
- Variable initialization to 0 of the tick counters (count1, count2, countAnt1, countAnt2), the speeds (both rpm_req1, rpm_req2 and rpm_act1, rpm_act2) and the pulse width modulation (PWM_val1, PWM_val2).
- Initialization of ROS Node Handler.
- Sets Arduino pins.
- Inits servomotors.

##### void loop()

The main loop of the code is responsible for the two tasks of the Arduino: PID control and rpm publishing. It loops at 10 Hz.
The specific tasks that the function does are:
- Receive ROS messages of type <geometry_msgs::Twist>, cmd_msg, where it obtains rpm_req1 and rpm_req2, the required rpm that navigation sends to Arduino.
- Get the actual data from the motors -> it obtains rpm_act1, rpm_act2.
- Update the PID control with the pulse width modulation and both rpm_reqx and rpm_actx.
- With the updated PWM (1 and 2), we can assess the commands to pass to the wheelchair (go forward, backwards or stop).
- Lastly, it publishes the rpm topic of the actual speed of the motors.

### Message publisher and subscriber functions
#### void handle_cmd(const geometry_msgs::Twist& cmd_msg)

The command handler function reveiver ROS geometry messages, specifically Twist messages, which it converts into the required rpm for each motor. The format of the messages is a <geometry_msgs::Twist>.
The function performs the calculation of the rpm depending on:
- If there is no angular information (the wheelchair goes straight), it converts from m/s to rpm (!!!!!! 60/pi*radius or 2*pi).
- If there is no linear information (the wheelchair is turning), it converts from rad/s to rpm.
- If there is information of both, it computes the rpm in consequence (sum of linear and angular parameters).

#### void publishRPM(unsigned long time)

The RPM publisher function is responsible for constructing the ROS message to send the rpm_act information. It sends:
- In x, the rpm of servo 1.
- In y, the rpm of servo 2.
- In z, the time (in ms) between this event and the last event.

### Get data from motors
