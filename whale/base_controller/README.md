# README for whale/base_controller

The main objective of base_controller is to subscribe to the Arduino rpm topic (passed through rosserial) and convert the rpm to coordinates. These coordinates are going to be published in Odometry and tf messages.

All in all:
- Subscribed topic: rpm(geometry_msgs::Vector3Stamped)
- Published topic: odom(nav_msgs::Odometry)

## Functions
### int main(int argc, char** argv)

The main function carries all the tasks for getting the motor information, transforming it into odometry format and publishing it. Step by step, it:
- Creates a Node Handler for publishing Odometry messages and subscribes to the rpm topic.
- Converts rpm to x, y, rotation by


### void handle_rpm(const geometry_msgs::Vector3Stamped% rpm)

The rpm handler function subscribes to the rpm topic, sent by the base controller to obtain the current information about the motors. It basically gets:
- rpm_act1 and rpm_act2
- Time difference between this event and the last.

### void set_covariance(double rpm_act1, double rpm_act2) 

This function computes the covariance of the odometry pose.


