/** author: Maria E. Delgado
 ** credits : ros turtlebot node : https://github.com/Arkapravo/turtlebot
              arduino ros bridge : http://wiki.ros.org/ros_arduino_bridge
              Sung Jik Cha https://github.com/sungjik/my_personal_robotic_companion
**/

/* ------------------------------------------------------------------------------- */
/* -------------------------------- DEFINITIONS  --------------------------------- */

/* ROS includes */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <robot_specs.h>

/* Variable definitions */
// Velocity
double vel_actual_right = 0.0;
double vel_actual_left = 0.0;
double vel_required_right = 0.0;
double vel_required_left = 0.0;
double vel_dt = 0.0;

// Position
double x_pos = 0.0;
double y_pos = 0.0;
double yaw = 0.0;

// Other
ros::Time current_time;
ros::Time vel_time(0.0);
ros::Time last_time(0.0);

/* ------------------------------------------------------------------------------- */
/* --------------------------------- FUNCTIONS  ---------------------------------- */

/* Handle messages */
void handle_vel(const geometry_msgs::Vector3Stamped% vel) {
  vel_actual_right = vel.vector.x;
  vel_actual_left = vel.vector.y;
  vel_dt = vel.vector.z;
  vel_time = vel.header.stamp;
}

void set_covariance(double vel_actual_right, double vel_actual_left) {
  if( vel_actual_right==0 && vel_actual_left==0 ) {
    odom_msg.pose.covariance[0] = 1e-9;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[8] = 1e-9;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;
    odom_msg.pose.covariance[35] = 1e-9;
    odom_msg.twist.covariance[0] = 1e-9;
    odom_msg.twist.covariance[7] = 1e-3;
    odom_msg.twist.covariance[8] = 1e-9;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 1e-9;
  }
  else {
    odom_msg.pose.covariance[0] = 1e-3;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[8] = 0.0;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;
    odom_msg.pose.covariance[35] = 1e3;
    odom_msg.twist.covariance[0] = 1e-3;
    odom_msg.twist.covariance[7] = 1e-3;
    odom_msg.twist.covariance[8] = 0.0;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 1e3;
  }
}

/* Main */

int main(int argc, char** argv) {
  // ROS init topics for subscription and publishing
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private_("~");

  ros::Subscriber sub = nh.subscribe("vel", 50, handle_vel);
  ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  // Variable initialization
  // Position
  double rate = 10.0;                     // Publish rate for odom and tf

  double linear_scale_positive = 1.0;     // Amount to scale translational velocity in positive x direction 
  double linear_scale_negative = 1.0;     // Amount to scale translational velocity in negative x direction
  double angular_scale_positive = 1.0;    // Amount to scale rotational velocity counterclockwise
  double angular_scale_negative = 1.0;    // Amount to scale rotational velocity clockwise
  double angular_scale_accel = 1.0;

  double acc_yaw = 0.0;
  double acc_x = 0.0;
  double acc_max_yaw = 0.0;
  double acc_max_x = 0.0;

  // Differentials
  double dt = 0.0;
  double vel_translational = 0.0;
  double vel_angular = 0.0;

  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;

  double dyaw_odom = 0.0;
  double dyaw = 0.0;
  double dxy_ave = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  // Publisher chars
  char base_link[] = "/base_link";
  char odom[] = "/odom";

  ros::Duration d(1.0);

  // Get parameters
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);

  ros::Rate r(rate);

  // Node handle receiving info
  while(nh.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();

    // Compute x, y, yaw from actual velocities. Source: https://core.ac.uk/download/pdf/81937972.pdf
    vel_translational = (vel_actual_right + vel_actual_left)/2;
    vel_angular = (vel_actual_right - vel_actual_left)/track_width;
    
    dtheta = vel_angular;
    dx = cos(theta)*vel_translational;
    dy = sin(theta)*vel_translational;

    x_pos += (dx*cos(dtheta) - dy*sin(dtheta)) *dt;
    y_pos += (dx*sin(dtheta) + dy*cos(dtheta)) *dt;
    theta += dtheta*dt;
    
    /* Publish messages */
    // TF topic
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    geometry_msgs::TransformStamped t;
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    t.header.stamp = current_time;
    broadcaster.sendTransform(t);

    // Odometry topic
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.positiom.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    set_covariance(vel_actual_right, vel_actual_left);
    vx = (dt == 0)? 0 : vel_translational/dt;
    vth = (dt == 0)? 0 : dtheta/dt;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dyaw;
    odometry_publisher.publish(odom_msg);

    last_time = current_time;
    r.sleep();  
  }
}
