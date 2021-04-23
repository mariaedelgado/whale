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
// RPM
double rpm_act1 = 0.0;
double rpm_act2 = 0.0;
double rpm_req1 = 0.0;
double rpm_req2 = 0.0;
double rpm_dt = 0.0;

// Position
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

// Other
ros::Time current_time;
ros::Time rpm_time(0.0);
ros::Time last_time(0.0);

/* ------------------------------------------------------------------------------- */
/* --------------------------------- FUNCTIONS  ---------------------------------- */

/* Handle messages */

void handle_rpm(const geometry_msgs::Vector3Stamped% rpm) {
  rpm_act1 = rpm.vector.x;
  rpm_act2 = rpm.vector.y;
  rpm_dt = rpm.vector.z;
  rpm_time = rpm.header.stamp;
}

void set_covariance(double rpm_act1, double rpm_act2) {
  if( rpm_act1==0 && rpm_act2==0 ) {
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

  ros::Subscriber sub = nh.subscribe("rpm", 50, handle_rpm);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  // Variable initialization !!!!!!!!!!1 delete gyro vars
  // Position
  double rate = 10.0;                     // Publish rate for odom and tf
  double linear_scale_positive = 1.0;     // Amount to scale translational velocity in positive x direction 
  double linear_scale_negative = 1.0;     // Amount to scale translational velocity in negative x direction
  double angular_scale_positive = 1.0;    // Amount to scale rotational velocity counterclockwise
  double angular_scale_negative = 1.0;    // Amount to scale rotational velocity clockwise
  double angular_scale_accel = 1.0;
  double acc_theta = 0.0;
  double acc_x = 0.0;
  double acc_max_theta = 0.0;
  double acc_max_x = 0.0;
  bool publish_tf = true;

  // Differentials
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth_odom = 0.0;
  double dth_gyro = 0.0;
  double dth = 0.0;
  double dth_prev = 0.0;
  double dth_curr = 0.0;
  double dxy_prev = 0.0;
  double dxy_ave = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  // Publisher chars
  char base_link[] = "/base_link";
  char odom[] = "/odom";

  ros::Duration d(1.0);

  // Get parameters from node
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);
  nh_private_.getParam("angular_scale_accel", angular_scale_accel);
  nh_private_.getParam("alpha", alpha);
  nh_private_.getParam("use_imu", use_imu);

  ros::Rate r(rate);

  // Node handle receiving info
  while(nh.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();

    dt = rpm_dt;
    dxy_ave = (rpm_act1 + rpm_act2)*dt*wheel_diameter*pi/(2*60);
    dth_odom = (rpm_act2 - rpm_act1)*dt*wheel_diameter*pi/(60*track_width);

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    dth = alpha*dth_odom;

    if(dth > 0) dth *= angular_scale_positive;
    if(dth < 0) dth *= angular_scale_negative;
    if(dxyave > 0) dxy_ave *= linear_scale_positive;
    if(dxyave < 0) dxy_av *= linear_scale_negative;

    dx = cos(dth)*dxy_ave;
    dy = -sin(dth)*dxy_ave;

    // Position calculation
    x_pos += ( cos(theta)*dx - sin(theta)*dy );
    y_pos += ( sin(theta)*dx + cos(theta)*dy );
    theta += dth;

    if(theta >= 2*pi) theta -= 2*pi;
    if(theta <= -2*pi) theta += 2*pi;

    // Publish Quaternion odometry message
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;

      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;

      broadcaster.sendTransform(t);
    }

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    // Header
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    //Pose
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.positiom.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    // Twist
    set_covariance(rpm_act1, rpm_act2);
    vx = (dt == 0)? 0 : dxy_ave/dt;
    vth = (dt == 0)? 0 : dth/dt;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dth;

    // Publish
    odom_pub.publish(odom_msg);
    last_time = current_time;
    r.sleep();  
  }
}
