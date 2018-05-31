#pragma once

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <rosabridge_msgs/Odometry.h>

rosabridge_msgs::Odometry odom_msg;

ros::Publisher odom_pub("rosabridge/odom", &odom_msg);

// Position of the robot
double x = 0.0;
double y = 0.0;
double th = 0.0;

const char base_link[] = "base_link";
const char odom[] = "odom";

void fillOdometryMsg()
{
  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;

  // CBA Set Odom covariances
  odom_msg.pose_covariance[0] = 0.0;
  odom_msg.pose_covariance[1] = 0.0;
  odom_msg.pose_covariance[2] = 0.0;
  odom_msg.pose_covariance[3] = 0.0;
  odom_msg.pose_covariance[4] = 0.0;
  odom_msg.pose_covariance[5] = 0.0;

  odom_msg.twist_covariance[0] = 0.0;
  odom_msg.twist_covariance[1] = 0.0;
  odom_msg.twist_covariance[2] = 0.0;
  odom_msg.twist_covariance[3] = 0.0;
  odom_msg.twist_covariance[4] = 0.0;
  odom_msg.twist_covariance[5] = 0.0;
}

void PublishOdometry(double vx, double vy, double vyaw)
{
  geometry_msgs::Quaternion quat = tf::createQuaternionFromYaw(th);

  odom_msg.header.seq++;
  odom_msg.header.stamp = current_time;
  odom_msg.pose.position.x = x;
  odom_msg.pose.position.y = y;
  odom_msg.pose.position.z = 0.0;
  odom_msg.pose.orientation.x = quat.x;
  odom_msg.pose.orientation.y = quat.y;
  odom_msg.pose.orientation.z = quat.z;
  odom_msg.pose.orientation.w = quat.w;
  odom_msg.twist.linear.x = vx;
  odom_msg.twist.linear.y = vy;
  odom_msg.twist.linear.z = 0.0;
  odom_msg.twist.angular.x = 0.0;
  odom_msg.twist.angular.y = 0.0;
  odom_msg.twist.angular.z = vyaw;
  odom_pub.publish( &odom_msg );
}

/**
 * https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/?answer=231954#post-id-231954
 */
void odometry(double dT)
{
  // Linear velocities of the wheels
  double v_left = wl * wheel_radius;
  double v_right = wr * wheel_radius;

  // Velocities in the robot frame
  double v_rx = ( v_right + v_left ) / 2.0;
  double v_ry = 0; // we have a non-holonomic constraint (for a holonomic robot, this is non-zero)
  double omega_r = ( v_right - v_left ) / wheel_distance; // d denotes the distance between both wheels (track)

  // Velocities in the odom frame
  //compute odometry in a typical way given the velocities of the robot
  double delta_x = (v_rx * cos(th) - v_ry * sin(th)) * dT;
  double delta_y = (v_rx * sin(th) + v_ry * cos(th)) * dT;
  double delta_th = omega_r * dT;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  PublishOdometry(delta_x, delta_y, omega_r);
}
