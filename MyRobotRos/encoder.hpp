#pragma once

#include <ros.h>
#include <AS5048A.h>
#include <spline.h>
#include <std_msgs/Float32.h>

// Encoder messages
std_msgs::Float32 encoder_left_pos_msg;
std_msgs::Float32 encoder_right_pos_msg;
// Encoder devices
AS5048A encoder_left(encoder_left_pin);
AS5048A encoder_right(encoder_right_pin);
// ROS publishers
ros::Publisher encoder_left_pos_pub(LEFT_ENCODER_POSITION_TOPIC, &encoder_left_pos_msg);
ros::Publisher encoder_right_pos_pub(RIGHT_ENCODER_POSITION_TOPIC, &encoder_right_pos_msg);
// Splines for linearizing the encoders outputs
Spline<double> spline_left(REAL_LEFT, IDEAL_LEFT, sizeof(IDEAL_LEFT)/sizeof(double));
Spline<double> spline_right(REAL_RIGHT, IDEAL_RIGHT, sizeof(IDEAL_RIGHT)/sizeof(double));

void encodersLogic() {
  // Get values from sensor
  const double angle_left = encoder_left.getRotationInRadians();
  const double angle_right = encoder_right.getRotationInRadians();
  // Generate ROS messages
  encoder_left_pos_msg.data = spline_left.value(angle_left);
  encoder_right_pos_msg.data = -spline_right.value(angle_right);
  // Publish angles
  encoder_left_pos_pub.publish(&encoder_left_pos_msg);
  encoder_right_pos_pub.publish(&encoder_right_pos_msg);
}
