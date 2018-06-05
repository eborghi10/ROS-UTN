#pragma once

#include <ros.h>
#include <AS5048A.h>
#include <spline.h>
#include <std_msgs/Float32.h>

#include "angle.hpp"

std_msgs::Float32 encoder_left_vel_msg;
std_msgs::Float32 encoder_right_vel_msg;
std_msgs::Float32 encoder_left_pos_msg;
std_msgs::Float32 encoder_right_pos_msg;

AS5048A encoder_left(encoder_left_pin);
AS5048A encoder_right(encoder_right_pin);

Angle current_left_motor_angle(0);
Angle current_right_motor_angle(0);
Angle last_left_motor_angle(0);
Angle last_right_motor_angle(0);

ros::Publisher encoder_left_vel_pub("encoder/left/velocity", &encoder_left_vel_msg);
ros::Publisher encoder_right_vel_pub("encoder/right/velocity", &encoder_right_vel_msg);
ros::Publisher encoder_left_pos_pub("encoder/left/position", &encoder_left_pos_msg);
ros::Publisher encoder_right_pos_pub("encoder/right/position", &encoder_right_pos_msg);

double wl;
double wr;

Spline<double> spline_left(real_left, ideal_left, NUM_POINTS_LEFT);
Spline<double> spline_right(real_right, ideal_right, NUM_POINTS_RIGHT);

double get_velocity(Angle angle_t1, Angle angle_t0, double dT){
  // Angle unwrapping
  const double dAngle = Angle::Unwrap_PI(angle_t0, angle_t1);
  return dAngle / dT;
}

void encodersLogic(double dT){
  // Get values from sensor
  const double raw_angle_left = encoder_left.getRotationInRadians();
  const double raw_angle_right = encoder_right.getRotationInRadians();
  // Angle calibration
  current_left_motor_angle = spline_left.value(raw_angle_left);
  current_right_motor_angle = spline_right.value(raw_angle_right);
  // Publish angles
  encoder_left_pos_msg.data = current_left_motor_angle.GetAngle();
  encoder_right_pos_msg.data = current_right_motor_angle.GetAngle();
  encoder_left_pos_pub.publish(&encoder_left_pos_msg);
  encoder_right_pos_pub.publish(&encoder_right_pos_msg);

  wl = get_velocity(current_left_motor_angle, last_left_motor_angle, dT);
  encoder_left_vel_msg.data = wl;
  encoder_left_vel_pub.publish( &encoder_left_vel_msg );

  wr = (-1) * get_velocity(current_right_motor_angle, last_right_motor_angle, dT);
  encoder_right_vel_msg.data = wr;
  encoder_right_vel_pub.publish( &encoder_right_vel_msg );

  last_right_motor_angle = current_right_motor_angle;
  last_left_motor_angle = current_left_motor_angle;
}
