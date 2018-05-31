#pragma once

#include <ros.h>
#include <AS5048A.h>
#include <spline.h>
#include <std_msgs/Float32.h>

#include "angle.hpp"

ros::Time last_time;
ros::Time current_time;

std_msgs::Float32 encoder_left_msg;
std_msgs::Float32 encoder_right_msg;

AS5048A encoder_left(encoder_left_pin);
AS5048A encoder_right(encoder_right_pin);

Angle current_left_motor_angle(0);
Angle current_right_motor_angle(0);
Angle last_left_motor_angle(0);
Angle last_right_motor_angle(0);

ros::Publisher encoder_left_pub("encoder/left", &encoder_left_msg);
ros::Publisher encoder_right_pub("encoder/right", &encoder_right_msg);

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

  wl = get_velocity(current_left_motor_angle, last_left_motor_angle, dT);
  encoder_left_msg.data = wl;
  encoder_left_pub.publish( &encoder_left_msg );

  wr = (-1) * get_velocity(current_right_motor_angle, last_right_motor_angle, dT);
  encoder_right_msg.data = wr;
  encoder_right_pub.publish( &encoder_right_msg );

  last_right_motor_angle = current_right_motor_angle;
  last_left_motor_angle = current_left_motor_angle;
}
