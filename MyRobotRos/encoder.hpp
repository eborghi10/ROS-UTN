#pragma once

#include <ros.h>
#include <AS5048A.h>
#include <std_msgs/Float32.h>

#include "angle.hpp"

ros::Time last_time;
ros::Time current_time;

double dT;

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

double get_velocity(Angle angle_t1, Angle angle_t0, ros::Time t1, ros::Time t0){
  // Angle unwrapping
  double dAngle = Angle::Unwrap_PI(angle_t0, angle_t1);
  // Timestamp
  dT = abs(t1.toNsec() - t0.toNsec())/1E9;
 return dAngle / dT;
}

void encodersLogic(){
  current_left_motor_angle = encoder_left.getRotationInRadians();
  current_right_motor_angle = encoder_right.getRotationInRadians();

  wl = (-1) * get_velocity(
      current_left_motor_angle, last_left_motor_angle,
      current_time, last_time);
  encoder_left_msg.data = wl;
  encoder_left_pub.publish( &encoder_left_msg );

  wr = get_velocity(
        current_right_motor_angle, last_right_motor_angle,
        current_time, last_time);
  encoder_right_msg.data = wr;
  encoder_right_pub.publish( &encoder_right_msg );

  last_right_motor_angle = current_right_motor_angle;
  last_left_motor_angle = current_left_motor_angle;
}

