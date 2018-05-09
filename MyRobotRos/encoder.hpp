#pragma once

#include <ros.h>
#include <AS5048A.h>
#include <std_msgs/Float32.h>

#include "angle.hpp"

ros::Time last_time;
ros::Time current_time;

std_msgs::Float32 encoder_left_msg;
std_msgs::Float32 encoder_right_msg;

AS5048A encoder_left(encoder_left_pin);
AS5048A encoder_right(encoder_right_pin);

Angle initial_left_motor_angle(0);
Angle initial_right_motor_angle(0);
Angle current_left_motor_angle(0);
Angle current_right_motor_angle(0);
Angle last_left_motor_angle(0);
Angle last_right_motor_angle(0);

ros::Publisher encoder_left_pub("encoder/left", &encoder_left_msg);
ros::Publisher encoder_right_pub("encoder/right", &encoder_right_msg);

double wl;
double wr;

double read2angle(uint16_t register_output) 
{
  return register_output * (static_cast<double>(TWO_PI / 16383.0));
}

void ReadAngles() 
{
  double angle_left = read2angle( encoder_left.getRawRotation() );
  current_left_motor_angle.SetAngle( angle_left );
  double angle_right = read2angle( encoder_right.getRawRotation() );
  current_right_motor_angle.SetAngle( angle_right );

  current_left_motor_angle -= initial_left_motor_angle;
  current_right_motor_angle -= initial_right_motor_angle;

  current_left_motor_angle.NormalizeAngle();
  current_right_motor_angle.NormalizeAngle();
}

double get_velocity(Angle angle_t1, Angle angle_t0, ros::Time t1, ros::Time t0){
  Angle dAngle(angle_t1 - angle_t0);
  double dT = (t1.toNsec() - t0.toNsec())/1E9;
  return dAngle.GetAngle();// / dT;
}


void encodersLogic(){
  ReadAngles();

  current_time = nh.now();

  wl = get_velocity(
      current_left_motor_angle, last_left_motor_angle,
      current_time, last_time);
  encoder_left_msg.data = wl;
  encoder_left_pub.publish( &encoder_left_msg );

  wr = (-1) * get_velocity(
        current_right_motor_angle, last_right_motor_angle,
        current_time, last_time);
  encoder_right_msg.data = wr;
  encoder_right_pub.publish( &encoder_right_msg );

  last_right_motor_angle = current_right_motor_angle;
  last_left_motor_angle = current_left_motor_angle;
  last_time = current_time;
}

