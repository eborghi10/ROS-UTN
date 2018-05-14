#pragma once

#include <ros.h>
#include <DCMotor.h>
#include <std_msgs/UInt16.h>

#include "pinout.h"

DCMotor dcMotorLeft(EN_left, D0_left, D1_left);
DCMotor dcMotorRight(EN_right, D0_right, D1_right);

void dcmotor_left_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    dcMotorLeft.setSpeed(cmd_msg.data);
}

void dcmotor_right_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    dcMotorRight.setSpeed(-cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> subMotorLeft("output/left", dcmotor_left_cb);
ros::Subscriber<std_msgs::UInt16> subMotorRight("output/right", dcmotor_right_cb);
