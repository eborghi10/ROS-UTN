#pragma once

#include <ros.h>
#include <DCMotor.h>
#include <std_msgs/Int16.h>

const uint8_t EN_left = 11;
const uint8_t EN_right = 12;
const uint8_t D0_left = A0;
const uint8_t D1_left = A1;
const uint8_t D0_right = A3;
const uint8_t D1_right = A4;

const char* LEFT_MOTOR_TOPIC = "output/left";
const char* RIGHT_MOTOR_TOPIC = "output/right";

DCMotor dcMotorLeft(EN_left, D0_left, D1_left);
DCMotor dcMotorRight(EN_right, D0_right, D1_right);

void dcmotor_left_cb(const std_msgs::Int16& cmd_msg){
    dcMotorLeft.setSpeed(cmd_msg.data);
}

void dcmotor_right_cb(const std_msgs::Int16& cmd_msg){
    dcMotorRight.setSpeed(cmd_msg.data);
}

ros::Subscriber<std_msgs::Int16> subMotorLeft(LEFT_MOTOR_TOPIC, dcmotor_left_cb);
ros::Subscriber<std_msgs::Int16> subMotorRight(RIGHT_MOTOR_TOPIC, dcmotor_right_cb);
