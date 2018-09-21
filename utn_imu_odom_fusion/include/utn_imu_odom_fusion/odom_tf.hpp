#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

class OdomTf {
private:
  void LeftWheelCb(const std_msgs::Float32& msg);
  void RightWheelCb(const std_msgs::Float32& msg);
  static const char* JOINT_STATES_TOPIC;
  static const char* LEFT_WHEEL_POS_TOPIC;
  static const char* RIGHT_WHEEL_POS_TOPIC;
  ros::Publisher pub_js;
  ros::Subscriber sub_left_wheel;
  ros::Subscriber sub_right_wheel;
  ros::NodeHandle nh_;
  double left_angle;
  double right_angle;
  tf::Transform transform;
  tf::TransformBroadcaster br;
public:
  OdomTf(ros::NodeHandle*);
  void publishWheels();
};

OdomTf::OdomTf(ros::NodeHandle* nodeHandle)
: nh_(*nodeHandle)
{
  pub_js = nh_.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 10, true);
  sub_left_wheel = nh_.subscribe(LEFT_WHEEL_POS_TOPIC, 1, &OdomTf::LeftWheelCb, this);
  sub_right_wheel = nh_.subscribe(RIGHT_WHEEL_POS_TOPIC, 1, &OdomTf::RightWheelCb, this);
  transform.setOrigin(tf::Vector3(0,0,0));
}

void OdomTf::LeftWheelCb(const std_msgs::Float32& msg) {
  this->left_angle = msg.data;
}

void OdomTf::RightWheelCb(const std_msgs::Float32& msg) {
  this->right_angle = msg.data;
}

void OdomTf::publishWheels() {
  sensor_msgs::JointState msg;

  msg.name.push_back("left_wheel");
  msg.position.push_back(this->left_angle);
  msg.name.push_back("right_wheel");
  msg.position.push_back(this->right_angle);

  msg.header.stamp = ros::Time::now();
  pub_js.publish(msg);

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, this->left_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "left_wheel"));
  q = tf::createQuaternionFromRPY(0, 0, this->right_angle);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "right_wheel"));
}

const char* OdomTf::JOINT_STATES_TOPIC = "/joint_states";
const char* OdomTf::LEFT_WHEEL_POS_TOPIC = "/encoder/left/position";
const char* OdomTf::RIGHT_WHEEL_POS_TOPIC = "/encoder/right/position";
