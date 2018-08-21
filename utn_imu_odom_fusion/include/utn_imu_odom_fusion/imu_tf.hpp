#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class ImuTf {
private:
    void ImuCb(const sensor_msgs::Imu& msg);
    static const char* IMU_TOPIC;
    static const char* JOINT_STATES_TOPIC;
    ros::Publisher pub_js;
    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string frame_id;
    tf::Quaternion tfq;
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    tf::TransformBroadcaster br;
    tf::Transform transform;
public:
    ImuTf(ros::NodeHandle*);
    void publishTf();
    void publishJointStates();
};

ImuTf::ImuTf(ros::NodeHandle* nodeHandle)
: nh_(*nodeHandle)
{
    transform.setOrigin(tf::Vector3(0,0,0));
    sub = nh_.subscribe(IMU_TOPIC, 10, &ImuTf::ImuCb, this);
    pub_js = nh_.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 10, true);
}

void ImuTf::ImuCb(const sensor_msgs::Imu& msg) {
    const geometry_msgs::Quaternion q = geometry_msgs::Quaternion(msg.orientation);
    tf::quaternionMsgToTF(q, tfq);
    transform.setRotation(tfq);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_link"));
}

void ImuTf::publishJointStates() {
  sensor_msgs::JointState msg;
  msg.name.push_back("link_name");
  msg.position.push_back(0.0);

  msg.header.stamp = ros::Time::now();
  pub_js.publish(msg);
}

void ImuTf::publishTf() {
  publishJointStates();
  ros::spin();
}

const char* ImuTf::IMU_TOPIC = "/imu_data";
const char* ImuTf::JOINT_STATES_TOPIC = "/joint_states";
