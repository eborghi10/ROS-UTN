#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class ImuTf {
private:
    void ImuCb(const sensor_msgs::Imu& msg);
    static const char* IMU_TOPIC;
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
};

ImuTf::ImuTf(ros::NodeHandle* nodeHandle)
: nh_(*nodeHandle)
{
    transform.setOrigin(tf::Vector3(0,0,0));
    sub = nh_.subscribe(IMU_TOPIC, 10, &ImuTf::ImuCb, this);
}

void ImuTf::ImuCb(const sensor_msgs::Imu& msg) {
    const geometry_msgs::Quaternion q = geometry_msgs::Quaternion(msg.orientation);
    // tfq = tf::createIdentityQuaternion();
    tf::quaternionMsgToTF(q, tfq);
    transform.setRotation(tfq);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_link"));
}

void ImuTf::publishTf() {
    ros::spin();
}

const char* ImuTf::IMU_TOPIC = "/imu/data";
