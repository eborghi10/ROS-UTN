#include "motors.hpp"
#include "encoder.hpp"
#include "odometry.hpp"

std_msgs::Float32 time_msg;
ros::Publisher time_pub("time", &time_msg);

void setup(){
  encoder_left.begin();
  encoder_right.begin();
  // Right motor turns to the other side
  dcMotorRight.setClockwise(false);

  nh.initNode();
  nh.subscribe(subMotorLeft);
  nh.subscribe(subMotorRight);
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);
  nh.advertise(odom_pub);
  nh.advertise(time_pub);

  last_time = nh.now();

  fillOdometryMsg();
}

void loop(){
  current_time = nh.now();
  const double deltaTime = (current_time.toNsec() - last_time.toNsec())/1E9;
  if(deltaTime >= 1.0/rate)
  {
    time_msg.data = current_time.toSec();
    time_pub.publish(&time_msg);
    encodersLogic(deltaTime);
    odometry(deltaTime);
    last_time = current_time;
  }
  nh.spinOnce();
}
