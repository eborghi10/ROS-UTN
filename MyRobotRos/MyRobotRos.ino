#include "motors.hpp"
#include "encoder.hpp"
// #include "odometry.hpp"

std_msgs::Float32 time_msg;
ros::Publisher time_pub("time", &time_msg);
double last_time;

void setup(){
  encoder_left.begin();
  encoder_right.begin();
  // Right motor turns to the other side
  dcMotorRight.setClockwise(false);

  nh.initNode();
  nh.subscribe(subMotorLeft);
  nh.subscribe(subMotorRight);
//   nh.advertise(encoder_left_vel_pub);
//   nh.advertise(encoder_right_vel_pub);
  nh.advertise(encoder_left_pos_pub);
  nh.advertise(encoder_right_pos_pub);
//   nh.advertise(odom_pub);
  nh.advertise(time_pub);

  last_time = millis();

//   fillOdometryMsg();
}

void loop(){
  const double current_time = millis() / 1E3;
  const double delta_time = (current_time - last_time);
  if(delta_time >= 1.0/rate)
  {
    time_msg.data = current_time;
    time_pub.publish(&time_msg);
    encodersLogic(delta_time);
    // odometry(delta_time);
    last_time = current_time;
  }
  nh.spinOnce();
}
