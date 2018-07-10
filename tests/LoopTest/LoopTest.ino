/**
 * Tests a constant loop with Arduino & ROS
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::Time last_time;
ros::Time current_time;
ros::NodeHandle  nh;

double rate(100.0); // Hz

std_msgs::Float32 time_msg;
ros::Publisher time_pub("time", &time_msg);

void setup() {
  nh.initNode();
  nh.advertise(time_pub);
}

void loop() {
  current_time = nh.now();
  const double deltaTime = (current_time.toNsec() - last_time.toNsec())/1E9;

  if(deltaTime >= 1.0/rate)
  {
    time_msg.data = deltaTime;
    time_pub.publish( &time_msg );
    last_time = current_time;
  }
  nh.spinOnce();
}
