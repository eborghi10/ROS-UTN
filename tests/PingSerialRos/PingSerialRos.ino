/**
 * Tests the ultrasonic sensor using ROS
 */

#include <Ping.h>

#include <ros.h>
#include <std_msgs/Float32.h> // Usar sensor_msgs/Range

const int PIN = A1;

PingSensor ping(PIN);

ros::NodeHandle  nh;
std_msgs::Float32 ping_msg;
ros::Publisher ping_pub("ping", &ping_msg);

void setup()
{
  nh.initNode();
  nh.advertise(ping_pub);
}

void loop()
{
  // Envia informacion en metros
  ping_msg.data = ping.measureCM() / 100.0;
  ping_pub.publish( &ping_msg );
  nh.spinOnce();
  delay(200);
}

