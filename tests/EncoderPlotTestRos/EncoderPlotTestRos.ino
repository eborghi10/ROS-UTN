#include <ros.h>
#include <AS5048A.h>
#include <DCMotor.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

/**
 * $ roscore
 * $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
 * $ rqt_plot /encoder
 * $ rostopic pub /output std_msgs/UInt16 "data: 100"
 */

AS5048A angleSensor(7);
DCMotor dcMotor(A15, A13, A12);

void dcmotor_cb(const std_msgs::UInt16& cmd_msg){
    dcMotor.setSpeed(cmd_msg.data);
}

ros::NodeHandle  nh;
std_msgs::Float32 encoder_msg;
ros::Publisher encoder_pub("encoder", &encoder_msg);
ros::Subscriber<std_msgs::UInt16> subMotor("output", dcmotor_cb);

void setup()
{
  angleSensor.begin();
  nh.initNode();
  nh.subscribe(subMotor);
  nh.advertise(encoder_pub);
}

void loop()
{
  delay(50);

  // Select one of these possibilities
  double val = angleSensor.getRotationInRadians();
//  double val = angleSensor.getRotationInDegrees();
//  uint16_t val = angleSensor.getRawRotation();

  encoder_msg.data = val;
  encoder_pub.publish( &encoder_msg );

  nh.spinOnce();
}
