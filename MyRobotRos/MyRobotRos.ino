#include "motors.hpp"
#include "encoder.hpp"
#include "odometry.hpp"

uint32_t lastTime;
uint16_t rate(50);

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

  initial_left_motor_angle = encoder_left.getRotationInRadians();
  initial_right_motor_angle = encoder_right.getRotationInRadians();
  
  last_time = nh.now();

  fillOdometryMsg();
}

void loop(){
  const uint32_t currentTime = micros();
  if((currentTime - lastTime) >= 1E-6/rate)
  {
    lastTime = currentTime;
    encodersLogic();
    odometry();
  }
  nh.spinOnce();
}
