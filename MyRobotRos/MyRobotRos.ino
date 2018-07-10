#include "motors.hpp"
#include "encoder.hpp"

double last_time;

void setup(){
  encoder_left.begin();
  encoder_right.begin();
  // Right motor turns to the other side
  dcMotorRight.setClockwise(false);

  nh.initNode();
  // Motors subscribers
  nh.subscribe(subMotorLeft);
  nh.subscribe(subMotorRight);
  // Encoders publishers
  nh.advertise(encoder_left_pos_pub);
  nh.advertise(encoder_right_pos_pub);

  last_time = millis();
}

void loop(){
  const double current_time = millis() / 1E3;
  const double delta_time = (current_time - last_time);
  if(delta_time >= 1.0/rate)
  {
    encodersLogic();
    last_time = current_time;
  }
  nh.spinOnce();
}
