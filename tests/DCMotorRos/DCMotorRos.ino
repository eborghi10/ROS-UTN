/**
 * Tests a DC motor using a PWM signal
 */

#include <ros.h>
#include <std_msgs/UInt16.h>

#include <DCMotor.h>

ros::NodeHandle  nh;

const int EN = A15;
const int D0 = A13;
const int D1 = A12;

DCMotor dcMotor(EN, D0, D1);

void servo_cb( const std_msgs::UInt16& cmd_msg){
  dcMotor.setSpeed(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
