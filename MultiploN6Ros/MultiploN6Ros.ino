#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

/*
 * Mensajes que deberian usarse:
 * 
 * Motores:     geometry_msgs/Twist
 * Encoders:    sensor_msgs/JointState
 * Ultrasonico: sensor_msgs/Range
 */

#include <DCMotor.h>
#include <Ping.h>

#include "pinout.h"

ros::NodeHandle  nh;

std_msgs::UInt16 encoder_left_msg;
std_msgs::UInt16 encoder_right_msg;
std_msgs::Float32 ping_msg;

DCMotor dcMotorLeft(EN_left, D0_left, D1_left);
DCMotor dcMotorRight(EN_right, D0_right, D1_right);

void dcmotor_left_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180 
    dcMotorLeft.setSpeed(cmd_msg.data);
}

void dcmotor_right_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180 
    dcMotorRight.setSpeed(cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> subMotorLeft("output/left", dcmotor_left_cb);
ros::Subscriber<std_msgs::UInt16> subMotorRight("output/right", dcmotor_right_cb);

ros::Publisher encoder_left_pub("encoder/left", &encoder_left_msg);
ros::Publisher encoder_right_pub("encoder/right", &encoder_right_msg);
ros::Publisher ping_pub("ping", &ping_msg);

enum State{LEFT, RIGHT, STATES};

uint16_t oldState[STATES];
uint16_t newState[STATES];
uint32_t lastTime;
uint16_t period(150);

PingSensor ping(ping_pin);

void setup(){
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  nh.initNode();
  nh.subscribe(subMotorLeft);
  nh.subscribe(subMotorRight);
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);
  nh.advertise(ping_pub);
  oldState[LEFT] = digitalRead(encoder_left);
  oldState[RIGHT] = digitalRead(encoder_right);
}

void encodersLogic(){
  // Envia cada flanco creciente
  newState[LEFT] = digitalRead(encoder_left);
  newState[RIGHT] = digitalRead(encoder_right);
  
  if(oldState[LEFT] != newState[LEFT])
  {
    if(newState[LEFT])
    {
      // Agregar timestamp
      encoder_left_msg.data = newState[LEFT];
      encoder_left_pub.publish( &encoder_left_msg );
    }
    oldState[LEFT] = newState[LEFT];
  }

  if(oldState[RIGHT] != newState[RIGHT])
  {
    if(newState[RIGHT])
    {
      // Agregar timestamp
      encoder_right_msg.data = newState[RIGHT];
      encoder_right_pub.publish( &encoder_right_msg );
    }
    oldState[RIGHT] = newState[RIGHT];
  }
}

void pingLogic(){
  // Envia informacion en metros
  // Agregar timestamp
  ping_msg.data = ping.measureCM() / 100.0;
  ping_pub.publish( &ping_msg );
}

void loop(){
  const uint32_t currentTime = millis();
  if((currentTime - lastTime) >= period)
  {
    lastTime = currentTime;
    encodersLogic();
    pingLogic();
  }
  nh.spinOnce();
  delay(1);
}
