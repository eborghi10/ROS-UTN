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

std_msgs::Float32 encoder_left_msg;
std_msgs::Float32 encoder_right_msg;
std_msgs::Float32 ping_msg;

DCMotor dcMotorLeft(EN_left, D0_left, D1_left);
DCMotor dcMotorRight(EN_right, D0_right, D1_right);

/**
 * Odometria
 */

enum Dir{BW=-1, QUIET=0, FW=1};

int8_t left_motor_dir = Dir.QUIET;
int8_t right_motor_dir = Dir.QUIET;

double current_left_motor_angle = 0;
double current_right_motor_angle = 0;
double last_left_motor_angle = 0;
double last_right_motor_angle = 0;

const double DEG_PER_REV = 22.5;

ros::Time last_time_left;
ros::Time last_time_right;
ros::Time current_time;

void dcmotor_left_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    if(cmd_msg.data > 0) left_motor_dir = Dir.FW;
    else if(cmd_msg.data < 0) left_motor_dir = Dir.BW;
    else left_motor_dir = Dir.QUIET;
    dcMotorLeft.setSpeed(cmd_msg.data);
}

void dcmotor_right_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    if(cmd_msg.data > 0) right_motor_dir = Dir.FW;
    else if(cmd_msg.data < 0) right_motor_dir = Dir.BW;
    else right_motor_dir = Dir.QUIET;
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
  last_time = nh.now();
}

const double UPPER_ANGLE = 360.0;
const double LOWER_ANGLE = 0.0;

double wrap_angle(double angle, double upper_limit, double lower_limit){
  return a - b * floor(a / b);
}

uint16_t deg_to_rad(uint16_t deg_){
  return deg_ * M_PI / 180.0;
}

double count_angle(uint16_t angle, int8_t dir){
  if(dir == Dir.FW) angle += DEG_PER_REV;
  else if(dir == Dir.BW) angle -= DEG_PER_REV;
  return wrap_angle(angle);
}

double get_velocity(double angle_t1, double angle_t0, double t1, double t0){
  dAngle = deg_to_rad(angle_t1 - angle_t0);
  return dAngle / (t1 - t0);
}

/**
 * Envia cada flanco creciente
 */
void encodersLogic(){
  newState[LEFT] = digitalRead(encoder_left);
  newState[RIGHT] = digitalRead(encoder_right);

  current_time = nh.now();

  if(oldState[LEFT] != newState[LEFT])
  {
    if(newState[LEFT])
    {
      current_left_motor_angle = count_angle(current_left_motor_angle, left_motor_dir);
      encoder_left_msg.data = get_velocity(
          current_left_motor_angle, last_left_motor_angle,
          current_time.toSec(), last_time_left.toSec());
      encoder_left_pub.publish( &encoder_left_msg );

      last_left_motor_angle = current_left_motor_angle;
      last_time_left = current_time;
    }
    oldState[LEFT] = newState[LEFT];
  }

  if(oldState[RIGHT] != newState[RIGHT])
  {
    if(newState[RIGHT])
    {
      current_right_motor_angle = count_angle(current_right_motor_angle, right_motor_dir);
      encoder_right_msg.data = get_velocity(
            current_right_motor_angle, last_right_motor_angle,
            current_time.toSec(), last_time_right.toSec());
      encoder_right_pub.publish( &encoder_right_msg );

      last_right_motor_angle = current_right_motor_angle;
      last_time_right = current_time;
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
