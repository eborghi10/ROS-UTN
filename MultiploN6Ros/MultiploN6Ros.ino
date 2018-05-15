#include <ros.h>
#include <tf/tf.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Quaternion.h>
#include <rosabridge_msgs/Odometry.h>

/*
 * Mensajes que deberian usarse:
 *
 * Motores:     geometry_msgs/Twist
 * Encoders:    sensor_msgs/JointState
 * Ultrasonico: sensor_msgs/Range
 */

#include <DCMotor.h>
#include <Ping.h>

#include "angle.hpp"
#include "pinout.h"

ros::NodeHandle  nh;

std_msgs::Float32 encoder_left_msg;
std_msgs::Float32 encoder_right_msg;
std_msgs::Float32 ping_msg;
rosabridge_msgs::Odometry odom_msg;

DCMotor dcMotorLeft(EN_left, D0_left, D1_left);
DCMotor dcMotorRight(EN_right, D0_right, D1_right);

/**
 * Odometria
 */

enum Dir{BW=-1, QUIET=0, FW=1};

Dir left_motor_dir = Dir::QUIET;
Dir right_motor_dir = Dir::QUIET;

Angle current_left_motor_angle(0);
Angle current_right_motor_angle(0);
Angle last_left_motor_angle(0);
Angle last_right_motor_angle(0);

ros::Time last_time_left;
ros::Time last_time_right;
ros::Time current_time;

double wl;
double wr;

void dcmotor_left_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    if(cmd_msg.data > 1e-2) left_motor_dir = Dir::FW;
    else if(cmd_msg.data < 1e-2) left_motor_dir = Dir::BW;
    else left_motor_dir = Dir::QUIET;
    dcMotorLeft.setSpeed(cmd_msg.data);
}

void dcmotor_right_cb(const std_msgs::UInt16& cmd_msg){
    //set servo angle, should be from 0-180
    if(cmd_msg.data > 1e-2) right_motor_dir = Dir::FW;
    else if(cmd_msg.data < 1e-2) right_motor_dir = Dir::BW;
    else right_motor_dir = Dir::QUIET;
    dcMotorRight.setSpeed(cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> subMotorLeft("output/left", dcmotor_left_cb);
ros::Subscriber<std_msgs::UInt16> subMotorRight("output/right", dcmotor_right_cb);

ros::Publisher encoder_left_pub("encoder/left", &encoder_left_msg);
ros::Publisher encoder_right_pub("encoder/right", &encoder_right_msg);
ros::Publisher ping_pub("ping", &ping_msg);
ros::Publisher odom_pub("rosabridge/odom", &odom_msg);

enum State{LEFT, RIGHT, STATES};

uint16_t oldState[STATES];
uint16_t newState[STATES];
uint32_t lastTime;
uint16_t period(150);

const double wheel_radius = 6.4 / 1E2;    // 6.4 cm
const double wheel_distance = 14.6 / 1E2; // 14.6 cm
double dT;

// Position of the robot
double x = 0.0;
double y = 0.0;
double th = 0.0;

const char base_link[] = "base_link";
const char odom[] = "odom";

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
  nh.advertise(odom_pub);
  oldState[LEFT] = digitalRead(encoder_left);
  oldState[RIGHT] = digitalRead(encoder_right);
  last_time_left = nh.now();
  fillOdometryMsg();s
  last_time_right = last_time_left;
}

const Angle DEG_PER_REV(22.5);

Angle count_angle(Angle angle, int8_t dir){
  if(dir == Dir::FW) angle += DEG_PER_REV;
  else if(dir == Dir::BW) angle -= DEG_PER_REV;
  else angle = 0; // Dir::QUIET
  angle.NormalizeAngle();
  return angle;
}

void fillOdometryMsg()
{
  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;

  // CBA Set Odom covariances  
  odom_msg.pose_covariance[0] = 0.0;
  odom_msg.pose_covariance[1] = 0.0;
  odom_msg.pose_covariance[2] = 0.0;
  odom_msg.pose_covariance[3] = 0.0;
  odom_msg.pose_covariance[4] = 0.0;
  odom_msg.pose_covariance[5] = 0.0;

  odom_msg.twist_covariance[0] = 0.0;
  odom_msg.twist_covariance[1] = 0.0;
  odom_msg.twist_covariance[2] = 0.0;
  odom_msg.twist_covariance[3] = 0.0;
  odom_msg.twist_covariance[4] = 0.0;
  odom_msg.twist_covariance[5] = 0.0;
}

void PublishOdometry(double vx, double vy, double vyaw)
{
  geometry_msgs::Quaternion quat = tf::createQuaternionFromYaw(th);

  odom_msg.header.seq++;
  odom_msg.header.stamp = current_time;
  odom_msg.pose.position.x = x;
  odom_msg.pose.position.y = y;
  odom_msg.pose.position.z = 0.0;
  odom_msg.pose.orientation.x = quat.x;
  odom_msg.pose.orientation.y = quat.y;
  odom_msg.pose.orientation.z = quat.z;
  odom_msg.pose.orientation.w = quat.w;
  odom_msg.twist.linear.x = vx;
  odom_msg.twist.linear.y = vy;
  odom_msg.twist.linear.z = 0.0;
  odom_msg.twist.angular.x = 0.0;
  odom_msg.twist.angular.y = 0.0;
  odom_msg.twist.angular.z = vyaw;
  odom_pub.publish( &odom_msg );
}

/**
 * https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/?answer=231954#post-id-231954
 */
void odometry()
{
  // Linear velocities of the wheels
  double v_left = wl * wheel_radius;
  double v_right = wr * wheel_radius;
  
  // Velocities in the robot frame
  double v_rx = ( v_right + v_left ) /2.0;
  double v_ry = 0; // we have a non-holonomic constraint (for a holonomic robot, this is non-zero)
  double omega_r = ( v_right - v_left ) / wheel_distance; // d denotes the distance between both wheels (track)

  // Velocities in the odom frame
  //compute odometry in a typical way given the velocities of the robot
  double delta_x = (v_rx * cos(th) - v_ry * sin(th)) * dT;
  double delta_y = (v_rx * sin(th) + v_ry * cos(th)) * dT;
  double delta_th = omega_r * dT;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  PublishOdometry(delta_x, delta_y, omega_r);
}

double get_velocity(Angle angle_t1, Angle angle_t0, ros::Time t1, ros::Time t0){
  Angle dAngle(angle_t1 - angle_t0);
  dT = (t1.toSec() - t0.toSec()) + (t1.toNsec() - t0.toNsec())/1E9;
  return dAngle.GetAngle() / dT;
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
      wl = get_velocity(
          current_left_motor_angle, last_left_motor_angle,
          current_time, last_time_left);
      encoder_left_msg.data = wl;
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
      wr = get_velocity(
            current_right_motor_angle, last_right_motor_angle,
            current_time, last_time_right);
      encoder_right_msg.data = wr;
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
    odometry();
  }
  nh.spinOnce();
  delay(1);
}
