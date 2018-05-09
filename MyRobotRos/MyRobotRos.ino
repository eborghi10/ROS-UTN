#include "motors.hpp"
#include "encoder.hpp"

uint32_t lastTime;
uint16_t period(50);

void setup(){
  pinMode(encoder_left_pin, INPUT);
  pinMode(encoder_right_pin, INPUT);
  encoder_left.init();
  encoder_right.init();
  
  nh.initNode();
  nh.subscribe(subMotorLeft);
  nh.subscribe(subMotorRight);
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);

  initial_left_motor_angle = read2angle( encoder_left.getRawRotation() );
  initial_right_motor_angle = read2angle( encoder_right.getRawRotation() );
  
  last_time = nh.now();
}

/**
 * https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/?answer=231954#post-id-231954
 */
void odometry()
{
  // Get translational velocities of each wheel
  double v_left = wl * wheel_radius;
  doubel v_right = wr * wheel_radius;
  
  // Velocities in the robot frame
  // located in the center of rotation (base_link frame)
  double v_rx = (v_right + v_left) / 2.0;
  double v_ry = 0.0;
  double omega_r = (v_right - v_left) / wheel_distance;

  // Velocities in the odom frame
  // "theta": current orientation
  double theta = 0.0;
  double v_wx = v_rx * cos(theta) - v_ry * sin(theta);
  double v_wy = v_rx * sin(theta) + v_ry * cos(theta);
  double thetadot = omega_r;

  // Computing the current robot pose
  // Applying Explicit/Forward Euler Integration
  double x_kp1 = x_k + v_wx * dt; // kp1 := k+1
  double y_kp1 = y_k + v_wy * dt; // by the way, the last term is delta_y from the equations in your comment
  double theta_kp1 = theta_k + thetadot * dt;
}

void loop(){
  const uint32_t currentTime = millis();
  if((currentTime - lastTime) >= period)
  {
    lastTime = currentTime;
    encodersLogic();
    odometry();
  }
  nh.spinOnce();
  delay(1);
}
