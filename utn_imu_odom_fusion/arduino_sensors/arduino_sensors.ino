#include "encoder.hpp"
#include "mpu6050.h"
#include "ArduinoHardware.h"

ros::NodeHandle nh;
Encoder *encoders;
IMUSensor *imu_sensor;

void setup() {
    nh.initNode();
    encoders = new Encoder(&nh);
    imu_sensor = new IMUSensor(&nh);
}

void loop() {
    nh.spinOnce();
    encoders->publish();
    imu_sensor->publish(&nh);
}
