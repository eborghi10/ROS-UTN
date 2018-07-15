#include "mpu6050.h"

IMUSensor *imu_sensor;

void setup() {
    imu_sensor = new IMUSensor();
}

void loop() {
    imu_sensor->publish();
}
