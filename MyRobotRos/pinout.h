#pragma once

const uint8_t EN_left = 2;
const uint8_t EN_right = 3;
const uint8_t D0_left = 4;
const uint8_t D1_left = 5;
const uint8_t D0_right = 8;
const uint8_t D1_right = 9;

const uint8_t encoder_left_pin = 6;
const uint8_t encoder_right_pin = 7;

ros::NodeHandle  nh;

const double wheel_radius = 6.4 / 1E2;    // 6.4 cm
const double wheel_distance = 14.6 / 1E2; // 14.6 cm
