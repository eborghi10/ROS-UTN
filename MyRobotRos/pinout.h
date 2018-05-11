#pragma once

const uint8_t EN_left = A14;
const uint8_t EN_right = A15;
const uint8_t D0_left = A10;
const uint8_t D1_left = A11;
const uint8_t D0_right = A12;
const uint8_t D1_right = A13;

const uint8_t encoder_left_pin = 6;
const uint8_t encoder_right_pin = 7;

ros::NodeHandle  nh;

const double wheel_radius = 6.4 / 1E2;    // 6.4 cm
const double wheel_distance = 14.6 / 1E2; // 14.6 cm
