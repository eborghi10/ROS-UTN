#pragma once

const uint8_t EN_left = 11;
const uint8_t EN_right = 12;
const uint8_t D0_left = A0;
const uint8_t D1_left = A1;
const uint8_t D0_right = A3;
const uint8_t D1_right = A4;

const uint8_t encoder_left_pin = 6;
const uint8_t encoder_right_pin = 7;

ros::NodeHandle  nh;

double rate(100.0); // Hz

const double wheel_radius = 3.1 / 1E2;    // 3.1 cm
const double wheel_distance = 14.6 / 1E2; // 14.6 cm
