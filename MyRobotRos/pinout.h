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

const int NUM_POINTS = 39;

const double real_left[NUM_POINTS] PROGMEM = {0.00000, 0.07671, 0.18448, 0.32294, 0.48787, 0.74062, 1.16904, 1.64501, 1.99595, 2.19846, 2.32158, 2.40366, 2.46771, 2.53982, 2.61921, 2.72085, 2.85931, 3.03382, 3.25589, 3.50020, 3.74145, 3.94473, 4.10735, 4.25348, 4.38273, 4.51084, 4.63395, 4.76129, 4.90358, 5.06544, 5.24494, 5.42520, 5.59204, 5.74776, 5.88238, 6.01739, 6.12977, 6.22680, 6.28319};
const double ideal[NUM_POINTS] PROGMEM = {0.00000, 0.16535, 0.33069, 0.49604, 0.66139, 0.82673, 0.99208, 1.15743, 1.32278, 1.48812, 1.65347, 1.81882, 1.98416, 2.14951, 2.31486, 2.48020, 2.64555, 2.81090, 2.97625, 3.14159, 3.30694, 3.47229, 3.63763, 3.80298, 3.96833, 4.13367, 4.29902, 4.46437, 4.62972, 4.79506, 4.96041, 5.12576, 5.29110, 5.45645, 5.62180, 5.78714, 5.95249, 6.11784, 6.28319};
