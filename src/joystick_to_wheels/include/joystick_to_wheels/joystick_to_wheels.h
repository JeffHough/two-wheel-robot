#ifndef JOYSTICK_TO_WHEELS_H_
#define JOYSTICK_TO_WHEELS_H_

#include <Eigen/Dense>

// Gets just the right-wheel speed (the left-wheel is solved from the same function, time-shifted):
double normalized_right_wheel_speed(double theta);
double normalized_left_wheel_speed(double theta);

// Takes in a single (r, theta) of the joystick. "r" will go from 0->1, theta from 0->2*pi.
// Outputs a vector of two speeds, one for each wheel. This "speed" will be the volgate at each
// wheel... 
Eigen::Vector2d wheel_speeds_from_joystick(Eigen::Vector2d joystick, double max_voltage);

#endif