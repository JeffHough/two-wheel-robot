#ifndef JOYSTICK_TO_WHEELS_MOTOR_H_
#define JOYSTICK_TO_WHEELS_MOTOR_H_

// The maximum PWM value:
constexpr int PWM_MAX = 1024;

// A struct to define a single motor on the Rasp PI:
struct Motor
{
  const int PWM;
  const int ENABLE_FORWARD;
  const int ENABLE_BACKWARD;
};

#endif