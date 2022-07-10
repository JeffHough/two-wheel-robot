#ifndef JOYSTICK_TO_WHEELS_CONFIG_PINS_H_
#define JOYSTICK_TO_WHEELS_CONFIG_PINS_H_

#include "../include/joystick_to_wheels/motor.h"

// These are the pins I am selecting for control of two wheels.

namespace defaults
{
  // PWM pins for each motor (A and B):
  inline constexpr int PWM_A = 1;
  inline constexpr int PWM_B = 24;

  // Enables for motor A:
  inline constexpr int ENABLE_FORWARD_A = 21;
  inline constexpr int ENABLE_BACKWARD_A = 22;

  // Enables for motor B:
  inline constexpr int ENABLE_FORWARD_B = 27;
  inline constexpr int ENABLE_BACKWARD_B = 28;

  // Function to return motor A:
  inline Motor MotorA()
  {
    Motor A{
        .PWM = PWM_A,
        .ENABLE_FORWARD = ENABLE_FORWARD_A,
        .ENABLE_BACKWARD = ENABLE_BACKWARD_A};

    return A;
  }

  // Function to return motor B:
  // Function to return motor A:
  inline Motor MotorB()
  {
    Motor B{
        .PWM = PWM_B,
        .ENABLE_FORWARD = ENABLE_FORWARD_B,
        .ENABLE_BACKWARD = ENABLE_BACKWARD_B};

    return B;
  }
}

#endif
