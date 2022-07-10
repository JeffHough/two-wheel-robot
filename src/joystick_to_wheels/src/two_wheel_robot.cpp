#include <math.h>
#include "../include/joystick_to_wheels/two_wheel_robot.h"
#include <wiringPi.h>
#include <iostream>

TwoWheelRobot::TwoWheelRobot(
    const Motor &motor_a,
    const Motor &motor_b)
    : motor_a_{motor_a}, motor_b_{motor_b}
{
  // Call the wiring pi setup function:
  wiringPiSetup();

  // Add the motors to the vector of motors:
  motors_.push_back(motor_a_);
  motors_.push_back(motor_b_);

  // Configure each pin on the raspberry pi:
  for (const auto &motor : motors_)
  {
    pinMode(motor.PWM, PWM_OUTPUT);
    pinMode(motor.ENABLE_FORWARD, OUTPUT);
    pinMode(motor.ENABLE_BACKWARD, OUTPUT);
  }
};

double TwoWheelRobot::NomalizedRightWheelSpeed(const double &theta)
{
  // make sure we're in range of (0->2*pi):
  double new_theta = theta;
  if (theta < 0)
  {
    new_theta += 2 * M_PI;
  }
  else if (theta >= 2 * M_PI)
  {
    new_theta -= 2 * M_PI;
  }

  // Solve for the speed based on the quadrant:
  if (new_theta < M_PI_2)
  {

    // Quadrant I
    double theta_prime = 2 * new_theta;
    return -cos(theta_prime);
  }
  else if (new_theta < M_PI)
  {

    // Quadrant II
    return 1.0;
  }
  else if (new_theta < 3 / 2 * M_PI)
  {

    // Quadrant III
    double theta_prime = 2 * (new_theta - M_PI);
    return cos(theta_prime);
  }
  else if (new_theta <= 2 * M_PI)
  {

    // Quadrant IV
    return -1.0;
  }
}

// The left wheel speed is the same as the right wheel, just
double TwoWheelRobot::NomalizedLeftWheelSpeed(const double &theta)
{
  return NomalizedRightWheelSpeed(theta + M_PI_2);
}

// Get the wheel speeds from the joystick:
Eigen::Vector2d TwoWheelRobot::WheelSpeedsFromJoystick(const Eigen::Vector2d &joystick)
{
  // get the r and theta:
  const double &r = joystick(0);
  const double &theta = joystick(1);

  // Get the [normalized] wheel speeds in a vector:
  Eigen::Vector2d spd = {NomalizedLeftWheelSpeed(theta), NomalizedRightWheelSpeed(theta)};

  // Use the "r" and "max_voltage" as scaling:
  spd = spd * r;

  // Return the speed vector:
  return spd;
}

void TwoWheelRobot::WriteMotors(const Eigen::Vector2d &joystick)
{
  // Convert the joystick to two different motor spds:
  Eigen::Vector2d spds = WheelSpeedsFromJoystick(joystick);

  // std::cout << "The motor speeds are:\n" << spds << std::endl;

  // For each motor:
  for (int i = 0; i < motors_.size(); ++i)
  {
    // Get our speed and our motor:
    const double &spd = spds(i);
    const Motor &motor = motors_[i];

    // Direction of the motor speed:
    if (spd > 0)
    {
      // Write the positive direction:
      digitalWrite(motor.ENABLE_FORWARD, HIGH);
      digitalWrite(motor.ENABLE_BACKWARD, LOW);
    }
    else
    {
      // Write the negative direction:
      digitalWrite(motor.ENABLE_FORWARD, LOW);
      digitalWrite(motor.ENABLE_BACKWARD, HIGH);
    }

    // Write the PWM signal of the motor:
    pwmWrite(
        motor.PWM,
        abs(spd) * PWM_MAX);
  }
}
