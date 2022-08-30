#include <math.h>
#include "../include/joystick_to_wheels/two_wheel_robot.h"

double TwoWheelRobot::NomalizedRightWheelSpeed(const double &theta)
{
  // make sure we're in range of (0->2*pi):
  double new_theta = theta;
  if (new_theta < 0)
  {
    new_theta += (2 * M_PI);
  }
  else if (new_theta > 2 * M_PI)
  {
    new_theta -= (2 * M_PI);
  }

  // Solve for the speed based on the quadrant:
  if (new_theta < M_PI/2)
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
  else if (new_theta < 1.50 * M_PI)
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
  throw(std::runtime_error("Not a valid theta?"));
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
  Eigen::Vector2d direction = {NomalizedLeftWheelSpeed(theta), NomalizedRightWheelSpeed(theta)};

  // Use the "r" as scaling:
  auto spd = direction * r;

  // Return the speed vector:
  return spd;
}