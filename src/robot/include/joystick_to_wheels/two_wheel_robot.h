#ifndef JOYSTICK_TO_WHEELS_H_
#define JOYSTICK_TO_WHEELS_H_

#include <Eigen/Dense>

// Define some common vectors:
namespace Eigen
{
  using Vector2d = Eigen::Vector<double, 2>;
  using Vector3d = Eigen::Vector<double, 3>;
  using Vector4d = Eigen::Vector<double, 4>;
  using Vector5d = Eigen::Vector<double, 5>;
};

class TwoWheelRobot
{
protected:

  // Gets just the right-wheel speed (the left-wheel is solved from the same function, time-shifted):
  double NomalizedRightWheelSpeed(const double &theta);
  double NomalizedLeftWheelSpeed(const double &theta);

public:
  // Constructor for the motor writer:
  TwoWheelRobot(){};

  // Takes in a single (r, theta) of the joystick. "r" will go from 0->1, theta from 0->2*pi.
  // Outputs a vector of two speeds, one for each wheel.
  Eigen::Vector2d WheelSpeedsFromJoystick(const Eigen::Vector2d &joystick);
};

#endif