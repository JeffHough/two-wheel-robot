#ifndef JOYSTICK_TO_WHEELS_H_
#define JOYSTICK_TO_WHEELS_H_

#include <Eigen/Dense>
#include <vector>
#include "motor.h"
#include "../../config/defaults.h"

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
  // Two motors:
  const Motor motor_a_;
  const Motor motor_b_;

  // A standard vector of motors:
  std::vector<Motor> motors_;

  // Gets just the right-wheel speed (the left-wheel is solved from the same function, time-shifted):
  double NomalizedRightWheelSpeed(const double &theta);
  double NomalizedLeftWheelSpeed(const double &theta);

  // Takes in a single (r, theta) of the joystick. "r" will go from 0->1, theta from 0->2*pi.
  // Outputs a vector of two speeds, one for each wheel.
  Eigen::Vector2d WheelSpeedsFromJoystick(const Eigen::Vector2d &joystick);

public:
  // Constructor for the motor writer (need a pwm pin and two enable pins):
  TwoWheelRobot(
      const Motor &motor_a,
      const Motor &motor_b);

  // Default constructor uses the default configuration:
  TwoWheelRobot() : TwoWheelRobot(defaults::MotorA(), defaults::MotorB()){};

  // To write to a single motor:
  void WriteMotors(
      const Eigen::Vector2d &joystick // A number from 0 to 1, driving the speed of the motor.
  );
};

#endif