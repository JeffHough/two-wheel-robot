#include <joystick_to_wheels/joystick_to_wheels.h>
#include <math.h>

double normalized_right_wheel_speed(double theta)
{
  // make sure we're in range of (0->2*pi):
  if (theta < 0){
    theta+=2*M_PI;
  } else if (theta >= 2*M_PI){
    theta-=2*M_PI;
  }

  // Solve for the speed based on the quadrant:
  if (theta < M_PI_2){

    // Quadrant I
    double theta_prime = 2*theta;
    return -cos(theta_prime);

  } else if (theta < M_PI){

    // Quadrant II
    return 1.0;

  } else if (theta < 3/2 * M_PI){

    // Quadrant III
    double theta_prime = 2*(theta-M_PI);
    return cos(theta_prime);

  } else if (theta <= 2*M_PI){

    // Quadrant IV
    return -1.0;
  }
}

// The left wheel speed is the same as the right wheel, just 
double normalized_left_wheel_speed(double theta)
{
  return normalized_right_wheel_speed(theta + M_PI_2);
}

// Get the wheel speeds from the joystick:
Eigen::Vector2d wheel_speeds_from_joystick(Eigen::Vector2d joystick, double max_voltage)
{
  // get the r and theta:
  double r = joystick(0);
  double theta = joystick(1);

  // Get the [normalized] wheel speeds in a vector:
  Eigen::Vector2d spd = {normalized_left_wheel_speed(theta), normalized_right_wheel_speed(theta)};

  // Use the "r" and "max_voltage" as scaling:
  spd = spd*r*max_voltage;

  // Return the speed vector:
  return spd;
}