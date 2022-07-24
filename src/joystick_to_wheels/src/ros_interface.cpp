#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include "../include/joystick_to_wheels/two_wheel_robot.h"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <Eigen/Dense>

constexpr double kRate = 10.0; // Hz
const std::string kTopicName = "/joystick";
constexpr int kQosProfile = 1;

using std::placeholders::_1;

// Create the interface to our robot:
class RobotInterace : public rclcpp::Node
{
protected:
  // The two wheel robot:
  TwoWheelRobot robot_;

  // A subscriber to the joystick topic:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joystick_sub_;

  // The internal joystick variable:
  Eigen::Vector2d joystick_value_;

  // The rate at which we will write the motor spds:
  rclcpp::Rate::SharedPtr rate_;

  // write the joystick values to the motors:
  void WriteMotors()
  {
    // write to the RPi using the robot_ object and the current robot object:
    robot_.WriteMotors(joystick_value_);
    // This is okay here too!
    // RCLCPP_INFO_STREAM(
    //   get_logger(),
    //   "Writing: r == " << joystick_value_(0) << ". theta == " << joystick_value_(1)
    // );
  }

  // setter of the joystick value:
  void SetJoystickValue(const std_msgs::msg::Float32MultiArray &msg)
  {
    joystick_value_(0) = msg.data[0];
    joystick_value_(1) = msg.data[1];

    // Sets properly!
    // RCLCPP_INFO_STREAM(
    //   get_logger(),
    //   "Setting: r == " << joystick_value_(0) << ". theta == " << joystick_value_(1)
    // );
  }

public:
  RobotInterace(const rclcpp::NodeOptions &node_options)
      : rclcpp::Node("two_wheel_robot", node_options), robot_{TwoWheelRobot()}
  {
    // Set the joystick value to zeros to begin with:
    joystick_value_ = Eigen::Vector2d::Zero();

    // Setup the subscriber with the callback setter function:
    joystick_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(kTopicName, 1, std::bind(&RobotInterace::SetJoystickValue, this, _1));

    // Create the rate object (which will control how quickly we run):
    rate_ = std::make_shared<rclcpp::Rate>(kRate);
  };

  void Run()
  {
    while (rclcpp::ok())
    {
      // write the new motor values:
      WriteMotors();

      // Sleep:
      rate_->sleep();
    }
  }
};

int main(int argc, char **argv)
{

  // instantiate the ros2 client:
  rclcpp::init(argc, argv);

  // create options for our node:
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // create a new node:
  auto robot_interface = std::make_shared<RobotInterace>(node_options);

  // spin up the ros interface to the robot:
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(robot_interface);
  std::thread([&exec]()
              { exec.spin(); })
      .detach();

  // Run the robot interface:
  robot_interface->Run();

  // shutdown ROS:
  rclcpp::shutdown();

  return 0;
}
