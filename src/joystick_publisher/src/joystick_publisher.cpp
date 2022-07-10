#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

int main(int argc, char **argv)
{

  // instantiate the ros2 client:
  rclcpp::init(argc, argv);

  // create options for our node:
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // create a new node:
  auto node = std::make_shared<rclcpp::Node>("publisher", node_options);

  // spin up the ros interface to the robot:
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec]()
              { exec.spin(); })
      .detach();

  // Run the robot interface:
  auto publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("/joystick", 1);

  // Publish something to the node:
  std_msgs::msg::Float32MultiArray msg;

  // Create a rate object:
  auto rate = std::make_shared<rclcpp::Rate>(5);

  for (int i = 0; i < 100; ++i)
  {
    float r = 0.5;
    float theta = i / 100.0 * M_PI * 2;
    msg.data = {r, theta};
    publisher->publish(msg);

    rate->sleep();
  }

  // shutdown ROS:
  rclcpp::shutdown();
}
