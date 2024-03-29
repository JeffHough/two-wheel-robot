cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)

# install the python modules:
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  robot/nodes/joystick_listener.py
  robot/nodes/serial_write.py
  DESTINATION lib/${PROJECT_NAME}
)

# create the target:
add_executable(ros_interface
  src/ros_interface.cpp
  src/two_wheel_robot.cpp
)

target_link_libraries(ros_interface Eigen3::Eigen)

# add the dependencies (MUST INCLUDE RCLCPP):
ament_target_dependencies(
  ros_interface
  rclcpp
  std_msgs
)

# the install:
install(TARGETS
  ros_interface
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch:
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
