#!/usr/bin/env python3
import threading

# ros-related:
from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node

class StateHolder(Node):
  def __init__(self):
    # initialize node, name it 'state_holder'
    super().__init__("state_holder")

    # Create a service to toggle the camera:
    self.create_service(Trigger, "toggle_camera", self.ToggleCamera)

    # initialize the states:
    self.camera_on = False

  def ToggleCamera(self, req, res):
    try:
      self.camera_on = not self.camera_on
      res.success = True
    except:
      res.success = False
      res.message = "Failed for some reason :("


def main(args=None):

  TOPIC_NAME = "/joystick"
  RATE = 2

  # some constants:
  PI_IP_ADDR = "127.0.0.1"
  PI_PORT = 20001
  BUFFER_SIZE = 1024

  # initialize the ros client:
  rclpy.init()

  joystick_listener = JoystickListener(PI_IP_ADDR, PI_PORT, BUFFER_SIZE, RATE, TOPIC_NAME)

  # Need to spin up the node if we want the rate to work!!
  spinner = threading.Thread(target=rclpy.spin, args=(joystick_listener,), daemon=True)
  spinner.start()

  listen_thread = threading.Thread(target=joystick_listener.listen_controller, args=(), daemon=True)
  listen_thread.start()

  joystick_listener.publish_joystick()
  joystick_listener.destroy_node()
  rclpy.shutdown()
  listen_thread.join()
  spinner.join()


if __name__ == "__main__":
  main()
