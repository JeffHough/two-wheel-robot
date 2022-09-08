#!/usr/bin/env python3
from multiprocessing.util import get_logger
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

    # Message that we are ready
    self.get_logger().info("Ready to toggle the states!")

  def ToggleCamera(self, req, res):
    try:
      self.camera_on = not self.camera_on
      res.success = True
      res.message = f"Camera on?: {str(self.camera_on)}"
    except:
      res.success = False
      res.message = "Failed for some reason :("

    return res
    

def main(args=None):

  # initialize the ros client:
  rclpy.init()
  state_holder = StateHolder()

  # spinning will block, but we don't do anything else:
  rclpy.spin(state_holder)
  rclpy.shutdown()


if __name__ == "__main__":
  main()
