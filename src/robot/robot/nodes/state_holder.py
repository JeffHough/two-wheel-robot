#!/usr/bin/env python3
import threading

# ros-related:
from example_interfaces.srv import Trigger
from example_interfaces.msg import Bool
import rclpy
from rclpy.node import Node

class StateHolder(Node):
  def __init__(self):
    # initialize node, name it 'state_holder'
    super().__init__("state_holder")

    # Create a service to toggle the camera:
    self.create_service(Trigger, "toggle_camera", self.ToggleCamera)

    # Create a publisher of the current camera state:
    self.state_publisher_ = self.create_publisher(Bool, "camera_on", 1)

    # create a rate to publish the states every 1 Hz or so:
    self.rate_ = self.create_rate(1.0)

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

  def RunStatePublisher(self):
    while rclpy.ok():
      msg = Bool()
      msg.data = self.camera_on
      self.state_publisher_.publish(msg)

      self.rate_.sleep()
    

def main(args=None):

  # initialize the ros client:
  rclpy.init()
  state_holder = StateHolder()

  # spinning will block, but we don't do anything else:
  spinner = threading.Thread(target=rclpy.spin, args=(state_holder, ), daemon=True)
  spinner.start()

  state_holder.RunStatePublisher()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
