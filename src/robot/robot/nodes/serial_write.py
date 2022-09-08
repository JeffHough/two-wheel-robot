#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray
from example_interfaces.msg import Bool

# USB_PORT = "/dev/ttyACM0"  # Arduino Uno WiFi Rev2
# BAUD_RATE = 9600

# # should be done about 2x the write speed
# TIMEOUT = 0.05

# Imports
import json

class WriteToArduino(Node):
    def __init__(self):
        super().__init__("write_to_arduino")
        
        # The subscription to listen to the wheels speeds to write to the arduino:
        self.create_subscription(Float32MultiArray, "/wheel_spds", self.WriteToArduino, 1)

        # The subscription to listen to the camera being on or off:
        self.create_subscription(Bool, "/camera_on", self.SetStates, 1)

        # The motor speeds we want to write to the arduino:
        self.arduino_data = {"motor_A": 0.0, "motor_B": 0.0, "camera_on": False}

        # Assume the camera starts OFF:
        self.camera_on = False

    def WriteToArduino(self, msg):
        # grab the two motor speeds from the subscription:
        self.arduino_data["motor_A"] = msg.data[0]
        self.arduino_data["motor_B"] = msg.data[1]
        self.arduino_data["camera_on"] = self.camera_on

        # # write these to the arduino:
        # self.usb.write(json.dumps(self.motors).encode())
        self.get_logger().info("We would be writing the values: " + str(self.arduino_data))

    def SetStates(self, msg):
        # update the internal state:
        self.camera_on = msg.data

def main(args=None):
    # initialize the ros client:
    rclpy.init()

    # initialize:
    write_to_arduino_node = WriteToArduino()
    
    # launch the subscriber (this is a blocking call):
    rclpy.spin(write_to_arduino_node)

    # shut down (we will never get here - crash landing always)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
