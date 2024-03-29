#!/usr/bin/env python3
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray

# import a specialized python serial communication library:
from pySerialTransfer import pySerialTransfer as txfer


USB_PORT = "/dev/ttyACM0"  # Arduino Uno WiFi Rev2

# Imports
import serial
import json

class WriteToArduino(Node):
    def __init__(self):
        super().__init__("write_to_arduino")

        # The usb port for writing to the arduino:
        self.usb = txfer.SerialTransfer(USB_PORT)
        link_open = False
        
        # Open the usb link:
        while not link_open:
            try:
                self.usb.open()
                link_open = True
            except:
                self.usb.close()

        # The subscription to listen to the wheels speeds to write to the arduino:
        self.create_subscription(Float32MultiArray, "/wheel_spds", self.WriteToArduino, 1)

    def WriteToArduino(self, msg):
        send_size = 0
        # grab the two motor speeds from the subscription:
        motor_speeds = [msg.data[0], msg.data[1]]
        list_size = self.usb.tx_obj(motor_speeds)
        send_size += list_size

        # serial write these to the arduino:
        self.usb.send(send_size)

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
