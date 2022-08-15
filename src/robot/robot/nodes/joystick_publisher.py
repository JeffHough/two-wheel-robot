#!/usr/bin/env python3
import json
import socket
import threading

# ros-related:
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node

class JoystickListener(Node):
    def __init__(self, pi_ip_addr, pi_port, buffer_size, rate, topic_name):
        # initialize node, name it 'joystick_listener'
        super().__init__("joystick_listener")

        # initialize the joystick server:
        self.pi_addr_port = (pi_ip_addr, pi_port)
        self.buffer = buffer_size
        self.web_server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        # create a publisher for the joystick data:
        self.joystick_publisher = self.create_publisher(Float32MultiArray, topic_name, 1)
        self.joystick_msg = Float32MultiArray()

        # create a rate object for publishing speed:
        self.rate = self.create_rate(rate)

        # initialize the joystick_msg:
        self.joystick_msg.data = [0.0, 0.0]

    def publish_joystick(self):
        while (rclpy.ok()):
            # publish the most recent msg:
            self.joystick_publisher.publish(self.joystick_msg)

            # sleep:
            self.rate.sleep()

    def listen_joystick(self):
        while (rclpy.ok()):
            # listen on the web server for joystick updates from the gui:
            msg = self.web_server.recv(self.buffer)
            data = json.loads(msg)
            self.joystick_msg.data = [data['r'], data['theta']]

def main(args=None):

    TOPIC_NAME = "/joystick"
    RATE = 10

    # some constants:
    PI_IP_ADDR = "192.168.2.69"
    PI_PORT = 20001
    BUFFER_SIZE = 1024

    # initialize the ros client:
    rclpy.init()

    joystick_listener = JoystickListener(PI_IP_ADDR, PI_PORT, BUFFER_SIZE, RATE, TOPIC_NAME)
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(joystick_listener,), daemon=True)
    spin_thread.start()
    joystick_listener.listen_joystick()
    joystick_listener.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()