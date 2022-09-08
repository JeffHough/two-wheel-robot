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
        self.web_server.bind(self.pi_addr_port)

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
        # listen on the web server for joystick updates from the gui:
        while (True):
            msg_addr = self.web_server.recvfrom(self.buffer)
            data = json.loads(msg_addr[0])
            self.joystick_msg.data = [float(data['r']), float(data['theta'])]

def main(args=None):

    TOPIC_NAME = "/joystick"
    RATE = 10

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

    listen_thread = threading.Thread(target=joystick_listener.listen_joystick, args=(), daemon=True)
    listen_thread.start()

    joystick_listener.publish_joystick()
    joystick_listener.destroy_node()
    rclpy.shutdown()
    listen_thread.join()
    spinner.join()


if __name__ == "__main__":
    main()
