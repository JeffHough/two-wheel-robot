#!/usr/bin/env python3
import json
import socket
import threading
from time import sleep

# ros-related:
from std_msgs.msg import Float32MultiArray
from example_interfaces.srv import Trigger
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

        # create a client to toggle the camera on and off:
        self.toggle_camera_client = self.create_client(Trigger, "toggle_camera")

        while not self.toggle_camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def publish_joystick(self):
        while (rclpy.ok()):
            # publish the most recent msg:
            self.joystick_publisher.publish(self.joystick_msg)

            # sleep:
            self.rate.sleep()

    def listen_controller(self):
        # listen on the web server for joystick updates from the gui:
        while (True):
            msg_addr = self.web_server.recvfrom(self.buffer)
            try:
                data = json.loads(msg_addr[0])
                self.joystick_msg.data = [float(data['r']), float(data['theta'])]
            except:
                data = msg_addr[0].decode()
                self.get_logger().info(f"data: {data}")
                if data == "toggle_camera":
                    # # create a "trigger" service to toggle the camera mode
                    trigger = Trigger.Request()
                    self.future = self.toggle_camera_client.call_async(trigger)
                    while not self.future.done():
                        # definitely not the best way to do this.
                        sleep(0.001)
                    self.get_logger().info(self.future.result().message)
                    

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