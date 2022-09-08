from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.behaviors import DragBehavior
from kivy.graphics.vertex_instructions import Line, Rectangle, Ellipse
from kivy.core.window import Window
from kivy.properties import ObjectProperty, NumericProperty, BooleanProperty
from kivy.metrics import dp
from kivy.clock import Clock

# For vector math:
import numpy as np

# For web communication:
import socket
import json

# some constants:
PI_IP_ADDR = "127.0.0.1"
PI_PORT = 20001
BUFFER_SIZE = 1024

class JoystickSocket():
  def __init__(self, pi_ip_addr, pi_port, buffer_size=1024):
    # This is the data we will send:
    self.server_address_port = (pi_ip_addr, pi_port)
    self.buffer_size = buffer_size
    self.joystick = {'r': 0, 'theta': 0}
    self.udp_client = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

  def set_data(self, r, theta):
    self.joystick = {'r': r, 'theta': theta}

  def send_data(self):
    bytes_to_send = str.encode(json.dumps(self.joystick))
    self.udp_client.sendto(bytes_to_send, self.server_address_port)

# Combines the ellispe with draggable behaviour!
class JoystickCircle(DragBehavior, Widget):

  # properties

  # The current position
  pos_x = NumericProperty(50.0)
  pos_y = NumericProperty(150.0)

  # the center position
  center_x = NumericProperty()
  center_y = NumericProperty()

  radius = NumericProperty(dp(40))
    

class Application(BoxLayout):

  outer_radius = NumericProperty()
  switch_on = BooleanProperty()

  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    Clock.schedule_once(self.set_scene)

    # initialize the socket for the joystick:
    self.joystick_socket = JoystickSocket(PI_IP_ADDR, PI_PORT, BUFFER_SIZE)

    # schedule the data to be send every 0.1 seconds:
    Clock.schedule_interval(self.send_joystick_position, 0.1)

  def set_scene(self, *args):
    # Set the radius of the circle containing the joystick:
    self.outer_radius = self.ids.joystick.radius * 2.5

    # Center the joystick:
    self.ids.joystick.pos_x = self.center_x - self.ids.joystick.radius/2
    self.ids.joystick.pos_y = self.center_y - self.ids.joystick.radius/2

  def on_touch_up(self, touch):
    self.ids.joystick.pos_x = self.center_x - self.ids.joystick.radius/2
    self.ids.joystick.pos_y = self.center_y - self.ids.joystick.radius/2

    # reset the position to 0, 0 for the joystick:
    self.joystick_socket.set_data(0, 0)

  def switch_change_state(self, switch):
    self.switch_on = switch.active

  def on_touch_move(self, touch):

    if (not self.switch_on):
      self.on_touch_up(touch)
      return

    x = touch.x
    y = touch.y

    dx = x - self.center_x
    dy = y - self.center_y

    joystick_distance = np.sqrt( dx**2 + dy**2 )
    max_radius = self.outer_radius - self.ids.joystick.radius/2

    # set r and theta:
    r = joystick_distance/max_radius if (joystick_distance < max_radius) else 1
    theta = np.math.atan2(dy, dx)
    self.joystick_socket.set_data(r, theta)

    if joystick_distance > max_radius:
      self.ids.joystick.pos_x = int(dx / joystick_distance * max_radius) + self.center_x - self.ids.joystick.radius/2
      self.ids.joystick.pos_y = int(dy / joystick_distance * max_radius) + self.center_y - self.ids.joystick.radius/2
    else:
      self.ids.joystick.pos_x = dx + self.center_x - self.ids.joystick.radius/2
      self.ids.joystick.pos_y = dy + self.center_y - self.ids.joystick.radius/2

  def send_joystick_position(self, event):
    self.joystick_socket.send_data()

  def call_toggle_camera(self, event):
    print("Calling the camera toggle!")


class JoystickApp(App):
  pass

if __name__=="__main__":
  JoystickApp().run()