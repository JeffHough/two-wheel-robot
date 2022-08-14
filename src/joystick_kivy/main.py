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
    

class JoystickLayout(BoxLayout):

  outer_radius = NumericProperty()
  switch_on = BooleanProperty()

  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    Clock.schedule_once(self.set_scene)

  def set_scene(self, *args):
    # Set the radius of the circle containing the joystick:
    self.outer_radius = self.ids.joystick.radius * 2.5

    # Center the joystick:
    self.ids.joystick.pos_x = self.center_x - self.ids.joystick.radius/2
    self.ids.joystick.pos_y = self.center_y - self.ids.joystick.radius/2

  def on_touch_up(self, touch):
    self.ids.joystick.pos_x = self.center_x - self.ids.joystick.radius/2
    self.ids.joystick.pos_y = self.center_y - self.ids.joystick.radius/2

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

    if joystick_distance > max_radius:
      self.ids.joystick.pos_x = int(dx / joystick_distance * max_radius) + self.center_x - self.ids.joystick.radius/2
      self.ids.joystick.pos_y = int(dy / joystick_distance * max_radius) + self.center_y - self.ids.joystick.radius/2

    else:
      self.ids.joystick.pos_x = dx + self.center_x - self.ids.joystick.radius/2
      self.ids.joystick.pos_y = dy + self.center_y - self.ids.joystick.radius/2


class JoystickApp(App):
  pass

if __name__=="__main__":
  JoystickApp().run()