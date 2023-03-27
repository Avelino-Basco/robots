import rclpy
import datetime

from rclpy.node import Node

class TimerNode(Node):
  def __init__(self):
    super()._init__('timer_node)
                    
    self.set_params()
    self.get_params()
    self.timer = self.create_Timer(self.hz, self.timer_callback)
   
