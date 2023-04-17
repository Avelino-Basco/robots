import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class SensorsSubscriber(Node):
  def __init__(self):
    super().__init__('sensors_subscriber')       
    
    self.set_params()
    self.get_params()
    self.scan_ranges = []
    self.init_scan_state = False
    
    self.timer = self.create_timer(self.hz, self.timer_callback)
    self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile=qos_profile_sensor_data)
    self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
    self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

    self.counter = 4

  def set_params(self):
    self.declare_parameter('hz', 1)
    self.declare_parameter('imu_show', 1)
    self.declare_parameter('laserscan_show', 1)
    self.declare_parameter('battery_show', 1)

  def get_params(self):
    self.hz = self.get_parameter('hz').get_parameter_value().integer_value
    self.imu_show = self.get_parameter('imu_show').get_parameter_value().integer_value
    self.laserscan_show = self.get_parameter('laserscan_show').get_parameter_value().integer_value
    self.battery_show = self.get_parameter('battery_show').get_parameter_value().integer_value

  def timer_callback(self):
    self.get_logger().info(str(datetime.datetime.now()))
    if(self.counter == 4):
        self.counter = 1
    
  def imu_callback(self, msg):
    if(self.counter == 2 and self.imu_show == 1):
        self.get_logger().info('IMU: [ORIENTATION] (qo_x, qo_y, qo_z): (%f,%f,%f) | [ANGULAR VELOCITY] (av_x, av_y, av_z): (%f,%f,%f) | [LINEAR ACCELERATION] (la_x, la_y, la_z): (%f,%f,%f)' % (msg.orien>
        self.counter = self.counter + 1

  def scan_callback(self, msg):
    if(self.counter == 3 and self.laserscan_show == 1):
        self.get_logger().info('SCAN: min_range = %f | max_range = %f\n' %(min(value for value in msg.ranges if value != 0),max(msg.ranges)))
        self.counter = self.counter + 1

  def battery_callback(self, msg):
    if(self.counter == 1 and self.battery_show == 1):
        self.get_logger().info('BATTERY: %s | Voltage: %f | Temp: %f | Current: %f' %(msg.present, msg.voltage, msg.temperature, msg.current))
        self.counter = self.counter + 1


def main(args=None):
  rclpy.init(args=args)
  sensor_subscriber = SensorsSubscriber()
  rclpy.spin(sensor_subscriber)
  sensor_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
