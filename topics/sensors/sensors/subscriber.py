import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, Batterystate
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class SensorsSubscriber(Node):
  def __init__(self):
    super().__init__('sensors_subcriber')       //calls constructor to create SensorsSubscriber node with name "sensors_subscriber"
    
    self.set_params()
    self.get_params()
    self.scan_ranges = []
    self.init_scan_state = False
    
    self.timer = self.create_timer(self.timer_period, self.timer_callback)
    self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile=qos_profile_sensor_data)
    self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
    self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
    self.imu_count = 0

  def imu_callback(self, msg):
    self.get_logger().info('IMU: (qo_x, qo_y, qo_z): (%f,%f,%f) |' % (msg.orientation_covariance.x,msg.orientation_covariance.y,msg.orientation_covariance.z))
    self.get_logger().info(' (av_x, av_y, av_z): (%f,%f,%f) |' % (msg.angular_velocity_covariance.x,msg.angular_velocity_covariance.y,msg.angular_velocity_covariance.z))
    self.get_logger().info(' (la_x, la_y, la_z): (%f,%f,%f)\n' % (msg.linear_acceleration_covariance.x,msg.linear_acceleration_covariance.y,msg.linear_acceleration_covariance.z))

  def scan_callback(self, msg):
    self.get_logger().info('SCAN: max_range = %f | min_range = %f\n' %(msg.range_max,msg.range_min))
    
  def battery_callback(self, msg):
    self.get_logger().info('BATTERY: %s | Voltage: %f | Temp: %f | Current: %f' %(msg.present, msg.voltage, msg.temperature, msg.current))
