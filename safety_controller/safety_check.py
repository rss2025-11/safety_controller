"""
List of scenarios in which to stop:
- wall distance < 0.5 according to wall following
- wall distance < 0.5 from front -45 to 45 degrees

"""


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("scan_topic", "default")

        self.laser_scan_sub = self.create_subscription(LaserScan,
                                                     self.SCAN_TOPIC,
                                                     self.listener_callback,
                                                     10)
        self.acker_sub = self.create_subscription(AckermannDriveStamped,
                                                  self.DRIVE_TOPIC, 
                                                  self.listener_callback,
                                                  10)
        self.safety_command = self.create_publisher(AckermannDriveStamped,
                                                    self.DRIVE_TOPIC,
                                                    10)
        

    def listener_callback(self, LaserScanMsg, AckerCmd):
        # manipulate laserscan message based on parameters
        ranges = np.array(LaserScanMsg.ranges)
        angle_min = LaserScanMsg.angle_min
        angle_max = LaserScanMsg.angle_max
        angle_increment = LaserScanMsg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)

        mask_infront = (angles > -np.pi/4) and (angles < np.pi/4)

        relavent_ranges = ranges[mask_infront]
        relavent_angles = angles[mask_infront]

        # find the equation of the wall's line
        # use some form of least squares...
        x = relavent_ranges*np.cos(relavent_angles)
        y = relavent_ranges*np.sin(relavent_angles)
        A = np.vstack([x, np.ones_like(x)]).T  # coefficient matrix
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]  # Solve Ax = b
        x_ls = x
        y_ls = m*x+b
        # Convert y = mx + b to Ax + By + C = 0 form
        A = -m
        B = 1
        C = -b
        x_r, y_r = 0, 0 # robot is center of coordinate system
        distance_from_wall = np.abs(A*x_r+B*y_r+C)/np.sqrt(A**2+B**2)

        if distance_from_wall < 0.5: # TODO: check value
            acker_cmd = AckermannDriveStamped()
            acker_cmd.header.stamp = self.get_clock().now().to_msg()
            acker_cmd.header.frame_id = 'map'
            acker_cmd.drive.steering_angle = 0.0
            acker_cmd.drive.steering_angle_velocity = 0.0
            acker_cmd.drive.speed = 0.0
            acker_cmd.drive.acceleration = 0.0 
            acker_cmd.drive.jerk = 0.0
            self.safety_command.publish(acker_cmd)