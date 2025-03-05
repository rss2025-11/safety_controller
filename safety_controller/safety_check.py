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


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")


        self.laser_scan_sub = self.create_subscription(LaserScan,
                                                     "/scan",
                                                     self.scan_callback,
                                                     10)
        self.acker_sub = self.create_subscription(AckermannDriveStamped,
                                                  "/vesc/low_level/ackermann_cmd", 
                                                  self.drive_callback,
                                                  10)
        self.safety_command = self.create_publisher(AckermannDriveStamped,
                                                    "/vesc/low_level/input/safety",
                                                    10)
        
        self.current_speed = 0.0
        
        # self.line_pub = self.create_publisher(Marker, "/wall", 1)
    def scan_callback(self, LaserScanMsg):
        ranges = np.array(LaserScanMsg.ranges)
        angle_min = LaserScanMsg.angle_min
        angle_max = LaserScanMsg.angle_max
        angle_increment = LaserScanMsg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)
    
        # ignore distances more than 3 seconds away
        # -20 to 20 degrees
        mask_infront = (angles > -0.349) & (angles < 0.349) & (ranges < 5*self.current_speed)
        
        relavent_ranges = ranges[mask_infront]
        relavent_angles = angles[mask_infront]

        # find the equation of the wall's line
        # use some form of least squares...
        if len(relavent_ranges) != 0:
            x = relavent_ranges*np.cos(relavent_angles)
            
            avg_x = np.average(x)

            if avg_x < self.current_speed*2:
                acker_cmd = AckermannDriveStamped()
                acker_cmd.header.stamp = self.get_clock().now().to_msg()
                acker_cmd.header.frame_id = 'map'
                acker_cmd.drive.steering_angle = 0.0
                acker_cmd.drive.steering_angle_velocity = 0.0
                acker_cmd.drive.speed = 0.0
                acker_cmd.drive.acceleration = 0.0 
                acker_cmd.drive.jerk = 0.0
                self.get_logger().info(f'STOPPED with avg x: {avg_x}')
                self.safety_command.publish(acker_cmd)
        self.get_logger().info(f'length of x array: {len(relavent_ranges)}')

    def drive_callback(self, AckerMsg):
        current_speed = AckerMsg.drive.speed
        self.current_speed = current_speed



def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()