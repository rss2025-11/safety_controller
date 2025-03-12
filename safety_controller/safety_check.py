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
from std_msgs.msg import Float32
import math
from safety_controller.visualization_helper import SafetyVisualizer

class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("lookahead", 5.0)  # in seconds
        self.declare_parameter("buffer", 2.0)  # in meters

        self.SCAN_TOPIC = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.LOOKAHEAD = (
            self.get_parameter("lookahead").get_parameter_value().double_value
        )
        self.BUFFER = self.get_parameter("buffer").get_parameter_value().double_value

        self.laser_scan_sub = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.acker_sub = self.create_subscription(
            AckermannDriveStamped,
            "/vesc/low_level/ackermann_cmd",
            self.drive_callback,
            10,
        )
        self.safety_command = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )

        self.safety_publish = self.create_publisher(
            Float32, "/safety_controller/distance_to_front_wall", 10
        )

        self.current_speed = 0.0
        self.steering_angle = 0.0
        self.reverse_factor = 0.5

        # Initialize visualizer
        self.visualizer = SafetyVisualizer(self)
        
        # Define angle ranges for visualization
        self.ANGLE_FRONT_MARGIN = 0.174533  # 10 degrees in radians
        self.ANGLE_BACK_MARGIN = -0.174533  # -10 degrees in radians

    def scan_callback(self, LaserScanMsg):
        ranges = np.array(LaserScanMsg.ranges)
        angle_min = LaserScanMsg.angle_min
        angle_max = LaserScanMsg.angle_max
        angle_increment = LaserScanMsg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)

        # Visualize the view angle
        self.visualizer.visualize_detection_angle(
            self.ANGLE_FRONT_MARGIN, self.ANGLE_BACK_MARGIN, 1  # Using 1 since we look forward
        )

        # -10 to 10 degrees
        look_ahead = max(self.LOOKAHEAD, self.LOOKAHEAD * self.current_speed)
        # mask_infront = (angles > -0.174533) & (angles < 0.174533) & (ranges < look_ahead)

        # relevant_ranges = ranges[mask_infront]
        # relevant_angles = angles[mask_infront]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        side_distance = 0.2
        sides_filter = (abs(y) < side_distance)


        x = x[sides_filter]
        y = y[sides_filter]
        
        mask_infront = (0 < x) & (x < look_ahead)

        x = x[mask_infront]
        y = y[mask_infront]

        if len(x) >= 5:
            closest_points = np.sort(x)[:len(x) // 5]
            # avg_x = np.mean(closest_points)
            min_x = np.min(x)
            distance_msg = Float32()
            distance_msg.data = min_x#avg_x
            self.safety_publish.publish(distance_msg)

            # Visualize the average distance line and buffer line
            # self.visualizer.visualize_obstacle_line(avg_x, (-1.0, 1.0))
            self.visualizer.visualize_obstacle_line(min_x, (-1.0, 1.0))

            buffer = self.BUFFER + (math.floor(self.current_speed) - 1)*0.05
            self.visualizer.visualize_buffer_line(buffer, (-1.0, 1.0))
            # if avg_x < buffer:

            if min_x < buffer:
                acker_cmd = AckermannDriveStamped()
                acker_cmd.header.stamp = self.get_clock().now().to_msg()
                acker_cmd.header.frame_id = "map"
                acker_cmd.drive.steering_angle = 0.0
                acker_cmd.drive.steering_angle_velocity = 0.0
                acker_cmd.drive.speed = 0.0
                acker_cmd.drive.acceleration = 0.0
                acker_cmd.drive.jerk = 0.0
                self.safety_command.publish(acker_cmd)

    def drive_callback(self, AckerMsg):
        self.current_speed = AckerMsg.drive.speed
        self.steering_angle = AckerMsg.drive.steering_angle

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
