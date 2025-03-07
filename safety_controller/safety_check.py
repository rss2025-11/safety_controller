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

    def scan_callback(self, LaserScanMsg):
        ranges = np.array(LaserScanMsg.ranges)
        angle_min = LaserScanMsg.angle_min
        angle_max = LaserScanMsg.angle_max
        angle_increment = LaserScanMsg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_increment)

        # -20 to 20 degrees
        look_ahead = max(self.LOOKAHEAD, self.LOOKAHEAD * self.current_speed)
        mask_infront = (angles > -0.349) & (angles < 0.349) & (ranges < look_ahead)

        relevant_ranges = ranges[mask_infront]
        relevant_angles = angles[mask_infront]

        if len(relevant_ranges) != 0:
            x = relevant_ranges * np.cos(relevant_angles)

            avg_x = np.mean(x)
            distance_msg = Float32()
            distance_msg.data = avg_x
            self.safety_publish.publish(distance_msg)

            buffer = max(self.BUFFER, self.BUFFER * self.current_speed)
            if avg_x < buffer:
                self.get_logger().debug(f"STOPPED with avg x: {avg_x}")
                acker_cmd = AckermannDriveStamped()
                acker_cmd.header.stamp = self.get_clock().now().to_msg()
                acker_cmd.header.frame_id = "map"
                acker_cmd.drive.steering_angle = -1/4 * np.pi
                acker_cmd.drive.steering_angle_velocity = 0.0
                acker_cmd.drive.speed = 0.0
                acker_cmd.drive.acceleration = 0.0
                acker_cmd.drive.jerk = 0.0

                self.safety_command.publish(acker_cmd)
            else:
                self.get_logger().debug("failed due to avg x")
        else:
            self.get_logger().debug("failed due to no relevant ranges")

    def drive_callback(self, AckerMsg):
        current_speed = AckerMsg.drive.speed
        self.current_speed = current_speed


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
