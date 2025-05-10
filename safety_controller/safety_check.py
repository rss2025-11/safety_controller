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
from .safety_visualization import SafetyVisualizer


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety")
        self.declare_parameter(
            "control_listener_topic", "/vesc/low_level/ackermann_cmd"
        )

        # Hardcoded parameters
        self.SCAN_TOPIC = "/scan"
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").value
        self.CONTROL_LISTENER_TOPIC = self.get_parameter("control_listener_topic").value
        self.MIN_TTC_THRESHOLD_SEC = 0.55
        self.MAX_DETECTION_RANGE_M = 4.0
        self.CAR_FRONT_HALF_WIDTH = 0.13  # 16 cm is real value
        self.STEER_FACTOR = 0.75
        self.TRAPEZOID_FLARE_ANGLE_RAD = np.deg2rad(5)  # 10-degree flare on each side

        # ROS Subscribers/Publishers
        self.laser_scan_sub = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10
        )
        self.acker_sub = self.create_subscription(
            AckermannDriveStamped,
            self.CONTROL_LISTENER_TOPIC,
            self.drive_callback,
            10,
        )
        self.safety_command = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 10
        )

        # Instantiate the visualizer
        self.visualizer = SafetyVisualizer(
            self,
            max_detection_range=self.MAX_DETECTION_RANGE_M,
            car_front_half_width=self.CAR_FRONT_HALF_WIDTH,
            trapezoid_flare_angle_rad=self.TRAPEZOID_FLARE_ANGLE_RAD,
        )

        self.current_speed = 0.0
        self.current_steering_angle = 0.0

    def scan_callback(self, LaserScanMsg):
        # Early exit if not moving forward
        if self.current_speed <= 1e-3:
            # If visualizer exists and we want to clear/show empty when not active:
            # self.visualizer.update_visualization(LaserScanMsg.header.frame_id, 0.0, self.MAX_DETECTION_RANGE_M, self.CAR_FRONT_HALF_WIDTH, self.TRAPEZOID_FLARE_ANGLE_RAD, math.inf, False)
            return

        ranges = np.array(LaserScanMsg.ranges)
        angle_min_scan = LaserScanMsg.angle_min
        angle_max_scan = LaserScanMsg.angle_max

        # Create an array of angles for each lidar point, matching the ranges array
        angles = np.linspace(angle_min_scan, angle_max_scan, len(ranges))

        center_fov_angle = self.current_steering_angle * self.STEER_FACTOR

        # Initial filter for basic validity of ranges
        valid_range_mask = np.isfinite(ranges)
        valid_ranges = ranges[valid_range_mask]
        valid_angles_absolute = angles[valid_range_mask]

        if len(valid_ranges) == 0:
            # No valid points from scan at all, effectively same as no points in trapezoid
            effective_distance = float("inf")
        else:
            # Transform points to Cartesian relative to steered centerline
            angles_relative_to_center = valid_angles_absolute - center_fov_angle
            # Normalize angles to [-pi, pi]
            angles_relative_to_center = (angles_relative_to_center + np.pi) % (
                2 * np.pi
            ) - np.pi

            px_all = valid_ranges * np.sin(angles_relative_to_center)
            py_all = valid_ranges * np.cos(angles_relative_to_center)

            # Filter points within the trapezoid
            py_mask = (py_all >= 0) & (py_all <= self.MAX_DETECTION_RANGE_M)

            # Calculate lateral allowance at each py_all distance
            # Ensure py_all used here is only positive for tan calculation if flare angle is large
            # However, since py_mask already filters for py_all >=0, this should be fine.
            max_lateral_at_py = self.CAR_FRONT_HALF_WIDTH + py_all * np.tan(
                self.TRAPEZOID_FLARE_ANGLE_RAD
            )

            px_mask = (px_all >= -max_lateral_at_py) & (px_all <= max_lateral_at_py)

            final_trapezoid_mask = py_mask & px_mask
            x_forward_distances_in_trapezoid = py_all[final_trapezoid_mask]

            # Calculate effective_distance from points within the trapezoid
            if len(x_forward_distances_in_trapezoid) >= 3:
                num_to_average = len(x_forward_distances_in_trapezoid) // 3
                closest_subset = np.sort(x_forward_distances_in_trapezoid)[
                    :num_to_average
                ]
                effective_distance = np.mean(closest_subset)
            else:
                effective_distance = float("inf")

        should_stop = False
        reason_for_stop_consideration = "N/A"

        if effective_distance != float("inf"):
            current_ttc = effective_distance / self.current_speed
            if current_ttc < self.MIN_TTC_THRESHOLD_SEC:
                should_stop = True
                reason_for_stop_consideration = f"TTC {current_ttc:.2f}s < threshold {self.MIN_TTC_THRESHOLD_SEC:.2f}s (dist {effective_distance:.2f}m, speed {self.current_speed:.2f}m/s)"

        if should_stop:
            self.get_logger().info(
                f"STOPPING: Speed {self.current_speed:.2f}m/s. Reason: {reason_for_stop_consideration}."
            )
            acker_cmd = AckermannDriveStamped()
            acker_cmd.header.stamp = self.get_clock().now().to_msg()
            acker_cmd.header.frame_id = "map"
            acker_cmd.drive.steering_angle = 0.0
            acker_cmd.drive.steering_angle_velocity = 0.0
            acker_cmd.drive.speed = -0.7
            acker_cmd.drive.acceleration = 0.0
            acker_cmd.drive.jerk = 0.0
            self.safety_command.publish(acker_cmd)

        # Call the visualizer update method
        # Ensure to use math.inf when passing effective_distance if that's what visualizer expects
        # For now, assuming float('inf') is handled or visualizer uses math.inf internally
        self.visualizer.update_visualization(
            frame_id=LaserScanMsg.header.frame_id,
            center_fov_angle=center_fov_angle,  # This is self.current_steering_angle
            effective_distance=effective_distance,  # Pass the calculated effective_distance
            should_stop=should_stop,
        )

    def drive_callback(self, AckerMsg):
        self.current_speed = AckerMsg.drive.speed
        self.current_steering_angle = AckerMsg.drive.steering_angle


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
