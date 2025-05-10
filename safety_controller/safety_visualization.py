import rclpy
from rclpy.node import Node  # Though parent node is passed, good for type hinting
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
import math


class SafetyVisualizer:
    def __init__(
        self,
        parent_node: Node,
        max_detection_range: float,
        car_front_half_width: float,
        trapezoid_flare_angle_rad: float,
    ):
        """
        Initializes the SafetyVisualizer.
        Args:
            parent_node: The ROS 2 node that instantiates this visualizer (e.g., SafetyController).
                         Used for getting the clock and logger.
            max_detection_range: Max forward distance of the trapezoid.
            car_front_half_width: Half width of the trapezoid base.
            trapezoid_flare_angle_rad: Flare angle of the trapezoid sides.
        """
        self.parent_node = parent_node
        self.marker_publisher = self.parent_node.create_publisher(
            MarkerArray, "/safety_controller/visualization_markers", 10  # QoS depth
        )
        self._marker_id_counter = 0

        self.max_detection_range = max_detection_range
        self.car_front_half_width = car_front_half_width
        self.trapezoid_flare_angle_rad = trapezoid_flare_angle_rad

    def _create_new_marker_id(self) -> int:
        current_id = self._marker_id_counter
        self._marker_id_counter += 1
        return current_id

    def clear_markers(self, frame_id: str):
        """Publishes a MarkerArray to delete all markers."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = self.parent_node.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = "safety_zone"  # Ensure we only delete our own namespace
        marker_array.markers.append(delete_marker)
        self.marker_publisher.publish(marker_array)
        self._marker_id_counter = 0  # Reset counter after clearing

    def update_visualization(
        self,
        frame_id: str,
        center_fov_angle: float,
        effective_distance: float,
        should_stop: bool,
    ):
        """
        Updates and publishes the visualization markers.
        Args:
            frame_id: The coordinate frame for the markers (e.g., from LaserScan header).
            center_fov_angle: Current steering angle, used for rotating the trapezoid.
            effective_distance: Calculated distance to the closest relevant obstacle.
            should_stop: Boolean indicating if a stop is triggered.
        """

        # Ensure we clear previous markers in the same frame_id
        self.clear_markers(frame_id)

        marker_array = MarkerArray()

        # --- 1. Trapezoid Marker ---
        trapezoid_marker = Marker()
        trapezoid_marker.header.frame_id = frame_id
        trapezoid_marker.header.stamp = self.parent_node.get_clock().now().to_msg()
        trapezoid_marker.ns = "safety_zone"
        trapezoid_marker.id = self._create_new_marker_id()
        trapezoid_marker.type = Marker.LINE_STRIP
        trapezoid_marker.action = Marker.ADD

        q = Quaternion()  # Create Quaternion message instance
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(center_fov_angle / 2.0)
        q.w = math.cos(center_fov_angle / 2.0)
        trapezoid_marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        trapezoid_marker.pose.orientation = q

        trapezoid_marker.scale.x = 0.05  # Line width

        if should_stop:
            trapezoid_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
        else:
            trapezoid_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)  # Green

        # Points for the trapezoid (in the "steered" frame, Y forward, X left)
        # The lidar/origin of the trapezoid is assumed to be at (0,0) in this steered frame.
        # The diagram shows lidar (purple dot) at the base of the trapezoid.
        # X positive is to the left, X negative is to the right when looking forward.

        # Point 1: Near-left
        p1 = Point(x=0.0, y=self.car_front_half_width, z=0.0)

        # Point 2: Far-left
        p2 = Point(
            x=self.max_detection_range,
            y=self.car_front_half_width
            + self.max_detection_range * math.tan(self.trapezoid_flare_angle_rad),
            z=0.0,
        )

        # Point 3: Far-right
        p3 = Point(
            x=self.max_detection_range,
            y=-(
                self.car_front_half_width
                + self.max_detection_range * math.tan(self.trapezoid_flare_angle_rad)
            ),
            z=0.0,
        )

        # Point 4: Near-right
        p4 = Point(x=0.0, y=-self.car_front_half_width, z=0.0)

        trapezoid_marker.points = [p1, p2, p3, p4, p1]  # Close the loop
        marker_array.markers.append(trapezoid_marker)

        # --- 2. Effective Distance Marker (Optional) ---
        if (
            effective_distance != math.inf
            and effective_distance < self.max_detection_range
        ):
            eff_dist_marker = Marker()
            eff_dist_marker.header.frame_id = frame_id
            eff_dist_marker.header.stamp = self.parent_node.get_clock().now().to_msg()
            eff_dist_marker.ns = "safety_zone"
            eff_dist_marker.id = self._create_new_marker_id()
            eff_dist_marker.type = Marker.CYLINDER  # A small disc on the ground
            eff_dist_marker.action = Marker.ADD

            # Calculate position in the frame_id based on steered angle
            pos_x = effective_distance * math.cos(center_fov_angle)
            pos_y = effective_distance * math.sin(center_fov_angle)

            eff_dist_marker.pose.position = Point(
                x=pos_x, y=pos_y, z=0.01
            )  # z=0.01 is a small ground offset
            eff_dist_marker.pose.orientation = (
                q  # Use the same rotation as trapezoid for the cylinder itself
            )
            eff_dist_marker.scale = Vector3(x=0.2, y=0.2, z=0.02)  # Small disc
            eff_dist_marker.color = ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.9)  # Orange
            marker_array.markers.append(eff_dist_marker)

        self.marker_publisher.publish(marker_array)


if __name__ == "__main__":
    pass
