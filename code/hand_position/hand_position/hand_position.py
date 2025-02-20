"""ROS 2 Node for hand position tracking."""

import argparse

import numpy as np
import numpy.typing as npt
import rclpy
import ros2_numpy as rnp
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from termcolor import colored
from zed_interfaces.msg import Object, ObjectsStamped

WRIST_TO_PALM_DISTANCE = 0.08
EXPECTED_KEYPOINTS_BODY_FORMAT = 2  # 38 keypoints
MAGIC_BODYTRK_ROTATION = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])


class HandTracking(Node):
    """ROS 2 Node to calculate the position of the right hand."""

    def __init__(self, zed_camera_name: str) -> None:
        """Initialize hand tracking node."""
        super().__init__("hand_tracking")

        self._skeleton_sub = self.create_subscription(
            ObjectsStamped,
            f"{zed_camera_name}/zed_node/body_trk/skeletons",
            self._objects_callback,
            10,
        )
        self._position_pub = self.create_publisher(PointStamped, "hand_position", 10)

    def _objects_callback(self, objects: ObjectsStamped) -> None:
        """Publish the position of the first visible human right hand."""
        obj: Object
        header = objects.header
        for obj in objects.objects:
            if (
                not obj.skeleton_available
                or obj.body_format != EXPECTED_KEYPOINTS_BODY_FORMAT
            ):
                continue

            right_wrist, right_elbow = self._get_hand_coordinates(obj)

            if self._is_invalid(right_wrist) or self._is_invalid(right_elbow):
                continue

            position_msg = self._calculate_hand_position(
                right_wrist,
                right_elbow,
            )
            msg = PointStamped()
            msg.point = position_msg
            msg.header = header

            self._position_pub.publish(msg)

            return

        self.get_logger().warning(
            colored("No human in frame. Could not calculate hand position.", "yellow"),
            throttle_duration_sec=2,
        )

    def _get_hand_coordinates(self, obj: Object) -> tuple[npt.NDArray, npt.NDArray]:
        keypoints = obj.skeleton_3d.keypoints

        right_wrist = np.array(keypoints[17].kp, dtype=np.float64)
        right_elbow = np.array(keypoints[15].kp, dtype=np.float64)

        return right_wrist, right_elbow

    def _calculate_hand_position(
        self,
        wrist: npt.NDArray,
        elbow: npt.NDArray,
    ) -> Point:
        direction_vec = wrist - elbow

        direction_vec /= np.linalg.norm(
            direction_vec,
        )

        position_vector = wrist + WRIST_TO_PALM_DISTANCE * direction_vec

        # Only works like this, sorry...
        position_vector = MAGIC_BODYTRK_ROTATION @ position_vector

        return rnp.msgify(Point, position_vector)

    def _is_invalid(self, keypoint: npt.NDArray) -> bool:
        return np.isnan(keypoint).any() or (keypoint == 0.0).any()


def main(args: list[str] | None = None) -> None:
    """Initialize and run the hand tracking node."""
    parser = argparse.ArgumentParser(
        description="Tracker for right palm position.",
    )
    parser.add_argument(
        "--zed-camera-name",
        type=str,
        default="zed",
    )
    parser.add_argument(
        "--ros-args",
        action="store_true",
        help="Ignored. There for ros2 launch compatibility.",
    )
    camera_name = parser.parse_args(args).zed_camera_name

    rclpy.init(args=args)

    hand_tracking = HandTracking(camera_name)
    rclpy.spin(hand_tracking)
