"""Node carrying out top level actions for the LLM planner."""

from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from std_srvs.srv import Trigger

from aidara_common.node_utils import NodeMixin
from aidara_common.singleton import Singleton
from aidara_common.tf_utils import TfMixin
from aidara_msgs.srv import (
    GeometricGrasp,
    TargetPose,
)


class LLMActions(Node, TfMixin, NodeMixin, metaclass=Singleton):
    """Singleton for the performing LLM actions."""

    def __init__(self, config_file: str) -> None:
        """Initialize node for performing LLM actions."""
        Node.__init__(self, "llm_actions")
        TfMixin.__init__(self)
        NodeMixin.__init__(self)

        self._user_feedback_pub = self.create_publisher(String, "/user_feedback", 1)

        self._open_gripper_client = self.create_client(Trigger, "/open_gripper")
        self._close_gripper_client = self.create_client(Trigger, "/close_gripper")

        self._relax_client = self.create_client(
            Trigger,
            "/compliant_trajectory_controller/relax",
        )
        self._tighten_up_client = self.create_client(
            Trigger,
            "/compliant_trajectory_controller/tighten_up",
        )

        self._move_eef_client = self.create_client(TargetPose, "/move_eef")

        self._geometric_grasp_client = self.create_client(
            GeometricGrasp,
            "/geometric_grasp",
        )

    @property
    def user_feedback_pub(self) -> Publisher:
        """Publisher to the '/user_feedback' topic."""
        return self._user_feedback_pub

    @property
    def open_gripper_client(self) -> Client:
        """Client for the '/open_gripper' service."""
        return self._open_gripper_client

    @property
    def close_gripper_client(self) -> Client:
        """Client for the '/close_gripper' service."""
        return self._close_gripper_client

    @property
    def relax_client(self) -> Client:
        """Client for the '/relax' service."""
        return self._relax_client

    @property
    def tighten_up_client(self) -> Client:
        """Client for the '/tighten_up' service."""
        return self._tighten_up_client

    @property
    def move_eef_client(self) -> Client:
        """Client for the '/move_eef_to' service."""
        return self._move_eef_client

    @property
    def geometric_grasp_client(self) -> Client:
        """Client for the '/geometric_grasp' service."""
        return self._geometric_grasp_client
