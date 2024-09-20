"""
ROS2 controller for the Stäubli TX2-60 robot.

This controller communicates with the Robot via sockets. It sends joint positions as
commands and receives the current joint positions and publishes them.
"""

import socket

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from aidara_common.tf_utils import TfMixin

HOST = "192.168.1.155"
PORT_READ = 1235
PORT_WRITE = 1237

FREQUENCY = 5
TIMER_PERIOD = 0.01
TIMEOUT = 40
TOLERANCE = 0.01

TIMEOUT_MSG = (
    "\n\n"
    "██     ██    █████    ██████    ███    ██   ██   ███    ██    ██████\n"
    "██     ██   ██   ██   ██   ██   ████   ██   ██   ████   ██   ██      \n"
    "██  █  ██   ███████   ██████    ██ ██  ██   ██   ██ ██  ██   ██   ███\n"
    "██ ███ ██   ██   ██   ██   ██   ██  ██ ██   ██   ██  ██ ██   ██    ██\n"
    " ███ ███    ██   ██   ██   ██   ██   ████   ██   ██   ████    ██████\n"
    "\n"
    "    ------------------------------------------------------------\n"
    "    ------------------------------------------------------------\n"
    "    || Eef did not reach goal. Restart the application on the ||\n"
    "    || Stäubli Controller.n                                   ||\n"
    "    || FAILURE TO COMPLY MAY LEAD TO DAMAGE TO THE ROBOT.     ||\n"
    "    ------------------------------------------------------------\n"
    "    ------------------------------------------------------------\n"
)


class StaubliController(Node, TfMixin):
    """
    ROS 2 Node for sending and reading joint positions of Stäubli robots.

    The controller can be called from an action client and sends trajectories to the
    robot. It also publishes the current joint states and the eef transform.
    """

    def __init__(self) -> None:
        """Initialize Stäubli_controller node."""
        Node.__init__(self, node_name="staubli_controller")
        TfMixin.__init__(self)

        # Establish connection with the robot via sockets
        self._read_setup()
        self._control_setup()


    def _read_setup(self) -> None:
        """Initialize socket for reading joint position."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT_READ))
            s.listen()
            self._conn_read, self._read_address = s.accept()
        self.get_logger().info(f"Connection for reading from: '{self._read_address}'.")

    def _control_setup(self) -> None:
        """Initialize socket for sending joint positions."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT_WRITE))
            s.listen()
            self._conn_control, address = s.accept()
        self.get_logger().info(f"Connection for commands from: '{address}'.")


    def _reset_socket_connections(self) -> None:
        """Close the existing connections and setup new sockets."""
        self._conn_read.close()
        self._read_setup()
        self._conn_control.close()
        self._control_setup()


def main(args: list[str] | None = None) -> None:
    """Spin Node."""
    rclpy.init(args=args)

    try:
        node = StaubliController()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    else:
        rclpy.shutdown()
