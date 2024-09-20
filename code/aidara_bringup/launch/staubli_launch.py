"""Launch file for staubli trajectory planner, speech_to_text, static transforms."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch controller and planner."""
    return LaunchDescription(
        [
            Node(
                package="staubli_controller",
                executable="controller",
                name="controller",
                emulate_tty=True,
            ),
            Node(
                package="speech_to_text",
                executable="speech_to_text",
            ),
            Node(
                package="text_to_speech",
                executable="text_to_speech",
            ),
        ],
    )
