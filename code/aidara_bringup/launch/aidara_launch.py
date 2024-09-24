"""Launch file for a variety of aidara_packages.

llm_planner, vision_launch, hand_position, text_to_speech
and geometric_grasp.
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from aidara_common.types import RobotName


def generate_launch_description() -> LaunchDescription:
    """
    Launch a variety of aidara packages.

    Launches llm_planner, vision_launch, hand_position,
    tf2_service, text_to_speech, geometric_grasp.
    """
    llm = DeclareLaunchArgument(
        "llm",
        default_value="gpt-4",
        description="The LLM the planner uses.",
        choices=["gpt-4"],
    )
    prompt_version = DeclareLaunchArgument(
        "prompt_version",
        default_value="playground",
        description="The prompt version passed to the llm.",
        choices=["playground", "minigame"],
    )
    dry_run = DeclareLaunchArgument(
        "is_dry_run",
        default_value="False",
        choices=["True", "False"],
        description="If true, do not execute the generated code.",
    )
    robot = DeclareLaunchArgument(
        "robot",
        default_value=RobotName.PEPPER,
        choices=RobotName.get_valid_options(),
        description="The name of the used robot.",
    )

    def launch_llm_planner(ctx: LaunchContext) -> list[Node]:
        return [
            Node(
                package="llm_planning",
                executable="llm_planner",
                arguments=[
                    *(
                        ["--dry-run"]
                        if LaunchConfiguration("is_dry_run").perform(ctx) == "True"
                        else []
                    ),
                    "--llm",
                    LaunchConfiguration("llm"),
                    "--prompt-version",
                    LaunchConfiguration("prompt_version"),
                    "--robot",
                    LaunchConfiguration("robot"),
                ],
            ),
        ]

    description = [
        llm,
        prompt_version,
        dry_run,
        robot,
        OpaqueFunction(function=launch_llm_planner),
    ]
    return LaunchDescription(description)
