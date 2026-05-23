from launch import LaunchDescription
from launch_ros.actions import SetParameter

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my_arm", package_name="movelt_config").to_moveit_configs()
    launch_description = generate_demo_launch(moveit_config)
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        *launch_description.entities,
    ])

