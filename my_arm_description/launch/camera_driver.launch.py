#!/usr/bin/env python3
"""Camera driver launch (default: v4l2_camera)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_v4l2 = LaunchConfiguration('use_v4l2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    v4l2_node = Node(
        package=LaunchConfiguration('camera_pkg'),
        executable=LaunchConfiguration('camera_exe'),
        name=LaunchConfiguration('camera_name'),
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [
                LaunchConfiguration('image_width'),
                LaunchConfiguration('image_height'),
            ],
            'pixel_format': LaunchConfiguration('pixel_format'),
            'framerate': ParameterValue(LaunchConfiguration('framerate'), value_type=float),
            'use_sim_time': use_sim_time_param,
        }],
        output='screen',
        condition=IfCondition(use_v4l2),
    )

    external_launch = LaunchConfiguration('external_camera_launch')
    include_external = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(external_launch),
        condition=IfCondition(PythonExpression(["'", external_launch, "' != ''"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_v4l2', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_pkg', default_value='v4l2_camera'),
        DeclareLaunchArgument('camera_exe', default_value='v4l2_camera_node'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('external_camera_launch', default_value=''),
        v4l2_node,
        include_external,
    ])
