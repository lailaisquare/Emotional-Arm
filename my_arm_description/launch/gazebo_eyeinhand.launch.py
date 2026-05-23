#!/usr/bin/env python3
"""Gazebo 手眼标定启动文件（眼在手上）"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _with_default_path(prefix: str, existing: str) -> str:
    if existing:
        return f"{prefix}:{existing}"
    return prefix


def generate_launch_description():
    pkg_my_arm = 'my_arm_description'
    pkg_share = get_package_share_directory(pkg_my_arm)

    world_file = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm), 'worlds', 'calibration_world.sdf'
    ])
    bridge_config = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm), 'config', 'gazebo_eyeinhand', 'ros_gz_bridge_camera.yaml'
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm), 'config', 'gazebo_control', 'controllers.yaml'
    ])

    robot_description = {
        'robot_description': Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare(pkg_my_arm), 'urdf', 'scenes', 'gazebo_eyeinhand.xacro'
            ]),
        ]),
        'use_sim_time': True,
    }

    gz_plugin_path = _with_default_path(
        '/opt/ros/humble/lib', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )
    ign_plugin_path = _with_default_path(
        '/opt/ros/humble/lib', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    )

    resource_base = f"{os.path.join(pkg_share, 'models')}:{os.path.join(pkg_share, 'worlds')}"
    gz_resource_path = _with_default_path(
        resource_base, os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    ign_resource_path = _with_default_path(
        resource_base, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_arm',
            '-allow_renaming', 'false',
            '-x', '0.0', '-y', '0.0', '-z', '0.32', '-Y', '0.0'
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'ros_image_encoding': 'bgr8',
            'use_sim_time': True,
        }],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file', controllers_file,
        ],
    )

    handeye = Node(
        package='easy_handeye2',
        executable='handeye_server',
        parameters=[{
            'name': 'my_arm_calibration',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'lamp_link_1',
            'tracking_base_frame': 'camera_link',
            'tracking_marker_frame': 'calibration_marker',
            'use_sim_time': True,
        }],
        output='screen',
    )

    camera_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'my_arm/lamp_link_1/camera_sensor'],
        output='screen',
    )

    marker_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.8',
            '--y', '0',
            '--z', '0.8',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '1.5708',
            '--frame-id', 'world',
            '--child-frame-id', 'calibration_marker',
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', gz_plugin_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ign_plugin_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),

        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner]
            )
        ),

        TimerAction(period=8.0, actions=[camera_frame_tf]),
        TimerAction(period=9.0, actions=[marker_tf]),
        TimerAction(period=10.0, actions=[handeye]),
    ])
