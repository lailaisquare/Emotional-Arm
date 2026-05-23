#!/usr/bin/env python3
"""Camera calibration launch (chessboard)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import JoinSubstitution, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    board_size = LaunchConfiguration('board_size')
    square_size = LaunchConfiguration('square_size')
    pattern = LaunchConfiguration('pattern')
    image_topic = LaunchConfiguration('image_topic')
    camera_ns = LaunchConfiguration('camera_ns')
    start_camera_driver = LaunchConfiguration('start_camera_driver')

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_arm_description'), 'launch', 'camera_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'use_v4l2': LaunchConfiguration('camera_use_v4l2'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_pkg': LaunchConfiguration('camera_pkg'),
            'camera_exe': LaunchConfiguration('camera_exe'),
            'camera_name': LaunchConfiguration('camera_name'),
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': LaunchConfiguration('camera_width'),
            'image_height': LaunchConfiguration('camera_height'),
            'pixel_format': LaunchConfiguration('camera_pixel_format'),
            'framerate': LaunchConfiguration('camera_fps'),
            'external_camera_launch': LaunchConfiguration('camera_external_launch'),
        }.items(),
        condition=IfCondition(start_camera_driver),
    )

    image_remap = JoinSubstitution([TextSubstitution(text='image:='), image_topic])
    camera_remap = JoinSubstitution([TextSubstitution(text='camera:='), camera_ns])

    calibrator = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'camera_calibration', 'cameracalibrator',
            '--size', board_size,
            '--square', square_size,
            '--pattern', pattern,
            image_remap,
            camera_remap,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('board_size', default_value='8x6'),
        DeclareLaunchArgument('square_size', default_value='0.025'),
        DeclareLaunchArgument('pattern', default_value='chessboard'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_ns', default_value='/camera'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_use_v4l2', default_value='true'),
        DeclareLaunchArgument('camera_pkg', default_value='v4l2_camera'),
        DeclareLaunchArgument('camera_exe', default_value='v4l2_camera_node'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('camera_external_launch', default_value=''),
        camera_driver,
        calibrator,
    ])
