#!/usr/bin/env python3
"""Real robot face tracking launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('my_arm_description')

    start_camera_driver = LaunchConfiguration('start_camera_driver')

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'camera_driver.launch.py'])
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

    tracker = Node(
        package='my_arm_description',
        executable='face_servo_tracker',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'image_encoding': LaunchConfiguration('image_encoding'),
            'scale_factor': ParameterValue(LaunchConfiguration('scale_factor'), value_type=float),
            'min_neighbors': ParameterValue(LaunchConfiguration('min_neighbors'), value_type=int),
            'min_face_size': ParameterValue(LaunchConfiguration('min_face_size'), value_type=int),
            'servo_port': LaunchConfiguration('servo_port'),
            'servo_baudrate': ParameterValue(LaunchConfiguration('servo_baudrate'), value_type=int),
            'servo_timeout': ParameterValue(LaunchConfiguration('servo_timeout'), value_type=float),
            'servo_wait_status': LaunchConfiguration('servo_wait_status'),
            'servo_endian': LaunchConfiguration('servo_endian'),
            'servo_raw_max': ParameterValue(LaunchConfiguration('servo_raw_max'), value_type=int),
            'servo_pos_addr': ParameterValue(LaunchConfiguration('servo_pos_addr'), value_type=int),
            'pan_id': ParameterValue(LaunchConfiguration('pan_id'), value_type=int),
            'tilt_id': ParameterValue(LaunchConfiguration('tilt_id'), value_type=int),
            'pan_center_deg': ParameterValue(LaunchConfiguration('pan_center_deg'), value_type=float),
            'tilt_center_deg': ParameterValue(LaunchConfiguration('tilt_center_deg'), value_type=float),
            'pan_min_deg': ParameterValue(LaunchConfiguration('pan_min_deg'), value_type=float),
            'pan_max_deg': ParameterValue(LaunchConfiguration('pan_max_deg'), value_type=float),
            'tilt_min_deg': ParameterValue(LaunchConfiguration('tilt_min_deg'), value_type=float),
            'tilt_max_deg': ParameterValue(LaunchConfiguration('tilt_max_deg'), value_type=float),
            'gain_pan': ParameterValue(LaunchConfiguration('gain_pan'), value_type=float),
            'gain_tilt': ParameterValue(LaunchConfiguration('gain_tilt'), value_type=float),
            'step_limit_deg': ParameterValue(LaunchConfiguration('step_limit_deg'), value_type=float),
            'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=float),
            'return_to_center': LaunchConfiguration('return_to_center'),
            'lost_timeout': ParameterValue(LaunchConfiguration('lost_timeout'), value_type=float),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('camera_use_v4l2', default_value='true'),
        DeclareLaunchArgument('camera_pkg', default_value='v4l2_camera'),
        DeclareLaunchArgument('camera_exe', default_value='v4l2_camera_node'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video1'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('camera_external_launch', default_value=''),

        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('image_encoding', default_value='bgr8'),
        DeclareLaunchArgument('scale_factor', default_value='1.1'),
        DeclareLaunchArgument('min_neighbors', default_value='4'),
        DeclareLaunchArgument('min_face_size', default_value='60'),

        DeclareLaunchArgument('servo_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_baudrate', default_value='1000000'),
        DeclareLaunchArgument('servo_timeout', default_value='0.05'),
        DeclareLaunchArgument('servo_wait_status', default_value='true'),
        DeclareLaunchArgument('servo_endian', default_value='little'),
        DeclareLaunchArgument('servo_raw_max', default_value='4095'),
        DeclareLaunchArgument('servo_pos_addr', default_value='42'),

        DeclareLaunchArgument('pan_id', default_value='1'),
        DeclareLaunchArgument('tilt_id', default_value='2'),
        DeclareLaunchArgument('pan_center_deg', default_value='180.0'),
        DeclareLaunchArgument('tilt_center_deg', default_value='180.0'),
        DeclareLaunchArgument('pan_min_deg', default_value='0.0'),
        DeclareLaunchArgument('pan_max_deg', default_value='360.0'),
        DeclareLaunchArgument('tilt_min_deg', default_value='0.0'),
        DeclareLaunchArgument('tilt_max_deg', default_value='360.0'),

        DeclareLaunchArgument('gain_pan', default_value='5.0'),
        DeclareLaunchArgument('gain_tilt', default_value='5.0'),
        DeclareLaunchArgument('step_limit_deg', default_value='3.0'),
        DeclareLaunchArgument('update_rate', default_value='5.0'),
        DeclareLaunchArgument('return_to_center', default_value='false'),
        DeclareLaunchArgument('lost_timeout', default_value='1.0'),

        camera_driver,
        tracker,
    ])
