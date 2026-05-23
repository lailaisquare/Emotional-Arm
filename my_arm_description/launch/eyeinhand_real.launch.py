#!/usr/bin/env python3
"""Real robot eye-in-hand calibration launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_arm = 'my_arm_description'
    pkg_share = FindPackageShare(pkg_my_arm)

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    robot_description_xacro = LaunchConfiguration('robot_description_xacro')
    robot_description = {
        'robot_description': Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_description_xacro,
        ]),
    }

    publish_robot_state = LaunchConfiguration('publish_robot_state')

    start_servo_driver = LaunchConfiguration('start_servo_driver')
    start_camera_driver = LaunchConfiguration('start_camera_driver')

    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        parameters=[{
            'marker_id': ParameterValue(LaunchConfiguration('marker_id'), value_type=int),
            'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
            'dictionary': LaunchConfiguration('dictionary'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'marker_frame': LaunchConfiguration('marker_frame'),
            'min_marker_size': ParameterValue(LaunchConfiguration('min_marker_size'), value_type=float),
            'use_sim_time': use_sim_time_param,
        }],
        remappings=[
            ('/image', image_topic),
            ('/camera_info', camera_info_topic),
        ],
        output='screen',
    )

    handeye_node = Node(
        package='easy_handeye2',
        executable='handeye_server',
        parameters=[{
            'name': LaunchConfiguration('calibration_name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
            'use_sim_time': use_sim_time_param,
        }],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time_param}],
        condition=IfCondition(publish_robot_state),
        output='screen',
    )

    servo_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'servo_driver.launch.py'])
        ]),
        launch_arguments={
            'port': LaunchConfiguration('servo_port'),
            'baudrate': LaunchConfiguration('servo_baudrate'),
            'timeout': LaunchConfiguration('servo_timeout'),
            'ids': LaunchConfiguration('servo_ids'),
            'interval': LaunchConfiguration('servo_interval'),
            'once': LaunchConfiguration('servo_once'),
        }.items(),
        condition=IfCondition(start_servo_driver),
    )

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, 'launch', 'camera_driver.launch.py'])
        ]),
        launch_arguments={
            'use_v4l2': LaunchConfiguration('camera_use_v4l2'),
            'use_sim_time': use_sim_time,
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description_xacro',
            default_value=PathJoinSubstitution([
                FindPackageShare(pkg_my_arm), 'urdf', 'core', 'my_arm.xacro'
            ]),
        ),
        DeclareLaunchArgument('publish_robot_state', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('start_servo_driver', default_value='true'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('servo_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('servo_baudrate', default_value='1000000'),
        DeclareLaunchArgument('servo_timeout', default_value='0.05'),
        DeclareLaunchArgument('servo_ids', default_value='1-6'),
        DeclareLaunchArgument('servo_interval', default_value='0.2'),
        DeclareLaunchArgument('servo_once', default_value='false'),

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

        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),

        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('marker_frame', default_value='calibration_marker'),
        DeclareLaunchArgument('marker_id', default_value='2'),
        DeclareLaunchArgument('marker_size', default_value='0.1'),
        DeclareLaunchArgument('dictionary', default_value='DICT_6X6_250'),
        DeclareLaunchArgument('min_marker_size', default_value='0.001'),

        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument('robot_effector_frame', default_value='lamp_link_1'),
        DeclareLaunchArgument('tracking_base_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('tracking_marker_frame', default_value='calibration_marker'),
        DeclareLaunchArgument('calibration_name', default_value='my_arm_calibration'),
        DeclareLaunchArgument('calibration_type', default_value='eye_in_hand'),

        servo_driver,
        camera_driver,
        robot_state_publisher,
        aruco_node,
        handeye_node,
    ])
