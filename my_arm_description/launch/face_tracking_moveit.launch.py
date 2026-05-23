#!/usr/bin/env python3
"""Face tracking with MediaPipe + MoveIt2."""

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
    start_move_group = LaunchConfiguration('start_move_group')
    start_rsp = LaunchConfiguration('start_rsp')

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

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('movelt_config'), 'launch', 'move_group.launch.py'
            ])
        ]),
        condition=IfCondition(start_move_group),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('movelt_config'), 'launch', 'rsp.launch.py'
            ])
        ]),
        condition=IfCondition(start_rsp),
    )

    tracker = Node(
        package='my_arm_description',
        executable='face_moveit_tracker',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'image_encoding': LaunchConfiguration('image_encoding'),
            'min_detection_confidence': ParameterValue(
                LaunchConfiguration('min_detection_confidence'), value_type=float
            ),
            'model_selection': ParameterValue(LaunchConfiguration('model_selection'), value_type=int),
            'model_path': LaunchConfiguration('model_path'),
            'joint_names': LaunchConfiguration('joint_names'),
            'base_link': LaunchConfiguration('base_link'),
            'end_effector': LaunchConfiguration('end_effector'),
            'group_name': LaunchConfiguration('group_name'),
            'use_move_group_action': LaunchConfiguration('use_move_group_action'),
            'ignore_new_calls': LaunchConfiguration('ignore_new_calls'),
            'max_velocity': ParameterValue(LaunchConfiguration('max_velocity'), value_type=float),
            'max_acceleration': ParameterValue(LaunchConfiguration('max_acceleration'), value_type=float),
            'pan_joint': LaunchConfiguration('pan_joint'),
            'tilt_joint': LaunchConfiguration('tilt_joint'),
            'pan_center': ParameterValue(LaunchConfiguration('pan_center'), value_type=float),
            'tilt_center': ParameterValue(LaunchConfiguration('tilt_center'), value_type=float),
            'pan_min': ParameterValue(LaunchConfiguration('pan_min'), value_type=float),
            'pan_max': ParameterValue(LaunchConfiguration('pan_max'), value_type=float),
            'tilt_min': ParameterValue(LaunchConfiguration('tilt_min'), value_type=float),
            'tilt_max': ParameterValue(LaunchConfiguration('tilt_max'), value_type=float),
            'gain_pan': ParameterValue(LaunchConfiguration('gain_pan'), value_type=float),
            'gain_tilt': ParameterValue(LaunchConfiguration('gain_tilt'), value_type=float),
            'max_step': ParameterValue(LaunchConfiguration('max_step'), value_type=float),
            'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=float),
            'return_to_center': LaunchConfiguration('return_to_center'),
            'lost_timeout': ParameterValue(LaunchConfiguration('lost_timeout'), value_type=float),
            'add_table': LaunchConfiguration('add_table'),
            'table_id': LaunchConfiguration('table_id'),
            'table_frame': LaunchConfiguration('table_frame'),
            'table_size_x': ParameterValue(LaunchConfiguration('table_size_x'), value_type=float),
            'table_size_y': ParameterValue(LaunchConfiguration('table_size_y'), value_type=float),
            'table_size_z': ParameterValue(LaunchConfiguration('table_size_z'), value_type=float),
            'table_center_x': ParameterValue(LaunchConfiguration('table_center_x'), value_type=float),
            'table_center_y': ParameterValue(LaunchConfiguration('table_center_y'), value_type=float),
            'table_center_z': ParameterValue(LaunchConfiguration('table_center_z'), value_type=float),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('start_move_group', default_value='true'),
        DeclareLaunchArgument('start_rsp', default_value='false'),

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
        DeclareLaunchArgument('image_encoding', default_value='bgr8'),
        DeclareLaunchArgument('min_detection_confidence', default_value='0.6'),
        DeclareLaunchArgument('model_selection', default_value='0'),
        DeclareLaunchArgument('model_path', default_value=''),

        DeclareLaunchArgument('joint_names', default_value='l1,l2,l3,l4,l5,l6'),
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('end_effector', default_value='lamp_link_1'),
        DeclareLaunchArgument('group_name', default_value='arm_group'),
        DeclareLaunchArgument('use_move_group_action', default_value='true'),
        DeclareLaunchArgument('ignore_new_calls', default_value='true'),
        DeclareLaunchArgument('max_velocity', default_value='0.3'),
        DeclareLaunchArgument('max_acceleration', default_value='0.3'),

        DeclareLaunchArgument('pan_joint', default_value='l1'),
        DeclareLaunchArgument('tilt_joint', default_value='l2'),
        DeclareLaunchArgument('pan_center', default_value='0.0'),
        DeclareLaunchArgument('tilt_center', default_value='0.0'),
        DeclareLaunchArgument('pan_min', default_value='-1.57'),
        DeclareLaunchArgument('pan_max', default_value='1.57'),
        DeclareLaunchArgument('tilt_min', default_value='-1.0'),
        DeclareLaunchArgument('tilt_max', default_value='1.0'),

        DeclareLaunchArgument('gain_pan', default_value='0.6'),
        DeclareLaunchArgument('gain_tilt', default_value='0.6'),
        DeclareLaunchArgument('max_step', default_value='0.2'),
        DeclareLaunchArgument('update_rate', default_value='3.0'),
        DeclareLaunchArgument('return_to_center', default_value='false'),
        DeclareLaunchArgument('lost_timeout', default_value='1.0'),

        DeclareLaunchArgument('add_table', default_value='true'),
        DeclareLaunchArgument('table_id', default_value='table'),
        DeclareLaunchArgument('table_frame', default_value='base_link'),
        DeclareLaunchArgument('table_size_x', default_value='1.0'),
        DeclareLaunchArgument('table_size_y', default_value='1.0'),
        DeclareLaunchArgument('table_size_z', default_value='0.05'),
        DeclareLaunchArgument('table_center_x', default_value='0.0'),
        DeclareLaunchArgument('table_center_y', default_value='0.0'),
        DeclareLaunchArgument('table_center_z', default_value='-0.2'),

        camera_driver,
        move_group,
        rsp,
        tracker,
    ])
