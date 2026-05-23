#!/usr/bin/env python3
"""Servo driver launch (Feetech ST3215 read)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_arm = 'my_arm_description'
    script_path = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm), 'scripts', 'st_read_angle.py'
    ])

    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    timeout = LaunchConfiguration('timeout')
    ids = LaunchConfiguration('ids')
    position_addr = LaunchConfiguration('position_addr')
    position_size = LaunchConfiguration('position_size')
    raw_max = LaunchConfiguration('raw_max')
    endian = LaunchConfiguration('endian')
    interval = LaunchConfiguration('interval')
    once = LaunchConfiguration('once')

    base_cmd = [
        'python3', script_path,
        '--port', port,
        '--baudrate', baudrate,
        '--timeout', timeout,
        '--ids', ids,
        '--position-addr', position_addr,
        '--position-size', position_size,
        '--raw-max', raw_max,
        '--endian', endian,
        '--interval', interval,
    ]

    run_loop = ExecuteProcess(
        cmd=base_cmd,
        output='screen',
        condition=UnlessCondition(once),
    )

    run_once = ExecuteProcess(
        cmd=base_cmd + ['--once'],
        output='screen',
        condition=IfCondition(once),
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('timeout', default_value='0.05'),
        DeclareLaunchArgument('ids', default_value='1-6'),
        DeclareLaunchArgument('position_addr', default_value='56'),
        DeclareLaunchArgument('position_size', default_value='2'),
        DeclareLaunchArgument('raw_max', default_value='4095'),
        DeclareLaunchArgument('endian', default_value='little'),
        DeclareLaunchArgument('interval', default_value='0.2'),
        DeclareLaunchArgument('once', default_value='false'),
        run_loop,
        run_once,
    ])
