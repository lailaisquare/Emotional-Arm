import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_my_arm = 'my_arm_description'
    pkg_movelt = 'movelt_config'

    # 机器人描述（xacro 生成 URDF），并启用仿真时间
    robot_description = {
        'robot_description': Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(pkg_my_arm), 'urdf', 'scenes', 'gazebo_moveit.xacro']),
        ]),
        'use_sim_time': True
    }

    # 控制器配置文件路径
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm),
        'config', 'gazebo_moveit', 'controllers_moveit.yaml'
    ])

    # ros_gz_bridge 配置文件路径
    ros_gz_bridge_config = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm),
        'config', 'gazebo_moveit', 'ros_gz_bridge_gazebo.yaml'
    ])

    # ==================== 1. Gazebo 仿真环境 ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # ==================== 2. Robot State Publisher ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ==================== 3. 生成机器人 (Spawn) ====================
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

    # ==================== 4. ros_gz_bridge ====================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    # ==================== 5. 控制器加载 (按顺序) ====================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_group_controller',
            '--param-file', robot_controllers,
        ],
    )

    # ==================== 6. MoveIt 核心节点 ====================
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(pkg_movelt), 'launch','gazebo_movelt', 'gazebo_move_group.launch.py'])
        ])
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare(pkg_movelt), 'launch','gazebo_movelt', 'gazebo_moveit_rviz.launch.py'])
        ])
    )

    # ==================== 7. 自动设置仿真时间 (解决 waitForExecution 超时) ====================
    set_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group', 'use_sim_time', 'true'],
        output='screen'
    )

    # 可选的：增加执行超时时间（若轨迹较长）
    set_exec_duration = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/move_group',
             'execute_trajectory_action_capability.allowed_execution_duration', '30.0'],
        output='screen'
    )

    # ==================== 启动顺序 ====================
    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,

        # 控制器在机器人模型生成之后才加载
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

        # MoveIt 在控制器完全启动后启动
        TimerAction(period=8.0, actions=[move_group]),
        TimerAction(period=10.0, actions=[moveit_rviz]),

        # 自动设置参数（稍微延迟，确保 move_group 已启动）
        TimerAction(period=12.0, actions=[set_sim_time]),
        TimerAction(period=13.0, actions=[set_exec_duration]),
    ])
