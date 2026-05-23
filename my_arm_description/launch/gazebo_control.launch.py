from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    pkg_my_arm = 'my_arm_description'
    
    # 处理 xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare(pkg_my_arm), 'urdf', 'scenes', 'gazebo_control.xacro'
        ]),
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # 控制器配置文件路径
    robot_controllers = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm),
        'config', 'gazebo_control', 'controllers.yaml',
    ])
    
    ros_gz_bridge_config = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm),
        'config', 'gazebo_control', 'ros_gz_bridge_gazebo.yaml',
    ])
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare(pkg_my_arm),
        'config', 'gazebo_control', 'gazebo.rviz',
    ])

    # 1. robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. spawn robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_arm',
            '-allow_renaming', 'false',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.32',
            '-Y', '0.0'
        ],
    )

    # 3. joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # 4. joint_trajectory_controller spawner（带参数文件）
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
        ],
    )

    # 5. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    # 6. RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        # Launch gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])]),
        
        # 先启动 bridge, robot_state_publisher, rviz
        bridge,
        node_robot_state_publisher,
        rviz,
        
        # spawn robot
        gz_spawn_entity,
        
        # 当机器人完全加载后，启动 joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        
        # 当 joint_state_broadcaster 加载完成后，启动轨迹控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ])
