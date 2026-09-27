"""Fault test system: Gazebo + TB3 Burger + fault injectors + health monitor + command arbitration.

Sensors:  Gazebo /scan_raw -> lidar_fault_injector -> /scan
          Gazebo /odom_raw -> odom_fault_injector  -> /odom
Commands: nav_command_source (respawn) -> /cmd_vel_nav_raw -> control_delay_injector -> /cmd_vel_nav
          -> speed_limiter -> /cmd_vel_nav_limited (prio 10)
          /cmd_vel_safety (255), /cmd_vel_idle_stop (1)  -> twist_mux -> /cmd_vel
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fault_bringup')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    gz = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world = os.path.join(share, 'worlds', 'turtlebot3_world_vis.world')
    model = os.path.join(share, 'models', 'turtlebot3_burger_fault', 'model.sdf')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'gzserver.launch.py')),
            launch_arguments={'world': world,
                              'params_file': os.path.join(share, 'config', 'gazebo_params.yaml')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'burger', '-file', model,
                        '-x', '-2.0', '-y', '-0.5', '-z', '0.01']),
        Node(package='fault_injector', executable='lidar_fault_injector', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='fault_injector', executable='odom_fault_injector', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='fault_bringup', executable='nav_command_source', output='screen',
             parameters=[os.path.join(share, 'config', 'nav_command_source.yaml')],
             respawn=True, respawn_delay=1.2),
        Node(package='fault_injector', executable='nav_fault_injector', output='screen'),
        Node(package='fault_injector', executable='control_delay_injector', output='screen',
             parameters=[os.path.join(share, 'config', 'control_delay.yaml')]),
        Node(package='fault_monitor', executable='speed_limiter', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='twist_mux', executable='twist_mux', output='screen',
             parameters=[os.path.join(share, 'config', 'twist_mux.yaml')],
             remappings=[('/cmd_vel_out', '/cmd_vel')]),
        Node(package='fault_monitor', executable='idle_stop', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='fault_monitor', executable='health_monitor', output='screen',
             parameters=[os.path.join(share, 'config', 'health_monitor.yaml')]),
    ])
