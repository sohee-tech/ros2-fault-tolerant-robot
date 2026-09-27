"""Warehouse AMR + fault system + Nav2 (map_server, AMCL, planner, controller, BT navigator).

Nav2 velocity output never goes straight to /cmd_vel:
  controller_server / behavior_server  cmd_vel -> /cmd_vel_nav_raw
  -> control_delay_injector -> /cmd_vel_nav -> speed_limiter -> /cmd_vel_nav_limited
  -> twist_mux (safety 255 > nav 10 > idle_stop 1) -> /cmd_vel
(nav2_bringup's navigation_launch.py is not used: it remaps the controller to /cmd_vel_nav and the
velocity smoother / behavior server to /cmd_vel, which would collide with the fault system topics.)

Arguments:
  gui:=true|false        Gazebo client
  dashboard:=true|false  fault dashboard
  mission:=true|false    start nav2_mission_runner (waypoint mission, waits for /mission/start)
  autostart_mission:=true|false  run the mission as soon as Nav2 is active
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAV_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower']


def generate_launch_description():
    share = get_package_share_directory('fault_bringup')
    params = os.path.join(share, 'config', 'nav2_warehouse.yaml')
    map_yaml = os.path.join(share, 'maps', 'amr_warehouse.yaml')
    to_fault_path = [('cmd_vel', 'cmd_vel_nav_raw')]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('autostart_mission', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'fault_sim.launch.py')),
            launch_arguments={'world': 'warehouse', 'demo': 'false',
                              'gui': LaunchConfiguration('gui'),
                              'dashboard': LaunchConfiguration('dashboard')}.items()),

        # --- localization ---
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),

        # --- navigation ---
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params], remappings=to_fault_path),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params], remappings=to_fault_path),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': NAV_NODES}]),

        # --- waypoint mission ---
        Node(package='fault_bringup', executable='nav2_mission_runner', output='screen',
             condition=IfCondition(LaunchConfiguration('mission')),
             parameters=[os.path.join(share, 'config', 'nav2_mission.yaml'),
                         {'layout_file': os.path.join(share, 'config', 'amr_warehouse_layout.yaml'),
                          'autostart': ParameterValue(LaunchConfiguration('autostart_mission'),
                                                      value_type=bool)}]),
    ])
