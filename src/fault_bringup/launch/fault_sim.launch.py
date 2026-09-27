"""Fault test system: Gazebo + TB3 Burger + fault injectors + health monitor + command arbitration.

Sensors:  Gazebo /scan_raw -> lidar_fault_injector -> /scan
          Gazebo /odom_raw -> odom_fault_injector  -> /odom
Commands: nav_command_source (respawn) -> /cmd_vel_nav_raw -> control_delay_injector -> /cmd_vel_nav
          -> speed_limiter -> /cmd_vel_nav_limited (prio 10)
          /cmd_vel_safety (255), /cmd_vel_idle_stop (1)  -> twist_mux -> /cmd_vel

Arguments:
  gui:=true|false        Gazebo client window
  dashboard:=true|false  fault_dashboard (status + fault injection buttons)
  world:=test|warehouse  test = TurtleBot3 world (regression tests, default)
                         warehouse = 14 m x 10 m AMR warehouse (worlds/amr_warehouse.world)
  demo:=true|false       integrated demo: robot drives a 0.4 m circle so it can drive for minutes
                         (test world: between four pillars around (-0.55, -0.55);
                          warehouse: inside the start zone around (0, -3.5))
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# (world, demo) -> world file, spawn pose, nav_command_source angular_z
# demo: start tangent to a 0.4 m circle (0.2 m/s / 0.4 m = 0.5 rad/s)
CONFIGS = {
    ('test', False): {'world': 'turtlebot3_world_vis.world', 'x': '-2.0', 'y': '-0.5', 'yaw': '0.0',
                      'angular_z': 0.0},
    ('test', True): {'world': 'turtlebot3_world_demo.world', 'x': '-0.55', 'y': '-0.95', 'yaw': '0.0',
                     'angular_z': 0.5},
    # warehouse: start zone, facing north up the main aisle
    ('warehouse', False): {'world': 'amr_warehouse.world', 'x': '0.0', 'y': '-4.0', 'yaw': '1.5708',
                           'angular_z': 0.0},
    ('warehouse', True): {'world': 'amr_warehouse.world', 'x': '0.0', 'y': '-3.9', 'yaw': '0.0',
                          'angular_z': 0.5},
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='false'),
        DeclareLaunchArgument('demo', default_value='false'),
        DeclareLaunchArgument('world', default_value='test'),
        OpaqueFunction(function=nodes),
    ])


def nodes(context):
    share = get_package_share_directory('fault_bringup')
    tb3 = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    gz = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world_name = LaunchConfiguration('world').perform(context)
    demo = LaunchConfiguration('demo').perform(context) == 'true'
    if (world_name, demo) not in CONFIGS:
        raise RuntimeError(f"world:={world_name} is not one of: test, warehouse")
    cfg = CONFIGS[(world_name, demo)]
    world = os.path.join(share, 'worlds', cfg['world'])
    model = os.path.join(share, 'models', 'turtlebot3_burger_fault', 'model.sdf')
    gui = LaunchConfiguration('gui')

    return [
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
                        '-x', cfg['x'], '-y', cfg['y'], '-z', '0.01', '-Y', cfg['yaw']]),
        Node(package='fault_injector', executable='lidar_fault_injector', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='fault_injector', executable='odom_fault_injector', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='fault_bringup', executable='nav_command_source', output='screen',
             parameters=[os.path.join(share, 'config', 'nav_command_source.yaml'),
                         {'angular_z': cfg['angular_z']}],
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
        Node(package='fault_bringup', executable='fault_dashboard', output='screen',
             condition=IfCondition(LaunchConfiguration('dashboard'))),
    ]
