# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by: Giordano Scarso

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_bringup_dir = get_package_share_directory('rumbo_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')
    robot_sdf = os.path.join(gazebo_bringup_dir, 'models', 'rumbo', 'robot-maze.sdf')

    # Simulation settings
    world = LaunchConfiguration('world')
    robot_conf = LaunchConfiguration('robots')
    simulator = LaunchConfiguration('simulator')
    decrease_battery = LaunchConfiguration('decrease_battery')
    dock_enabled = LaunchConfiguration('dock_enabled')
    sleep_time = LaunchConfiguration('sleep_time')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    use_proxies = LaunchConfiguration('use_proxies')
    log_settings = LaunchConfiguration('log_settings', default='true')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/odom', 'odom')]

    declare_sleep_time_cmd = DeclareLaunchArgument(
        'sleep_time', default_value='0',
        description='Time in seconds robots waits after reaching a navigation position')

    declare_robots_cmd = DeclareLaunchArgument(
        'robots',
        default_value=os.path.join(gazebo_bringup_dir, 'params', 'robots.yaml'),
        description='Full path to configuration for spawning multiple robots')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'library-new.yaml'),
        description='Full path to map file to load')

    declare_robot_params_template_file_cmd = DeclareLaunchArgument(
        'tb3_template_params_file',
        default_value=os.path.join(gazebo_bringup_dir, 'params', 'nav2_multirumbo_params_template.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Whether to restart nav2 crashed processes')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_proxies_cmd = DeclareLaunchArgument(
        'use_proxies',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to enable ROS2 proxies.')

    declare_decrease_battery = DeclareLaunchArgument(
        'decrease_battery',
        default_value='False',
        description='Whether to decrease battery charge level on successful goal.')

    declare_dock_enabled = DeclareLaunchArgument(
        'dock_enabled',
        default_value='0',
        choices=['0', '1', '2'],
        description='Whether to not return (0) or to return (1) to initial position on successful goal and (2) dock to base station.')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    time = 10.0
    with open(os.path.abspath(os.environ['ROBOTS_YAML']), 'r') as f:
        robots = yaml.safe_load(f)
    for robot in robots:
        group = GroupAction(
                [
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_to_laser',
                    namespace=TextSubstitution(text=robot['name']),
                    remappings=remappings,
                    arguments=['0.14','0.0','0.12','0','0','0','base_link','base_laser']
                    ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_ir_to_laser',
                    namespace=TextSubstitution(text=robot['name']),
                    remappings=remappings,
                    arguments=['0.16','0.0','0.06','0','0','0','base_link','base_ir']
                    ),
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    output='screen',
                    remappings=remappings,
                    namespace=TextSubstitution(text=robot['name']),
                    arguments=[
                        '-entity', TextSubstitution(text=robot['name']),
                        '-file', robot_sdf,
                        '-robot_namespace', TextSubstitution(text=robot['name']),
                        '-x', str(robot['x_pose']),
                        '-y', str(robot['y_pose']),
                        '-z', str(robot['z_pose']),
                        '-R', '0.0',
                        '-P', '0.0',
                        '-Y', str(robot['yaw'])]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo( condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        timed_group = TimerAction(period=time,
                actions=[group])
        time += 8.0
        nav_instances_cmds.append(timed_group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robots_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_params_template_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_proxies_cmd)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_decrease_battery)
    ld.add_action(declare_dock_enabled)
    ld.add_action(declare_sleep_time_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
