#!/usr/bin/env python3
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
# Authors: Giordano Scarso

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('rumbo_gazebo'), 'launch')
    bringup_dir = get_package_share_directory('rumbo_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_conf = LaunchConfiguration('robots')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    decrease_battery = LaunchConfiguration('decrease_battery')
    dock_enabled = LaunchConfiguration('dock_enabled')
    sleep_time = LaunchConfiguration('sleep_time')

    declare_sleep_time_cmd = DeclareLaunchArgument(
        'sleep_time', default_value='0',
        description='Time in seconds robots waits after reaching a navigation position')

    declare_robots_cmd = DeclareLaunchArgument(
        'robots',
        default_value=os.path.join(bringup_dir, 'params', 'robots.yaml'),
        description='Full path to configuration for spawning multiple robots')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'biblioteca.sdf'),
        description='Full path to world file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'library-new.yaml'),
        description='Full path to map file to load')

    declare_decrease_battery = DeclareLaunchArgument(
        'decrease_battery',
        default_value='False',
        description='Whether to decrease battery on goal success.')

    declare_dock_enabled = DeclareLaunchArgument(
        'dock_enabled',
        default_value='0',
        choices=['0', '1', '2'],
        description='Whether to not return (0) or to return (1) to initial position on successful goal and (2) dock to base station.')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'gdb': 'false', # Set to true for debugging
            }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    multi_robot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir,
                'multi_rumbo_simulation_ros2_only_launch.py')),
            launch_arguments={
                'use_namespace': 'True',
                'map': map_yaml_file,
                'decrease_battery': decrease_battery,
                'dock_enabled': dock_enabled,
                'sleep_time': sleep_time,
                'world': world,
                'robots': robot_conf,
                'use_sim_time': 'True',
                'autostart': 'True',
                'use_rviz': 'False',
                'use_simulator': 'False',
                'use_composition': 'False',
                'use_respawn': 'False',
                'headless': 'True',
                }.items())

    timed_multirobot = TimerAction(period=10.0,
            actions=[multi_robot_cmd])


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_sleep_time_cmd)
    ld.add_action(declare_robots_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_decrease_battery)
    ld.add_action(declare_dock_enabled)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(timed_multirobot)

    return ld
