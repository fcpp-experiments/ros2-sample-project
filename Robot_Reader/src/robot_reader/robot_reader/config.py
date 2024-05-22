"""!
Module with configuration constant used in the robot_reader package.
"""
# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

from rclpy.qos import qos_profile_sensor_data

ODOM_TOPIC              = "odom"
AMCL_TOPIC              = "amcl_pose"
BATTERY_TOPIC           = "battery_state"
GOAL_TOPIC              = "navigator_state"

DEFAULT_QOS_TOPIC       = qos_profile_sensor_data
BATTERY_QOS_TOPIC       = qos_profile_sensor_data

POLL_WRITER_SECONDS     = 0.5
