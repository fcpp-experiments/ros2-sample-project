# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

from .position import PositionDTO
from .battery  import BatteryDTO
from .goal     import GoalDTO
from .dock     import DockDTO

from . import config

class Store:
    """!
    Class used to store sensor data received from the robot before
    writing it into a feedback file.
    """
    def __init__(self):
        self._dictionary = dict()
        for robot_name in config.ROBOTS:
            self._dictionary[robot_name] = dict()
            self._dictionary[robot_name]["position"]    = PositionDTO(robot_name)
            self._dictionary[robot_name]["battery"]     = BatteryDTO(robot_name)
            self._dictionary[robot_name]["goal"]        = GoalDTO(robot_name)
            self._dictionary[robot_name]["dock"]        = DockDTO(robot_name)

content = Store()
