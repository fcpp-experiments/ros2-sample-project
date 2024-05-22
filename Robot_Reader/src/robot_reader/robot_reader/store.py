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
    def __init__(self, robot_name):
        self._dictionary = dict()
        self._dictionary["position"]    = PositionDTO(robot_name)
        self._dictionary["battery"]     = BatteryDTO(robot_name)
        self._dictionary["goal"]        = GoalDTO(robot_name)
        self._dictionary["dock"]        = DockDTO(robot_name)
