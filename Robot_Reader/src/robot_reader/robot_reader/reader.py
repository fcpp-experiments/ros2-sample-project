# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_system_interfaces.msg import NavigatorFeedback
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy.qos as qos

import time, threading, string

from . import store
from . import config
from . import coords
from .position import PositionDTO
from .battery import BatteryDTO
from .goal import GoalDTO
from .dock import DockDTO
from .feedback_writer import FeedbackWriter

class RobotInformationReader(Node):
    """!
    Class exteding ROS2 Node to receive sensor data from the robot and
    write it onto files to be used by other programs.
    """

    def __init__(self, namespace, robot_name, origin_x=0.0, origin_y=0.0, rotation=0.0):
        """!
        Class' constructor requiring a namespace and optionally the robot
        coordinates relative to the origin.
        """
        super().__init__('robot_information_reader', namespace=namespace)
        self.robot_name = robot_name
        self.content = store.Store(robot_name)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.rotation = float(rotation)
        self.timer = self.create_timer(config.POLL_WRITER_SECONDS, self.write_file)
        self.feedbackWriter = FeedbackWriter()
        self.get_logger().info('Robot reader initialized')
        self.subscription_position = self.position_subscribe(namespace)

        # subscribe battery
        self.get_logger().info('Subscribing to: "%s/%s"' % (namespace, config.BATTERY_TOPIC))
        self.subscription_battery = self.create_subscription(
            BatteryState,
            config.BATTERY_TOPIC,
            self.listener_battery_callback,
            config.BATTERY_QOS_TOPIC)

        # subscribe goal
        self.get_logger().info('Subscribing to: "%s/%s"' % (namespace, config.GOAL_TOPIC))
        self.subscription_goal = self.create_subscription(
            NavigatorFeedback,
            config.GOAL_TOPIC,
            self.listener_goal_callback,
            config.DEFAULT_QOS_TOPIC)
        self.subscription_position
        self.subscription_battery
        self.subscription_goal

    def position_subscribe(self, namespace):
        # subscribe amcl
        self.get_logger().info('Subscribing to: "%s/%s"' % (namespace, config.AMCL_TOPIC))
        return self.create_subscription(
                PoseWithCovarianceStamped,
                config.AMCL_TOPIC,
                self.listener_position_callback,
                qos.QoSProfile(depth=5,
                    durability=qos.DurabilityPolicy.TRANSIENT_LOCAL)
                )

    def listener_position_callback(self, msg):
        """!
        Given a message with pose.pose.position, converts the coordinate
        changing the origin point from the robot origin point to the global
        origin point. Callback for the position topic.
        """
        x, y, angle = coords.rel2abs(msg.pose.pose.position.x, msg.pose.pose.position.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
                self.origin_x, self.origin_y, self.rotation)
        position            = PositionDTO(self.robot_name, x, y, angle)
        self.content._dictionary["position"]     = position
        self.get_logger().debug('I heard position from %s: "%s"' % (self.robot_name, str(position)))

    def listener_battery_callback(self, msg: BatteryState):
        """!
        Given a sensor_msgs.msg.BatteryState message it stores it into
        the content dictionary. Callback for the /battery_state topic.
        """
        battery = BatteryDTO(self.robot_name,
                # range of msg.percentage is different for each robots.
                # so we compute the percentage
                msg.charge/msg.capacity*100,
                msg.temperature,
                msg.capacity,
                msg.power_supply_status)
        self.content._dictionary["battery"] = battery
        self.get_logger().debug('I heard battery from %s: "%s"' % (self.robot_name, str(battery)))

    def listener_goal_callback(self, msg: GoalDTO):
        """!
        Given a Goal message it stores it into the content dictionary.
        Callback for the /navigator_state topic.
        """
        dto = GoalDTO(self.robot_name, msg.id, msg.state, msg.current_step)
        target = "goal"
        if msg.type == "nav":
            dto = GoalDTO(self.robot_name, msg.id, msg.state, msg.current_step)
            target = "goal"
        elif msg.type == "dock":
            dto = DockDTO(self.robot_name, msg.id, msg.state)
            target = "dock"
        self.content._dictionary[target] = dto
        self.get_logger().debug('I heard goal from %s: "%s"' % (self.robot_name, str(dto)))
        self.write_file()

    def write_file(self):
        """!
        Writes the values stored in the content dictionary, to a feedback file.
        Called every config.POLL_WRITER_SECONDS seconds.
        """
        if (self.content._dictionary["position"].pos_x != None or
            self.content._dictionary["battery"].percentage_charge != None or
            self.content._dictionary["dock"].goal_status != -1 or
            self.content._dictionary["goal"].goal_status != -1):
            filename = self.feedbackWriter.write_file(self.robot_name,
                    self.content._dictionary["position"],
                    self.content._dictionary["battery"],
                    self.content._dictionary["goal"],
                    self.content._dictionary["dock"],
                    "-best_effort")
            self.get_logger().debug(
                    "filename: %s "
                    "store._position: %s "
                    "store._battery: %s "
                    "store._goal: %s "
                    "store._dock: %s "
                    % (
                        filename,
                        self.content._dictionary["position"],
                        self.content._dictionary["battery"],
                        self.content._dictionary["goal"],
                        self.content._dictionary["dock"]
                        ) 
                    )

def main(args=sys.argv):
    rclpy.init(args=args)
    if(len(args) > 4):
        robot_information_reader = RobotInformationReader(namespace=args[1],
                robot_name=args[2], origin_x=args[3], origin_y=args[4],
                rotation=args[5])
    else:
        robot_information_reader = RobotInformationReader(namespace=args[1],
                robot_name=args[2])
    executor = MultiThreadedExecutor()
    executor.add_node(robot_information_reader)
    executor.spin()
    robot_information_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
