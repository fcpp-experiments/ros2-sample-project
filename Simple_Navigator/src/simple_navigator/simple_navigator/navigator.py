import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import statistics as stat


class SimpleNavigator(Node):
    """!
    ROS 2 node that navigates based on boolean flag
    enabled by the LiDAR readings
    """

    def __init__(self):
        """!
        Initialize the navigator node and subscribe to 'scan' topic.
        """
        super().__init__('simple_navigator')
        self.scan_sub = self.create_subscription(LaserScan, 'scan',
                 self.scan_callback, 10)
        self.up_left     = False
        self.left        = False
        self.warn_left   = False
        self.down_left   = False
        self.right       = False
        self.down_right  = False
        self.warn_right  = False
        self.up_right    = False

    def scan_callback(self, msg):
        """!
        Set the corresponding flag to True whenever specified
        conditions are met.
        """
        self.up_left     = stat.mean(msg.ranges[0:5]) > 0.5
        self.left        = stat.mean(msg.ranges[5:11]) >  0.75
        self.warn_left   = min(msg.ranges[3:13]) <  0.2
        self.down_left   = stat.mean(msg.ranges[11:17]) > 1.0
        self.down_right  = stat.mean(msg.ranges[17:23]) > 1.0
        self.warn_right  = min(msg.ranges[22:32]) < 0.2
        self.right       = stat.mean(msg.ranges[23:29]) > 0.75
        self.up_right    = stat.mean(msg.ranges[29:35]) > 0.5 

def main(args=None):
    rclpy.init(args=args)
    simple_navigator = SimpleNavigator()
    rclpy.spin(simple_navigator)
    simple_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
