#THIS SCRIPT IS FOR THE FINITE STATE MACHINE< DOOR HANDLING, AND EXPLORATION

#!/usr/bin/env python
import copy
import numpy as np
from astar_class import AStarPlanner
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a class which we will use to take keyboard commands and convert them to a position
class MissionPlanner(Node):
    # On node initialization
    def __init__(self):
        super().__init__('mission_planner')

        # Set the timer to call the mainloop of our class
        self.rate = 5
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)
        self.get_logger().info('Waiting for goal from tower to map')

 
    def mainloop(self):
        pass



def main():
    rclpy.init()
    try:
        rclpy.spin(MissionPlanner())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()