# THIS SCRIPT HANDLES THE FINITE STATE MACHINE, UNLOCKS DOORS, AND EXPLORATION

#!/usr/bin/env python
import copy
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Bool, Int32

# A class to keep track of the quadrotors state
class DroneState(Enum):
    HOVERING = 1
    EXPLORE_WORLD = 2
    LOCATE_DOOR = 3
    OPEN_DOOR = 4
    MOVE_TO_WAYPOINT = 5
    UPDATE_MAP = 6

class MissionPlanner(Node):

    # Node initialization
    def __init__(self):
        super().__init__('mission_planner')
        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position', 1)
        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_gps, 1)

        self.acceptance_range = 0.5
       
        self.state = DroneState.HOVERING
        self.get_logger().info("Current State: HOVERING")
        self.goal_cmd = Vector3()

        self.drone_position = Point()
        self.goal_cmd.z = 3.0
        self.goal_changed = False

        # key stuff
        self.keys_remaining = 4
        self.get_keys_sub = self.create_subscription(Int32, '/uav/input/position_request',self.get_keys_remaining, 1)
        self.use_key_pub = self.create_publisher(Point, '/uav/input/position_request', 1)

        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def get_keys_remaining(self, msg):
        self.keys_remaining = msg.data

    def get_gps(self, msg):
        self.drone_position = msg.pose.position

    def use_key(self, x, y):
        if self.keys_remaining > 0:
            use_pos = Point()
            use_pos.x = x
            use_pos.y = y
            self.use_key_pub.publish(use_pos)
            self.get_logger().info(f"successfully used a key at: {x},{y}")
            self.get_logger().info(f"{self.keys_remaining} keys left")
        else:
            self.get_logger().info("no keys left")

    def processHovering(self):
        pass
    def processExplore(self):
        pass
    def processLocateDoor(self):
        pass
    def processOpenDoor(self):
        pass
    def processMoveToWaypoint(self):
        pass
    def processUpdateMap(self):
        pass

    # The main loop of the function
    def mainloop(self):
        self.position_pub.publish(self.goal_cmd)

        if self.state == DroneState.HOVERING:
            self.processHovering()
        elif self.state == DroneState.EXPLORE_WORLD:
            self.processExplore()
        elif self.state == DroneState.LOCATE_DOOR:
            self.processLocateDoor()
        elif self.state == DroneState.OPEN_DOOR:
            self.processOpenDoor()
        elif self.state == DroneState.MOVE_TO_WAYPOINT:
            self.processMoveToWaypoint()
        elif self.state == DroneState.UPDATE_MAP:
            self.processUpdateMap()

    def at_goal(self):
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        bool = Bool()
        bool.data = True if (distance_to_goal < self.acceptance_range) else False
        return bool


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