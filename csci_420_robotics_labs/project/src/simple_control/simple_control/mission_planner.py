# THIS SCRIPT HANDLES THE FINITE STATE MACHINE, UNLOCKS DOORS, AND EXPLORATION

#!/usr/bin/env python
import copy
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Bool

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
        self.at_goal_pub = self.create_publisher(Bool, '/uav/sensors/at_waypoint', 1)
        self.keyboard_sub = self.create_subscription(Vector3, '/uav/input/position_request',self.getPositionRequest, 1)

        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.get_gps, 1)

        self.acceptance_range = 0.5
       
        self.state = DroneState.HOVERING
        self.get_logger().info("Current State: HOVERING")
        self.goal_cmd = Vector3()

        self.drone_position = Point()
        self.goal_cmd.z = 3.0
        self.goal_changed = False

        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)


    # Callback for the keyboard manager
    def getPositionRequest(self, msg):
        if self.state == DroneState.HOVERING:
            self.goal_changed = True

    def get_gps(self, msg):
        self.drone_position = msg.pose.position

    # Converts a position to string for printing
    def goalToString(self, msg):
        pos_str = "(" + str(msg.x)
        pos_str += ", " + str(msg.y)
        pos_str += ", " + str(msg.z) + ")"
        return pos_str

    # This function is called when we are in the hovering state
    def processHovering(self):
        # Print the requested goal if the position changed
        if self.goal_changed:
            self.state = DroneState.HOVERING
            self.goal_changed = False

    # This function is called when we are in the moving state
    def processMoving(self):
        # Compute the distance between requested position and current position
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        dz = self.goal_cmd.z - self.drone_position.z

        # Euclidean distance
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2) + np.power(dz, 2))
        # If goal is reached transition to hovering
        if distance_to_goal < self.acceptance_range:
            self.state = DroneState.HOVERING
            bool = Bool()
            bool.data = True
            self.at_goal_pub.publish(bool)

    # The main loop of the function
    def mainloop(self):
        # Publish the position
        self.position_pub.publish(self.goal_cmd)

        if self.state == DroneState.HOVERING:
            self.processMoving()
        elif self.state == DroneState.HOVERING:
            self.processHovering()

        # Euclidean distance
        dx = self.goal_cmd.x - self.drone_position.x
        dy = self.goal_cmd.y - self.drone_position.y
        distance_to_goal = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        bool = Bool()
        bool.data = True if (distance_to_goal < self.acceptance_range) else False
        self.at_goal_pub.publish(bool)

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