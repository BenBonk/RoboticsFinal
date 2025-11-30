#!/usr/bin/env python
import copy
import math
import numpy as np
from simple_control.simple_control.global_planner import AStarPlanner
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from enum import Enum

class States(Enum): 
    IDLE = 0
    EXPLORING_WORLD = 1
    LOCATING_DOORS = 2
    OPENING_DOORS = 3
    MOVING_TO_WAYPOINT = 4
    UPDATING_MAP = 5
    PATHFINDING = 6

class RescueMission(Node):
    # On node initialization
    def __init__(self):
        super().__init__('rescue_mission')
        # Create the publisher and subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Tower goal subscriber to topic /uav/input/goal
        # will wait until something is published here
        self.goal_sub = self.create_subscription(Vector3, '/uav/input/goal', self.transformed_goal_callback, 10)
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position_request', 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.get_map, qos)
        self.gps_sub = self.create_subscription(Vector3, '/uav/sensors/gps', self.gps_callback, 10)
        self.goal = None
        self.map = None
        self.drone_position = None
        self.sent_position = None

        # Initial FSA state
        self.state = States.IDLE

        # Set the timer to call the mainloop of our class
        self.rate = 5
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)
        self.get_logger().info('Waiting for goal from tower to map')

    def get_map(self, msg):
        # Get the map width and height
        self.width = msg.info.width
        self.height = msg.info.height

        # Get the drone position
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Get the map
        self.map = np.reshape(msg.data, (self.width, self.height))

    def gps_callback(self, msg):
        self.drone_position = (msg.x, msg.y)

    # Goal callback
    def get_goal(self, msg):
        if self.origin_x is None or self.origin_y is None:  # map has not been set up yt
            return

        if len(self.goal_position) == 0:
            # Get the goal position
            x = int(round(msg.x, 0) - self.origin_x)
            y = int(round(msg.y, 0) - self.origin_y)

            # Get the drone position
            self.goal_position = [x, y]

    def mainloop(self):
        # FSA
        match self.state:
            case States.IDLE:
                # idle at start, wait until everything is initialized
                if self.goal is not None and self.map is not None and self.drone_position is not None:
                    self.state = States.UPDATING_MAP
            case States.MOVING_TO_WAYPOINT:
                # drone is currently moving towards 'self.sent_position', 
                # wait until it reaches there
                threshold = 0.2
                if math.dist(self.drone_position, self.sent_position) < threshold:
                    self.state = States.UPDATING_MAP
            case States.UPDATING_MAP:
                # wait until theres good confidence from local_planner for next move
                '''
                pseudocode:
                    - check surrounding tiles and make sure theyre either special tiles
                      or < 0.2 or > 0.8
                    - if surrounding tiles are all valid, do 'self.state=States.PATHFINDING'
                '''
            case States.PATHFINDING:
                # path towards dog
                self.get_logger().info('Planning path')
                #safe_distance_params = 'safe_distance'
                #self.declare_parameter(safe_distance_params, 1)
                #safe_distance = self.get_parameter(safe_distance_params).get_parameter_value().integer_value
                safe_distance = 1
                astar = AStarPlanner(safe_distance=safe_distance)
                # do some preprocessing on self.map
                processed_map = (self.map > 0.8).astype(np.float32)
                path = astar.plan(processed_map, self.drone_position, self.goal_position)
                if path is not None:
                    # publish path if you want
                    self.sent_position = path[0][0] + self.origin_x, path[0][1] + self.origin_y
                    if "tile at sent_position is a door":
                        # use service use_key
                        self.state = States.OPENING_DOORS
                    else:
                        msg = Vector3()
                        msg.x = float(self.sent_position[0])
                        msg.y = float(self.sent_position[1])
                        msg.z = float(3)
                        self.position_pub.publish(msg)
                        self.state = States.MOVING_TO_WAYPOINT
                else:
                    self.get_logger().info('Path not found, try another goal')
            #case States.LOCATING_DOORS:
                # shouldn't be a need for this if local planner does this automatically
            case States.OPENING_DOORS:
                # wait until door is opened, detected by local planner
                '''
                pseudocode:
                    - assuming you only open doors that are along your path, that means
                      the door position should be the same as 'self.sent_position'
                    - check if tile 'self.sent_position' is -2
                    - if yes, do 'self.state=States.PATHFINDING'
                '''
            

        # If you dont have a plan wait for a map, current position, and a goal
        if not self.have_plan:
            # If we have received the data
            if (len(self.map) != 0) and (len(self.drone_position) == 2) and (len(self.goal_position) == 2):
                self.get_logger().info('Planning path')
                # TODO Update to use a launch parameter instead of static value
                safe_distance_params = 'safe_distance'
                self.declare_parameter(safe_distance_params, 1)
                safe_distance = self.get_parameter(safe_distance_params).get_parameter_value().integer_value
                astar = AStarPlanner(safe_distance=safe_distance)
                self.path = astar.plan(self.map, self.drone_position, self.goal_position)
                if self.path is not None:
                    self.path = np.array(self.path)
                    self.have_plan = True
                    self.path[:, 0] = self.path[:, 0] + self.origin_x
                    self.path[:, 1] = self.path[:, 1] + self.origin_y
                    self.get_logger().info(f'Executing path: {self.path}')
                else:
                    self.get_logger().info('Path not found, try another goal')
        else: # We have a plan, execute it
            # Publish the path
            if len(self.p_path.data) != len(np.reshape(self.path,-1)):
                self.p_path.data = np.reshape(self.path,-1)
                self.path_pub.publish(self.p_path)

            # Publish the current waypoint
            if self.at_waypoint == False or self.sent_position == False or np.shape(self.path)[0] < 0:
                msg = Vector3()
                msg.x = float(self.path[0][0])
                msg.y = float(self.path[0][1])
                msg.z = float(3)
                self.position_pub.publish(msg)
                self.sent_position = True
            else:
                self.path = self.path[1:]
                self.sent_position = False

            # If we are done wait for next goal
            if np.shape(self.path)[0] <= 0 and self.at_waypoint:
                self.have_plan = False
                self.drone_position = copy.deepcopy(self.goal_position)
                self.goal_position = []

    def transformed_goal_callback(self, msg: Vector3):
        self.goal = msg
        self.get_logger().info('Recieved goal from tower to map')


def main():
    rclpy.init()
    try:
        rclpy.spin(RescueMission())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()