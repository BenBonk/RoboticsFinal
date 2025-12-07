#!/usr/bin/env python
import copy
import math
import numpy as np
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from enum import Enum
import time

from environment_controller.srv import UseKey
from astar_class import AStarPlanner

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
        self.use_key_client = self.create_client(UseKey, 'use_key')

        self.goal_sub = self.create_subscription(Vector3, '/uav/input/goal', self.transformed_goal_callback, 1)
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position', 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.get_map, qos)
        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.gps_callback, 1)
        self.reset_lidar_pub = self.create_publisher(Bool, '/reset_lidar', 1)
        self.path_pub = self.create_publisher(Int32MultiArray, '/uav/path', 1)
        self.p_path = Int32MultiArray()
        self.goal = None
        self.map = None
        self.drone_position = None
        self.sent_position = None
        self.recieved_og = False
        self.seen_pos = set()

        # Initial FSA state
        self.state = States.IDLE

        # Set the timer to call the mainloop of our class
        self.rate = 5
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)
        self.get_logger().info('Waiting for goal from tower to map')

    def use_key(self, x, y):
        #if self.keys_remaining > 0:
        if True:
            req = UseKey.Request()
            req.door_loc = Point(x=float(x), y=float(y), z=0.0)

            # Asynchronous service call
            future = self.use_key_client.call_async(req)
            future.add_done_callback(self.use_key_callback)

            self.get_logger().info(f"Attempting to open door at ({x},{y})...")
        else:
            self.get_logger().info("No keys left")

    def use_key_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        if result.success:
            self.get_logger().info("Door opened successfully!")
        else:
            self.get_logger().info("Door did not open.")

    def get_map(self, msg):
        # Get the map width and height
        self.width = msg.info.width
        self.height = msg.info.height

        # Get the drone position
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # Get the map
        self.map = np.reshape(msg.data, (self.width, self.height))
        self.recieved_og = True

    def gps_callback(self, msg):
        pos = msg.pose.position
        self.drone_position = [pos.x, pos.y]

    def transformed_goal_callback(self, msg: Vector3):
        self.goal = [int(round(msg.x, 0)), int(round(msg.y, 0))]
        self.get_logger().info(f'Recieved goal from tower to map: {self.goal}')
        #WE GOTTA UPDATE THE MAP TO MARK GOAL

    def mainloop(self):
        # FSA
        self.get_logger().info(f"Current State: {self.state}")
        match self.state:
            case States.IDLE:
                # idle at start, wait until everything is initialized
                if self.goal is not None and self.map is not None and self.drone_position is not None:
                    self.state = States.UPDATING_MAP
            case States.MOVING_TO_WAYPOINT:
                # drone is currently moving towards 'self.sent_position', 
                # wait until it reaches there
                threshold = 0.1
                self.get_logger().info(f"{self.drone_position} |||||| {self.sent_position}")
                if math.dist(self.drone_position, self.sent_position) < threshold:
                    processed_drone_position = (int(round(self.drone_position[0], 0) - self.origin_x),
                                            int(round(self.drone_position[1], 0) - self.origin_y))
                    
                    if processed_drone_position not in self.seen_pos:
                        self.seen_pos.add(processed_drone_position)

                        time.sleep(2)
                        self.recieved_og = False
                        self.state = States.UPDATING_MAP

                        msg = Bool()
                        msg.data = True
                        self.reset_lidar_pub.publish(msg)
                    else:
                        time.sleep(1)
                        self.state = States.PATHFINDING
                    
            case States.UPDATING_MAP:
                # wait until theres good confidence from local_planner for next move
                if self.recieved_og:
                    self.state = States.PATHFINDING

            case States.PATHFINDING:
                # path towards dog
                self.get_logger().info('Planning path')
                #safe_distance_params = 'safe_distance'
                #self.declare_parameter(safe_distance_params, 1)
                #safe_distance = self.get_parameter(safe_distance_params).get_parameter_value().integer_value
                safe_distance = 0
                astar = AStarPlanner(safe_distance=safe_distance)
                # do some preprocessing on self.map
                processed_map = np.where(self.map < 35, 0,
                                    np.where(self.map <= 60, 0, 100))

                processed_map[*self.goal] = 0 # idk astar says so
                processed_drone_position = [int(round(self.drone_position[0], 0) - self.origin_x),
                                            int(round(self.drone_position[1], 0) - self.origin_y)]
                processed_goal_position = [int(round(self.goal[0], 0) - self.origin_x),
                                            int(round(self.goal[1], 0) - self.origin_y)]
                
                self.get_logger().info(f"Position:{processed_drone_position}, Goal:{processed_goal_position}")
                self.get_logger().info(f"{processed_map}")
                self.get_logger().info(f"{self.map}")

                self.path = astar.plan(
                    processed_map, 
                    processed_drone_position, 
                    processed_goal_position
                    )
                if self.path is not None:
                    # publish path if you want
                    self.path = np.array(self.path)
                    self.path[:, 0] = self.path[:, 0] + self.origin_x
                    self.path[:, 1] = self.path[:, 1] + self.origin_y

                    self.p_path.data = np.reshape(self.path,-1)
                    self.path_pub.publish(self.p_path)

                    self.sent_position = [int(self.path[1][0]), int(self.path[1][1])]
                    self.get_logger().info(f"Moving to: {self.sent_position}")
                    self.get_logger().info(f"PATH:")
                    for pos in self.path:
                        self.get_logger().info(f"{pos}")
                    if self.map[int(self.sent_position[0]-self.origin_x), int(self.sent_position[1]-self.origin_y)] == -1:
                        # use service use_key
                        self.use_key(*self.sent_position)
                        time.sleep(3)
                        self.state = States.OPENING_DOORS
                        self.recieved_og = True
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
                if self.recieved_og:
                    if self.map[int(self.sent_position[0]-self.origin_x), int(self.sent_position[1]-self.origin_y)] == -1:
                        self.recieved_og = False
                        msg = Bool()
                        msg.data = True
                        self.reset_lidar_pub.publish(msg)
                    else:
                        msg = Vector3()
                        msg.x = float(self.sent_position[0])
                        msg.y = float(self.sent_position[1])
                        msg.z = float(3)
                        self.position_pub.publish(msg)
                        self.state = States.MOVING_TO_WAYPOINT
                else:
                    pass
    
        


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