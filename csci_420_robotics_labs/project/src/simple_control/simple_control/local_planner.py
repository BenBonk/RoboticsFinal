#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Vector3 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        # Timer
        self.timer = self.create_timer(0.2, self.mainloop)

        # grid configuration
        self.declare_parameter('map_width', 20)
        self.declare_parameter('map_height', 20)

        # read them
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.resolution = 1.0

        self.robot_x = 0.0
        self.robot_y = 0.0

        # create occupancy grid initialized to 50
        self.grid = [50] * (self.map_width * self.map_height)

        # setup OccupancyGrid message
        self.og_msg = OccupancyGrid()
        self.og_msg.header.frame_id = 'world' 
        self.og_msg.info.resolution = self.resolution
        self.og_msg.info.width = self.map_width
        self.og_msg.info.height = self.map_height
        self.og_msg.info.origin.position.x = -self.map_width * self.resolution / 2
        self.og_msg.info.origin.position.y = -self.map_height * self.resolution / 2

        #some thing that the pub needs for some reason
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.scan_sub = self.create_subscription(LaserScan, '/uav/sensors/lidar', self.lidar_callback, 10)
        self.gps_sub = self.create_subscription(Vector3, '/uav/sensors/gps', self.gps_callback, 10)

    def gps_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        # Yaw stays 0 per project assumptions

    def world_to_grid(self, x, y):
        # we have to do this because grid and world coords are different
        gx = int((x - self.og_msg.info.origin.position.x) / self.resolution)
        gy = int((y - self.og_msg.info.origin.position.y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return None
        return gy * self.map_width + gx

    def lidar_callback(self, msg):
        angle = msg.angle_min

        for r in msg.ranges:
            if np.isinf(r) or r <= 0:
                angle += msg.angle_increment
                continue

            hit_x = self.robot_x + r * np.cos(angle)
            hit_y = self.robot_y + r * np.sin(angle)

            steps = int(r / self.resolution)
            for i in range(steps):
                px = self.robot_x + (i * self.resolution) * np.cos(angle)
                py = self.robot_y + (i * self.resolution) * np.sin(angle)
                idx = self.world_to_grid(px, py)
                if idx is not None:
                    self.grid[idx] = max(0, self.grid[idx] - 5)  # more certain free

            idx = self.world_to_grid(hit_x, hit_y)
            if idx is not None:
                self.grid[idx] = min(100, self.grid[idx] + 10)  # more certain occupied

            angle += msg.angle_increment

        self.publish_grid()

    def publish_grid(self):
        self.og_msg.header.stamp = self.get_clock().now().to_msg()
        self.og_msg.data = self.grid
        self.map_pub.publish(self.og_msg)

    def mainloop(self):
        pass


def main():
    rclpy.init()
    try:
        rclpy.spin(LocalPlanner())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()