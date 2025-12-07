#THIS SCRIPT IS FOR COLLISION AVOIDANCE AND DISCOVERING THE MAP

#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Bool
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
        self.occupancy_grid = [50] * (self.map_width * self.map_height)
        
        self.max_n_readings = 100 # number of readings until code decides what is correct
        self.n_readings = 999
        self.reset_lidar_readings()

        # setup OccupancyGrid message
        self.og_msg = OccupancyGrid()
        self.og_msg.header.frame_id = 'world' 
        self.og_msg.info.resolution = self.resolution
        self.og_msg.info.width = self.map_width
        self.og_msg.info.height = self.map_height
        self.og_msg.info.origin.position.x = (-self.map_width * self.resolution / 2)+0.5
        self.og_msg.info.origin.position.y = (-self.map_height * self.resolution / 2)+0.5

        #some thing that the pub needs for some reason
        # lol what
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Tower goal subscriber to topic /uav/input/goal
        # will wait until something is published here
        self.goal_sub = self.create_subscription(
            Vector3,
            '/uav/input/goal',
            self.transformed_goal_callback,
            1
        )
        self.goal = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.scan_sub = self.create_subscription(LaserScan, '/uav/sensors/lidar', self.lidar_callback, 1)
        self.gps_sub = self.create_subscription(PoseStamped, '/uav/sensors/gps', self.gps_callback, 5)
        self.reset_lidar_sub = self.create_subscription(Bool,'/reset_lidar',self.reset_lidar_callback,1)


        self.get_logger().info(f"width: {self.map_width}, height: {self.map_height}")
        self.print_occupancy_grid()
        
        #PROOF WORLD TO GRID IS WRONG
        #goal_pos = self.world_to_grid(3.5,3.5)
        #self.occupancy_grid[goal_pos] = -3
        self.reset_lidar_timer = self.create_timer(10.0, self.reset_lidar_periodic)

    def reset_lidar_periodic(self):
        return
        self.get_logger().info("Periodic reset of lidar readings")
        self.reset_lidar_readings()

    def transformed_goal_callback(self, msg):
        #self.get_logger().info(f"Recieved goal at: {msg.x},{msg.y}")
        self.mark_goal(msg.x, msg.y)

    def gps_callback(self, msg):
        self.robot_x = msg.pose.position.y
        self.robot_y = msg.pose.position.x

    def reset_lidar_callback(self, msg):
        if msg.data:
            self.get_logger().info("reset lidar!")
            self.reset_lidar_readings()

    def world_to_grid(self, x, y):
        #gives coords, returns occupancy grid position
        gx = int((x - self.og_msg.info.origin.position.x) / self.resolution + 0.5)
        gy = int((y - self.og_msg.info.origin.position.y) / self.resolution + 0.5)
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return None
        return gy * self.map_width + gx

    def reset_lidar_readings(self):
        # intensities will always be the sfmarkame, just need
        # to make sure the distance reading is correct
        self.lidar_ranges = np.empty((16, self.max_n_readings), np.float32)
        self.n_readings = 0

    def lidar_callback(self, msg):
        if self.n_readings >= self.max_n_readings:
            return # trash
        
        # kind of spaghetti but assuming these don't change this is probably fine
        self.angle_min = msg.angle_min
        self.range_max = msg.range_max
        self.angle_increment = msg.angle_increment

        self.lidar_ranges[:, self.n_readings] = np.array(msg.ranges)
        self.n_readings += 1
        # update og if number of readings is reached
        if self.n_readings == self.max_n_readings:
            self.update_occupancy_grid()

    def update_occupancy_grid(self):
        ranges = np.mean(self.lidar_ranges, axis=1)
        readings = np.where(
            np.isfinite(self.lidar_ranges) & (self.lidar_ranges > 0),
            self.lidar_ranges,
            np.nan
        )
        noise = np.nanstd(readings, axis=1)
        
        noise_sorted = np.sort(noise)
        baseline = np.average(noise_sorted[:int(len(noise_sorted) * 0.5)])
        
        # Try multiple thresholds
        door_readings = np.where(noise > 3 * baseline)[0]

        angle = self.angle_min
        range_max = self.range_max

        #allowed_ranges = [0,2,3,4,6,7,8,10,11,12,14,15]
        allowed_ranges = [3,7,11,15]
        for n, r in enumerate(ranges):
            if not n in allowed_ranges:
                angle += self.angle_increment
                continue

            out_of_range = np.isinf(r)
            r = min(r, range_max)

            hit_x = self.robot_x + (r+0.1) * np.sin(angle)
            hit_y = self.robot_y + (r+0.1) * np.cos(angle)

            # Ray trace to mark free space
            incr = 0.2
            for sub in np.arange(0, r, incr):
                px = self.robot_x + sub * np.sin(angle)
                py = self.robot_y + sub * np.cos(angle)
                idx = self.world_to_grid(px, py)
                # Protect ALL special values (negative values)
                if idx is not None and self.occupancy_grid[idx] != -3:
                    v = self.occupancy_grid[idx]
                    self.occupancy_grid[idx] = 0 * 0.1 + v * 0.9

            if not out_of_range:
                # Mark obstacle at hit point
                idx = self.world_to_grid(hit_x, hit_y)
                # Also protect special values here
                if idx is not None and self.occupancy_grid[idx] >= 0:
                    if n in door_readings:  # door
                        x = idx % self.map_width
                        y = idx // self.map_width
                        if (x+y)%2==1:
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.get_logger().info(f"SET DOOR AT CELL: {idx}")
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.get_logger().info(f"-----------------------------------------------------------------")
                            self.occupancy_grid[idx] = -1
                    else:  # obstacle
                        current_val = self.occupancy_grid[idx]
                        self.occupancy_grid[idx] = 100 * 0.8 + current_val * 0.2

            angle += self.angle_increment

        self.publish_grid()

    def mark_open_door(self, x, y):
        cell = self.world_to_grid(x, y)
        if cell is not None:
            self.occupancy_grid[cell] = -2
            #self.publish_grid()

    def mark_goal(self, x, y):
        cell = self.world_to_grid(x, y)
        if cell is not None:
            self.occupancy_grid[cell] = -3
            #self.get_logger().info(f"set goal at cell:{cell}")
            #self.publish_grid()

    def print_occupancy_grid(self):
        h = self.map_height
        w = self.map_width
        arr = np.array(self.occupancy_grid, dtype=np.int8).reshape((h, w))
        arr = np.rot90(arr)  # Rotate 90° counter-clockwise
        self.get_logger().info("\n" + np.array2string(arr))

    def publish_grid(self):
        #self.print_occupancy_grid()
        self.og_msg.header.stamp = self.get_clock().now().to_msg()
        self.og_msg.data = [int(v) for v in self.occupancy_grid] #need to convert to ints for publishing
        self.map_pub.publish(self.og_msg)
        #self.get_logger().info("publish grid")
    

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