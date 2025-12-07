# THIS SCRIPT HANDLES THE FINITE STATE MACHINE, UNLOCKS DOORS, AND EXPLORATION

#!/usr/bin/env python
import copy
import time
from enum import Enum
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point
from std_msgs.msg import Bool, Int32

from environment_controller.srv import UseKey

class KeyManager(Node):

    # Node initialization
    def __init__(self):
        super().__init__('key_manager')
        #time.sleep(10)

        # key stuff
        self.keys_remaining = 4
        self.get_keys_sub = self.create_subscription(Int32, '/keys_remaining',self.get_keys_remaining, 1)
        self.use_key_client = self.create_client(UseKey, 'use_key')

        #self.use_key(1, 0)


        self.rate = 20
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def get_keys_remaining(self, msg):
        self.keys_remaining = msg.data

    def get_gps(self, msg):
        self.drone_position = msg.pose.position

    def use_key(self, x, y):
        if self.keys_remaining > 0:
            req = UseKey.Request()
            req.door_loc = Point(x=float(x), y=float(y), z=0.0)

            future = self.use_key_client.call_async(req)
            future.add_done_callback(self.use_key_callback)
            rclpy.spin_until_future_complete(self, future)

            self.get_logger().info(f"Attempting to open door at ({x},{y})...")
        else:
            self.get_logger().info("no keys left")
    
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

    # The main loop of the function
    def mainloop(self):
        pass

def main():
    rclpy.init()
    try:
        rclpy.spin(KeyManager())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()