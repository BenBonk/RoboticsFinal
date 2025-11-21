#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
from tf2_ros import TransformException
import time
import copy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler

class TowerToMap(Node):

    def __init__(self):
        time.sleep(10)
        super().__init__('TowerToMapNode')

        self.goal = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
        self.goal_pub = self.create_publisher(Vector3, '/uav/input/goal', 1)
        self.goal_sub = self.create_subscription(Vector3, '/cell_tower/position', self.tower_goal_callback, 1)

        self.rate = 2
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def tower_goal_callback(self, msg):
        self.goal = msg

    def mainloop(self):
        if self.goal:
            try:
                try:
                    self.t = self.tf_buffer.lookup_transform('cell_tower', 'world', rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(f'Could not transform world to tower: {ex}')

                point_stamped = PointStamped()
                point_stamped.header.frame_id = 'cell_tower'
                point_stamped.header.stamp = self.get_clock().now().to_msg()
                point_stamped.point.x = self.goal.x
                point_stamped.point.y = self.goal.y
                point_stamped.point.z = self.goal.z

                new_point = do_transform_point(point_stamped, self.t)
              
                msg = Vector3()
                msg.x = new_point.point.x
                msg.y = new_point.point.y
                msg.z = new_point.point.z
                
                self.get_logger().info(f'Publishing Transformed Goal: {msg.x}, {msg.y}')
                self.goal_pub.publish(msg)

                self.goal = None

            except TransformException as ex:
                self.get_logger().info(f'Error getting the tower transformation')


def main():
    rclpy.init()
    try:
        rclpy.spin(TowerToMap())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()