#!/usr/bin/env python3

import math

from geometry_msgs.msg import PointStamped, Twist
import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformListener, Buffer

class FollowerNode(Node):
    """Node that controls follower turtle"""

    def __init__(self):
        """Constructor"""
        super().__init__('follower_node')

        # Declare parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('turtle_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('leader_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('follow_period', 0.2)

        # Assign attributes from parameters
        self._debug = self.get_parameter('debug').value
        self._turtle_name = self.get_parameter('turtle_name').value
        self._leader_name = self.get_parameter('leader_name').value
        self._follow_distance = self.get_parameter('follow_distance').value

        # Create client and wait for spawn_status service (blocking)
        spawn_status_client = self.create_client(Trigger, 'spawn_status')
        while not spawn_status_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /spawn_status service...")

        # Query spawn_status service until spawning is complete (blocking)
        while True:
            req = Trigger.Request()
            future = spawn_status_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Spawner says we're ready!")
                    break
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # Delay briefly to prevent missed transform broadcast messages
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # Set up transform buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publisher that controls this turtle
        self._cmd_pub = self.create_publisher(
            Twist,
            f"/{self._turtle_name}/cmd_vel", 
            10
        )

        # Periodically follow the leader
        self.create_timer(
            self.get_parameter('follow_period').value,
            self._follow
        )

        # Log that node has started
        self.get_logger().info("Follower node started")

    def _follow(self):
        """Have the current turtle follow behind the given leader turtle"""
        
        # Create a target point 1 unit behind the leader in the leader's frame
        leader_target = PointStamped()
        leader_target.header.frame_id = self._leader_name
        leader_target.header.stamp = rclpy.time.Time().to_msg()
        leader_target.point.x = -1 * self._follow_distance
        leader_target.point.y = 0.0
        leader_target.point.z = 0.0

        # Look up the transform from self to leader and then transform point
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=self._turtle_name,
                source_frame=self._leader_name,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            follower_target = do_transform_point(leader_target, tf)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        # Find distance and angle to the point 1 unit behind leader
        dx = follower_target.point.x
        dy = follower_target.point.y
        dist = math.sqrt((dx ** 2) + (dy ** 2))
        angle = math.atan2(dy, dx)

        # Calculate command to move toward that point
        msg = Twist()
        msg.linear.x = 1.0 * dist
        msg.angular.z = 2.0 * angle
        self._cmd_pub.publish(msg)

        # Print debugging information
        if self._debug:
            self.get_logger().info(f"dx={dx:.2f}, dy={dy:.2f}, " \
                                   f"dist={dist:.2f}, angle={angle:.2f}")

def main():
    try:
        rclpy.init()
        node = FollowerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
