#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
from turtlesim.msg import Pose

from my_py_pkg import util

class PoseBroadcaster(Node):
    """Node that broadcasts all given turtle transform frames"""

    def __init__(self):
        """Constructor"""
        super().__init__('pose_broadcaster')

        # Declare parameters
        self.declare_parameter('turtle_names', ['turtle1'])
        self.declare_parameter('broadcast_period', 0.1)

        # Set up TF2 broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Remember poses
        self._poses = {}

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

        # Create subscriptions for all turtle poses
        for name in self.get_parameter('turtle_names').value:
            self.create_subscription(
                Pose,
                f"/{name}/pose",
                lambda msg, turtle=name: self._pose_callback(msg, turtle),
                10,
            )

        # Publish transforms periodically
        self.create_timer(
            self.get_parameter('broadcast_period').value,
            self._broadcast,
        )

        # Show that the node has started
        self.get_logger().info(f"Transform broadcaster node has started")

    def _pose_callback(self, msg, turtle):
        """Record the current pose of the given turtle"""
        self._poses[turtle] = msg

    def _broadcast(self):
        """Broadcast the frames of all the turtles"""
        for name, pose in self._poses.items():

            # Construct transform message with header data
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = name
            
            # Add translation data
            t.transform.translation.x = pose.x
            t.transform.translation.y = pose.y
            t.transform.translation.z = 0.0

            # Calculate quaternion from euler
            q = util.euler_to_quaternion(0, 0, pose.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # Broadcast the frame
            self._tf_broadcaster.sendTransform(t)

def main():
    try:
        rclpy.init()
        node = PoseBroadcaster()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
