import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist

import tf2_ros

class TurtleFollower(Node):
    """Uses TF2 listener to plot course and track main turtle"""

    def __init__(self):
        """Constructor"""
        super().__init__('turtle_follower')

        # Declare parameters
        self.declare_parameter('leader', "turtle1")
        self.declare_parameter('follower', "turtle2")
        self.declare_parameter('period', 0.1)

        # Set to attributes
        self._leader = self.get_parameter('leader').value
        self._follower = self.get_parameter('follower').value
        self._period = self.get_parameter('period').value

        # Create publisher
        self._publisher = self.create_publisher(
            Twist,
            f"/{self._follower}/cmd_vel",
            10
        )

        # Create TF2 listener
        self._tf_buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buf, self)

        # Periodically call the follow function
        self._timer = self.create_timer(self._period, self._follow)

        # Say we're started
        self.get_logger().info(
            f"Follower started: {self._follower} following {self._leader}"
        )

    def _follow(self):
        """Periodically steer the follower toward the leader"""

        # Attempt transform
        try:

            # Look up transform
            tf = self._tf_buf.lookup_transform(
                self._follower,
                self._leader,
                rclpy.time.Time()
            )

            # Figure out x, y difference and angle from leader
            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            angle = math.atan2(dy, dx)
            
            # Steer toward the leader
            twist = Twist()
            twist.linear.x = 2.0 * math.sqrt(dx**2 + dy**2)
            twist.angular.z = 4.0 * angle
            self._publisher.publish(twist)

        # If transform fails
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = TurtleFollower()
        rclpy.spin(node)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
