import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import String

class MinimalSubscriber(Node):
    """Subscriber example that prints messages to the console"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('minimal_subscriber')

        # Create a subscription object
        self._subscription = self.create_subscription(
          String,
          'my_topic',
          self._listener_callback,
          10
        )

    def _listener_callback(self, msg):
      """Prints message to the console"""
      self.get_logger().info(f"Received message: {msg.data}")

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = MinimalSubscriber()
        rclpy.spin(node)

    # Catch ctrl+c or shutdown request
    except (KeyboardInterrupt, ExternalShutdownException):
      pass

    # Destroy node (now) and gracefully exit
    finally:
        if node is not None:
          node.destroy_node()
        if rclpy.ok():
          rclpy.shutdown()

if __name__ == '__main__':
    main()