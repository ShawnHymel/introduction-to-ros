import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from my_interfaces.msg import Accelerometer

class AccPublisher(Node):
    """Publisher example that periodically sends out dummy accelerometer data"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('acc_publisher')

        # Create a publisher object
        self._publisher = self.create_publisher(Accelerometer, 'my_acc', 10)

        # Periodically call method
        self._timer = self.create_timer(0.5, self._timer_callback)

    def _timer_callback(self):
        """Publishes a simple message to topic"""

        # Fill out String message
        msg = Accelerometer()
        msg.x = 0.5
        msg.y = 0.1
        msg.z = -9.8

        # Publish message to topic
        self._publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = AccPublisher()
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
