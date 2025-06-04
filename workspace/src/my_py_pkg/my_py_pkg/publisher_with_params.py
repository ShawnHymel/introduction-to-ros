import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import String

class PublisherWithParams(Node):
    """Publisher example that periodically sends out a string"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('publisher_with_params')

        # Declare parameters
        self.declare_parameter('message', "Hello")
        self.declare_parameter('timer_period', 1.0)

        # Set to attributes
        self._message = self.get_parameter('message').value
        self._timer_period = self.get_parameter('timer_period').value

        # Configure callback for runtime parameter update
        self.add_post_set_parameters_callback(self._post_parameters_callback)

        # Create a publisher object
        self._publisher = self.create_publisher(String, 'my_topic', 10)

        # Periodically call method
        self._timer = self.create_timer(
            self._timer_period, 
            self._timer_callback
        )

    def _timer_callback(self):
        """Publishes a simple message to topic"""

        # Fill out String message
        msg = String()
        msg.data = self._message

        # Publish message to topic
        self._publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

    def _post_parameters_callback(self, params):
        """Set parameter after node started"""

        # Loop through all parameters
        for param in params:

            # Update message
            if param.name == 'message':
                self._message = param.value
                self.get_logger().info(f"Set {param.name} to {param.value}")

            # Update timer
            elif param.name == 'timer_period':

                # Update member variable
                self._timer_period = param.value
                self.get_logger().info(f"Set {param.name} to {param.value}")

                # Reset timer
                self._timer.cancel()
                self._timer = self.create_timer(
                    self._timer_period,
                    self._timer_callback
                )

            # Unknown parameter
            else:
                self.get_logger().warn(f"Unknown parameter: {param.name}")

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = PublisherWithParams()
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
