import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    """Client example that periodically calls the server"""

    def __init__(self):
        "Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('minimal_client')

        # Create a client object
        self._client = self.create_client(AddTwoInts, 'add_ints')

        # Wait for service
        while not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')

        # Periodically call method
        self._timer = self.create_timer(2.0, self._timer_callback)

    def _timer_callback(self):
        """Send request to server asking it to add two integers"""

        # Fill out request message
        req = AddTwoInts.Request()
        req.a = random.randint(0, 10)
        req.b = random.randint(0, 10)

        # Send request to server and set callback
        self.future = self._client.call_async(req)
        self.future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        """Log result when received from server"""
        try:
            resp = future.result()
            self.get_logger().info(f"Result: {resp.sum}")
        except Exception as e:
            self.get_logger().error(f"{str(e)}")

def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = MinimalClient()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
