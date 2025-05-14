import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class MinimalServer(Node):
    """Server example that adds two integers and responds with result"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('minimal_server')

        # Create a service object
        self._srv = self.create_service(
            AddTwoInts,
            'add_ints',
            self._server_callback
        )

    def _server_callback(self, req, resp):
        """Responds with sum of request integers"""
        resp.sum = req.a + req.b
        self.get_logger().info(f"Received request: a={req.a}, b={req.b}")

        return resp
    
def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = MinimalServer()
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
