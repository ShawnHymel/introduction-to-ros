import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point
from my_interfaces.srv import TriggerPoints

class TriggerPointsServer(Node):
    """Server that responds with array of random points"""

    def __init__(self):
        """Constructor"""

        # Call the Node class constructor with the node name
        super().__init__('trigger_points_server')

        # Create a service object
        self._srv = self.create_service(
            TriggerPoints,
            'trigger_points',
            self._server_callback
        )

    def _server_callback(self, req, resp):
        """Responds with array of random floats"""
        
        # Log request
        self.get_logger().info(f"Received request: num_points={req.num_points}")

        # Set success based on number of points
        if req.num_points > 0:
            resp.success = True
        else:
            resp.success = False

        # Create num_points number of random points
        resp.points = []
        for _ in range(req.num_points):
            point = Point()
            point.x = random.uniform(-1.0, 1.0)
            point.y = random.uniform(-1.0, 1.0)
            point.z = random.uniform(-1.0, 1.0)
            resp.points.append(point)

        return resp
    
def main(args=None):
    """Main entrypoint"""

    # Initialize and run node
    try:
        rclpy.init()
        node = TriggerPointsServer()
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
