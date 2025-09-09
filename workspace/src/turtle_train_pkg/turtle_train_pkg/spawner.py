#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.srv import Spawn

class SpawnerNode(Node):
    """Spawns follower turtles and notifies when spawning is complete"""

    def __init__(self):
        """Constructor"""
        super().__init__('spawner_node')

        # Declare parameters
        self.declare_parameter('canvas_width', 11.088)
        self.declare_parameter('canvas_height', 11.088)
        self.declare_parameter('seed', 42)
        self.declare_parameter('turtle_names', ['follower0'])

        # Set random seed
        random.seed(self.get_parameter('seed').value)

        # Client to call spawn service
        self._spawn_client = self.create_client(Spawn, 'spawn')
        while not self._spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")

        # Server that can be queried for spawn status
        self._spawn_complete = False
        self.create_service(
            Trigger, 
            'spawn_status', 
            self._spawn_status_callback
        )

        # Spawn follower turtles in random locations
        turtle_names = self.get_parameter('turtle_names').value
        for name in turtle_names:
            x = random.random() * self.get_parameter("canvas_width").value
            y = random.random() * self.get_parameter("canvas_height").value
            theta = random.random() * 2 * math.pi
            self._spawn(f"{name}", x, y, theta)

        # Update spawn status flag
        self._spawn_complete = True

    def _spawn(self, turtle_name, x, y, theta):
        """Spawn a turtle with a given name (blocking)"""
        
        # Construct request
        req = Spawn.Request()
        req.name = turtle_name
        req.x = x
        req.y = y
        req.theta = theta

        # Send request and wait until complete
        future = self._spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Print response
        resp = future.result()
        if resp:
            self.get_logger().info(f"Spawned turtle: {str(resp.name)}")
        else:
            self.get_logger().error("Failed to spawn turtle")

    def _spawn_status_callback(self, request, response):
        """Server that responds with True if spawning is complete"""
        response.success = self._spawn_complete
        response.message = ""
        return response

def main():
    try:
        rclpy.init()
        node = SpawnerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()