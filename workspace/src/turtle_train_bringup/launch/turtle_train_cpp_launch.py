from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Set up nodes and return a launch description for C++ version"""
    
    # Main turtle name
    lead_turtle = 'turtle1'

    # Configure follower turtle names
    followers = []
    for i in range(10):
        followers.append(f"follower{i}")

    # Base turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    
    # Spawn node (spawn all followers) - using C++ executable
    spawn_node = Node(
        package='turtle_train_cpp',
        executable='spawner',
        parameters=[
            {'seed': 42},
            {'turtle_names': followers},
        ]
    )

    # Transform broadcaster - using C++ executable
    pose_broadcaster_node = Node(
        package='turtle_train_cpp',
        executable='pose_broadcaster',
        parameters=[
            {'turtle_names': [lead_turtle] + followers},
            {'broadcast_period': 0.1},
        ]
    )

    # Create an array of follower nodes - using C++ executable
    follower_nodes = []
    for i, name in enumerate(followers):
        if i == 0:
            leader_name = lead_turtle
        else:
            leader_name = followers[i - 1]
        follower_nodes.append(
            Node(
                package='turtle_train_cpp',
                executable='follower',
                name=f"{name}_follower",
                parameters=[
                    {'turtle_name': name},
                    {'leader_name': leader_name},
                    {'follow_distance': 0.5},
                    {'follow_period': 0.2},
                ]
            )
        )

    # Construct launch description
    ld = LaunchDescription([
        turtlesim_node,
        spawn_node,
        pose_broadcaster_node,
        *follower_nodes
    ])

    return ld