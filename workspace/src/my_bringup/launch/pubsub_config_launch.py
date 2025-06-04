from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch multiple nodes"""

    # Create a dynamic object that will have a value at runtime
    config_file = LaunchConfiguration('config_file')

    # Declare a launch argument for the YAML config file name
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='pubsub_debug.yaml',
        description="Path to the YAML config file"
    )

    # Build the full path to the config file at runtime
    config_path = PathJoinSubstitution([
        FindPackageShare('my_bringup'),
        'config',
        config_file,
    ])

    # Create a launch description
    ld = LaunchDescription()
    nodes = []

    # Publisher 1 node
    nodes.append(Node(
        package='my_cpp_pkg',
        executable='publisher_with_params',
        namespace='talkie',
        name='publisher_1',
        parameters=[config_path]
    ))

    # Publisher 2 node
    nodes.append(Node(
        package='my_cpp_pkg',
        executable='publisher_with_params',
        namespace='talkie',
        name='publisher_2',
        parameters=[config_path]
    ))

    # Subscriber node
    nodes.append(Node(
        package='my_py_pkg',
        executable='minimal_subscriber',
        namespace='talkie',
        name='subscriber_1'
    ))

    # Add argument(s) to launch description
    ld.add_action(config_file_arg)

    # Add nodes to launch description
    for node in nodes:
        ld.add_action(node)

    return ld
