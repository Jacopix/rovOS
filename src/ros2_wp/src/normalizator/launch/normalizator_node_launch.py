from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # Define used names
    name_package = "normalizator"
    name_node = name_package + "_node"

    # Retrieve normalizator parameters from configuration file
    param_normalizator = PathJoinSubstitution([FindPackageShare(name_package), "config", "normalizator_node_config.yaml"])

    # Create and configure node parameters
    launch_norm_node = Node(
        package = name_package,
        executable = name_node,
        name = name_node,
        namespace = name_package,
        parameters = [param_normalizator],
        output = "log",
    )

    # Return node
    return LaunchDescription([
        launch_norm_node
    ])
