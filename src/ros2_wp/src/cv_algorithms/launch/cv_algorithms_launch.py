from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    name_package = "cv_algorithms"
    name_node_mono_ir = "mono_ir_node"
    name_node_stereo_ir = "stereo_ir_node"
    name_node_stereo_vo = "stereo_vo_node"

    # Retrieve C.V. algorithms parameters from their configuration files
    param_mono_ir = PathJoinSubstitution([FindPackageShare(name_package), "config", "mono_ir_config.yaml"])
    param_stereo_ir = PathJoinSubstitution([FindPackageShare(name_package), "config", "stereo_ir_config.yaml"])
    param_stereo_vo = PathJoinSubstitution([FindPackageShare(name_package), "config", "stereo_vo_config.yaml"])

    # Create and configure every node parameters
    launch_mono_ir = LifecycleNode(
        package = name_package,
        executable = name_node_mono_ir,
        name = name_node_mono_ir,
        namespace = name_package,
        parameters = [param_mono_ir],
        output = "log",
        autostart = False
    )
    launch_stereo_ir = LifecycleNode(
        package = name_package,
        executable = name_node_stereo_ir,
        name = name_node_stereo_ir,
        namespace = name_package,
        parameters = [param_stereo_ir],
        output = "log",
        autostart = True
    )
    launch_stereo_vo = LifecycleNode(
        package = name_package,
        executable = name_node_stereo_vo,
        name = name_node_stereo_vo,
        namespace = name_package,
        parameters = [param_stereo_vo],
        output = "log",
        autostart = False
    )

    # Return the nodes
    return LaunchDescription([
        launch_mono_ir,
        launch_stereo_ir,
        launch_stereo_vo
    ])
