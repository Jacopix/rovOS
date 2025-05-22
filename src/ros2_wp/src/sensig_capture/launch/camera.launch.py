from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('sensig_capture'),
        'config',
        'camera_config.yaml'
    )

    cam_left = LifecycleNode(
        package='sensig_capture',
        executable='camera_node',
        name='cam_left_node',
        namespace='cam_left',
        parameters=[config_path],
        output='log',
        autostart=True
    )

    cam_right = LifecycleNode(
        package='sensig_capture',
        executable='camera_node',
        name='cam_right_node',
        namespace='cam_right',
        parameters=[config_path],
        output='log',
        autostart=True
    )

    return LaunchDescription([
        cam_left,
        cam_right
    ])
