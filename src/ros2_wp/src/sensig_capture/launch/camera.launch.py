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

    camera1 = LifecycleNode(
        package='sensig_capture',
        executable='camera_node',
        name='camera1_node',
        namespace='camera1',
        parameters=[config_path],
        output='log',
        autostart=True
    )

    camera2 = LifecycleNode(
        package='sensig_capture',
        executable='camera_node',
        name='camera2_node',
        namespace='camera2',
        parameters=[config_path],
        output='log',
        autostart=True
    )

    return LaunchDescription([
        camera1,
        camera2
    ])
