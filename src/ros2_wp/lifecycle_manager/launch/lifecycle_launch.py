from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('sensig_capture'),
        'config',
        'camera_config.yaml'
    )
    print(config_path)
    return LaunchDescription([
        # Avvia prima i nodi lifecycle
        LifecycleNode(
            package='sensig_capture',
            executable='CameraNode',
            name='camera1_node',
            namespace='camera1',
            parameters=[config_path],
            output='screen'
        ),
        # Poi il lifecycle_manager_node
        Node(
            package='lifecycle_manager',
            executable='LifecycleManagerNode',
            name='lifecycle_manager',
            parameters=[{
                'managed_nodes': ['/camera1/camera1_node']
            }]
        ),
    ])
