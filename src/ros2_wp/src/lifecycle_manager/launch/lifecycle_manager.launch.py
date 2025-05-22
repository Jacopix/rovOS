from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


    # Get launcher file for cameras
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensig_capture'),
                'launch',
                'camera.launch.py'
            )
        )
    )

    # Get lifecycle manager node
    lifecycle_manager = Node(
        package='lifecycle_manager', 
        executable='LifecycleCameraManager',
        name='camera_manager_node',
        output='screen',
        parameters=[{
            'managed_nodes': ['/cam_left/cam_left_node', '/cam_right/cam_right_node'] 
        }]
    )

    return LaunchDescription([
        camera_launch,
        lifecycle_manager,
    ])
