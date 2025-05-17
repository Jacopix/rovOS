from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Includi il launch file esistente per la camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensig_capture'),
                'launch',
                'camera.launch.py'
            )
        )
    )

    # (Opzionale) altri nodi lifecycle se vuoi gestirli direttamente
    # ...

    # Nodo manager che riceve i nomi da gestire
    lifecycle_manager = Node(
        package='lifecycle_manager',  # cambia con il tuo pacchetto
        executable='LifecycleCameraManager',
        name='camera_manager_node',
        output='screen',
        parameters=[{
            'managed_nodes': ['/camera1/camera1_node']  # il nome del nodo da gestire
        }]
    )

    return LaunchDescription([
        camera_launch,
        lifecycle_manager,
    ])
