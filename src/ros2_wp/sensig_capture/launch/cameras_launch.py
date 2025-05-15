from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch.actions import TimerAction
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
        executable='CameraNode',
        name='camera1_node',
        namespace='camera1',
        parameters=[config_path],
        output='screen',
        autostart=True

    )

    supervisor = Node(
        package='sensig_capture',
        executable='CameraSupervisorNode',
        name='camera_supervisor_node',
        namespace='CameraSupervisorNode',
    )
    delayed_supervisor = TimerAction(
        period=2.0,  # aspetta 2 secondi (regolabile)
        actions=[supervisor]
    )


    return LaunchDescription([
        #supervisor,
        #delayed_supervisor,
        camera1,
    ])

#  configure_event
        # forced_configure,
        # configure_event,

'''
return LaunchDescription([
    camera1,
    TimerAction(
        period=0.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(camera1),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ]
    ),
    TimerAction(
        period=1.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(camera1),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )
        ]
    )
])
'''

    # forced_configure = TimerAction(
    # period=0.0,
    #     actions=[
    #         EmitEvent(
    #             event=ChangeState(
    #                 lifecycle_node_matcher=matches_action(camera1),
    #                 transition_id=Transition.TRANSITION_CONFIGURE,
    #             )
    #         )
    #     ]  
    # )

    # # Configura il nodo quando è nello stato 'unconfigured'
    # configure_event = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=camera1,
    #         goal_state='inactive',
    #         entities=[
    #             EmitEvent(
    #                 event=ChangeState(
    #                     lifecycle_node_matcher=matches_action(camera1),
    #                     transition_id=Transition.TRANSITION_ACTIVATE,
    #                 )
    #             )
    #         ],
    #     )
    # )