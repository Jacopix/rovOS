from typing import cast
from functools import partial
from dataclasses import dataclass
from typing import Optional, Dict
import debugpy

import rclpy
import rclpy.client
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.timer import Timer

from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State


'''

'''
@dataclass
class ManagedNode:
    name: str
    getter_client: rclpy.client.Client
    setter_client: rclpy.client.Client
    current_state: Optional[int] = None
    expected_state: Optional[int] = None
    pending_state: Optional[int] = None
    transition_timer: Optional[Timer] = None
    waiting_state_update: bool = False

    def reset_transition_timer(self):
        if self.transition_timer is not None:
            self.transition_timer.destroy()
            self.transition_timer = None


class LifecycleCameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager_node')

        # Get all managed LifeCycleNodes
        self.declare_parameter('managed_nodes', Parameter.Type.STRING_ARRAY)
        self.node_to_monitor = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        self.get_logger().info(f'Managing nodes: {self.node_to_monitor}')

        self.callback_group = ReentrantCallbackGroup()


        # Dictionary of ManagedNodes
        self.nodes: Dict[str, ManagedNode] = {}

        # Initialize managed nodes info
        for node_name in self.node_to_monitor:
            self.nodes[node_name] = ManagedNode(
                node_name, 
                self.create_client(GetState, f'{node_name}/get_state'), 
                self.create_client(ChangeState, f'{node_name}/change_state'), 
                None, 
                State.PRIMARY_STATE_ACTIVE, 
                None
                )
            
            # Check nodes state every second
            self.create_timer(1, partial(self.check_state_loop, node_name), callback_group=self.callback_group)
        

    def check_state_loop(self, node_name):
        client = self.nodes[node_name].getter_client

        # Check if service is available
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {node_name}/get_state not available.')
            return

        # Get current state
        req = GetState.Request()
        future = client.call_async(req)
        future.add_done_callback(partial(self.update_state, node_name=node_name))
            

    def update_state(self, future, node_name):

        if future.result() is not None:
            # Update current state
            self.nodes[node_name].current_state = future.result().current_state.id
            self.nodes[node_name].waiting_state_update = False
            self.get_logger().info(f'Updating state for camera {node_name}')

            # Pending state is reached. Reset pending state and stop transition_timer
            if self.nodes[node_name].current_state == self.nodes[node_name].pending_state:
                self.get_logger().info("Clearing pending state on update_state")
                self.nodes[node_name].pending_state = None
                self.nodes[node_name].reset_transition_timer()
        else:
            self.get_logger().error(f'Failed to update lifecycle state for {node_name}')
        
        # Check if current_state is set
        if( self.nodes[node_name].current_state is not None):
            current = self.nodes[node_name].current_state
            expected = self.nodes[node_name].expected_state
            pending = self.nodes[node_name].pending_state
            is_waiting = self.nodes[node_name].waiting_state_update
        
            # If current state is not as expected and there are no pending transition, change state
            if current != expected and not pending and not is_waiting:
                try:
                    self.get_logger().info(f'Quering transition {node_name}')
                    self.change_state(expected, node_name)
                except Exception as e:
                    self.get_logger().error(f'Transition Error: {e}')

    def change_state(self, new_state, node_name):
        current_state = self.nodes[node_name].current_state

        # Expected state is "2 states" away from current state. Must call a 'configure' transition first
        if abs(new_state - current_state) == 2:
            self.change_state(State.PRIMARY_STATE_INACTIVE, node_name)
        
        # All possible transition given the expected state
        if new_state == State.PRIMARY_STATE_ACTIVE:
            self.query_transition(Transition.TRANSITION_ACTIVATE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_INACTIVE and new_state < current_state:
            self.query_transition(Transition.TRANSITION_DEACTIVATE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_INACTIVE and new_state > current_state:
            self.query_transition(Transition.TRANSITION_CONFIGURE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_UNCONFIGURED:
            self.query_transition(Transition.TRANSITION_CLEANUP, node_name, new_state) 
    
    def query_transition(self, transition, node_name, new_state):

        # If there's a pending state, fail silently
        if self.nodes[node_name].pending_state:
            self.get_logger().info(f'Still waiting for transition to {self.nodes[node_name].pending_state}')
            return
        
        self.get_logger().info(f'Sending request for transition to {new_state}')

        # Query transition and initialize pending state with the expected state
        req = ChangeState.Request()
        req.transition.id = transition
        self.nodes[node_name].pending_state = new_state

        # If no response is received in "time_limit" seconds, reset pending_state and try again
        time_limit = 5.0
        self.start_transition_timeout(node_name, time_limit)

        future = self.nodes[node_name].setter_client.call_async(req)

        def change_callback(future):
            # Received response from transition query
            if future.result() is not None:
                # Response is successfull. Pending state will be reset later on update_state
                if cast(ChangeState.Response, future.result()).success:
                    self.get_logger().info(f"{node_name}: Transition to {new_state} successfull")
                    self.nodes[node_name].waiting_state_update = True
                # Failed transition. Reset pending state to allow other attempts
                else:
                    self.get_logger().error(f"{node_name}: Transition to {new_state} failed")
                    self.get_logger().info("Clearing pending state on transition error")
                    self.nodes[node_name].pending_state = None
                
                # If a valid response is received, always reset transition_timer
                self.nodes[node_name].reset_transition_timer()

        future.add_done_callback(change_callback)    

    def start_transition_timeout(self, node_name, timeout_sec=3.0):

        # Transition took too long. Reset and try again
        def timer_callback():
            self.get_logger().warn(f"{node_name}: Transition timeout, trying again")
            self.get_logger().info("Clearing pending state on transition timeout")
            self.nodes[node_name].pending_state = None
            self.nodes[node_name].reset_transition_timer()

        self.nodes[node_name].transition_timer = self.create_timer(timeout_sec, timer_callback)

def main(args=None):


    rclpy.init(args=args)
    #debug()
    node = LifecycleCameraManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# Debug function to user vscode debugger
def debug():
    debugpy.listen(("0.0.0.0", 5678))
    debugpy.wait_for_client()
    debugpy.breakpoint()