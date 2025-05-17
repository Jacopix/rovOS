import debugpy
import rclpy
import rclpy.client
from rclpy.parameter import Parameter
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
from rclpy.callback_groups import ReentrantCallbackGroup
from functools import partial
from dataclasses import dataclass


@dataclass
class LifeCycleNode:
    name: str
    getter_client: rclpy.client.Client
    setter_client: rclpy.client.Client
    current_state: int
    expected_state: int
    pending_state: int


class LifecycleCameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager_node')

        self.declare_parameter('managed_nodes', Parameter.Type.STRING_ARRAY)
        self.node_to_monitor = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        self.get_logger().info(f'Managing nodes: {self.node_to_monitor}')

        self.getter_clients = {}
        self.setter_clients = {}
        self.current_state = {}
        self.expected_state = {}
        self.pending_transition = {}

        for node_name in self.node_to_monitor:
            getter_client = self.create_client(GetState, f'{node_name}/get_state')
            setter_client = self.create_client(ChangeState, f'{node_name}/change_state')

            self.getter_clients[node_name] = getter_client
            self.setter_clients[node_name] = setter_client
            self.pending_transition[node_name] = None

            self.current_state[node_name] = None
            self.expected_state[node_name] = State.PRIMARY_STATE_ACTIVE
            self.create_timer(0.5, partial(self.check_state_loop, node_name), callback_group=ReentrantCallbackGroup())



    def check_state_loop(self, node_name):
        client = self.getter_clients[node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {node_name}/get_state not available.')
            return

        req = GetState.Request()
        self.my_future = client.call_async(req)
        self.my_future.add_done_callback(partial(self.update_state, node_name=node_name))
        if( self.current_state[node_name] ):
            current = self.current_state[node_name].id
            expected = self.expected_state[node_name]
            if current != expected and not self.pending_transition[node_name]:
                try:
                    self.change_state(expected, node_name)
                except Exception as e:
                    self.get_logger().error(f'ERRORE durante TRANSIZIONE')
            


    def update_state(self, future, node_name):
        if future.result() is not None:
            self.current_state[node_name] = future.result().current_state
            if self.current_state[node_name].id == self.pending_transition[node_name]:
                self.pending_transition[node_name] = None
        else:
            self.get_logger().error(f'Failed to get lifecycle state for {node_name}')

    def change_state(self, new_state, node_name):
        current_state = self.current_state[node_name].id

        if abs(new_state - current_state == 2):
            self.change_state(State.PRIMARY_STATE_INACTIVE, node_name)

        if new_state == State.PRIMARY_STATE_ACTIVE:
            self.query_transition(Transition.TRANSITION_ACTIVATE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_INACTIVE and new_state < current_state:
            self.query_transition(Transition.TRANSITION_DEACTIVATE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_INACTIVE and new_state > current_state:
            self.query_transition(Transition.TRANSITION_CONFIGURE, node_name, new_state)
        elif new_state == State.PRIMARY_STATE_UNCONFIGURED:
            self.query_transition(Transition.TRANSITION_CLEANUP, node_name, new_state) 


    def query_transition(self, transition, node_name, new_state):
        if self.pending_transition[node_name]:
            return
        req = ChangeState.Request()
        req.transition.id = transition
        self.pending_transition[node_name] = new_state
        self.start_transition_timeout(node_name)
        future = self.setter_clients[node_name].call_async(req)

        def change_callback(future):
            if future.result() is not None:
                self.get_logger().info(f"{node_name}: cambiato stato con successo.")
            else:
                self.get_logger().error(f"{node_name}: fallito il cambio di stato.")

        future.add_done_callback(change_callback)    

    
    def clear_pending_transition(self, node_name):
        self.get_logger().warn(f"Timeout transizione per {node_name}")
        self.pending_transition[node_name] = None

    def start_transition_timeout(self, node_name, timeout_sec=3.0):
        def timer_callback():
            self.clear_pending_transition(node_name)
            timer.cancel()  # Disattiva il timer (one-shot)

        timer = self.create_timer(timeout_sec, timer_callback)




def main(args=None):

    

    # Attacca il debugger (una sola volta)
    debugpy.listen(("0.0.0.0", 5678))
    print("✅ In attesa di connessione debugger su porta 5678...")
    debugpy.wait_for_client()
    debugpy.breakpoint()  # opzionale, primo breakpoint

    rclpy.init(args=args)
    node = LifecycleCameraManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
