import rclpy
from rclpy.parameter import Parameter

from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from rclpy.executors import SingleThreadedExecutor
import time

class LifecycleManagerNode(Node):
    def __init__(self):
        super().__init__('lifecycle_manager_node')

        # Prendi la lista dei nodi dal parametro
        self.declare_parameter('managed_nodes', Parameter.Type.STRING_ARRAY)

        self.managed_nodes = self.get_parameter('managed_nodes').get_parameter_value().string_array_value

        self.get_logger().info(f'Managing nodes: {self.managed_nodes}')

        # Attendi qualche secondo per sicurezza (es: se i nodi stanno ancora partendo)
        time.sleep(2.0)

        for node_name in self.managed_nodes:
            self.manage_node(node_name)

    def manage_node(self, node_name):
        self.get_logger().info(f'--- Managing {node_name} ---')

        if not self.wait_for_service(f'{node_name}/change_state'):
            self.get_logger().error(f'Timeout waiting for {node_name}/change_state')
            return

        if not self.wait_for_service(f'{node_name}/get_state'):
            self.get_logger().error(f'Timeout waiting for {node_name}/get_state')
            return

        # CONFIGURE
        self.get_logger().info(f'Configuring {node_name}...')
        if self.call_change_state(node_name, Transition.TRANSITION_CONFIGURE):
            self.get_logger().info(f'{node_name} configured.')

        # ACTIVATE
        self.get_logger().info(f'Activating {node_name}...')
        if self.call_change_state(node_name, Transition.TRANSITION_ACTIVATE):
            self.get_logger().info(f'{node_name} activated.')

    def wait_for_service(self, service_name, timeout_sec=5.0):
        start = time.time()
        while not self.has_service(service_name):
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.1)
        return True

    def has_service(self, service_name):
        services = self.get_service_names_and_types()
        return any(name == service_name for name, _ in services)

    def call_change_state(self, node_name, transition_id):
        client = self.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {node_name}/change_state not available')
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().error(f'Transition failed for {node_name}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManagerNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
