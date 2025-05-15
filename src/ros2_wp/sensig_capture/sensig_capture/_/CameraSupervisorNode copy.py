import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

class CameraSupervisorNode(Node):
    def __init__(self):
        super().__init__('camera_supervisor_node')
        self.node_name = '/camera1/camera1_node'

        self.get_logger().info('Aspettando che il nodo sia disponibile...')
        self.timer = self.create_timer(1.0, self.control_loop)

    def control_loop(self):
        state = self.get_state()
        if state is None:
            self.get_logger().warn('Nodo non pronto o non esiste ancora.')
            return

        self.get_logger().info(f'Stato attuale: {state}')

        if state == 'unconfigured':
            self.change_state(Transition.TRANSITION_CONFIGURE)
        elif state == 'inactive':
            self.change_state(Transition.TRANSITION_ACTIVATE)
        elif state == 'active':
            self.get_logger().info('Nodo già attivo, supervisor termina.')
            self.timer.cancel()

    def get_state(self):
        client = self.create_client(GetState, f'{self.node_name}/get_state')
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("No service")
            return None
        future = client.call_async(GetState.Request())

        if future.result() is not None:
            self.get_logger().info("No future")
            return future.result().current_state.label
        return None

    def change_state(self, transition_id):
        client = self.create_client(ChangeState, f'{self.node_name}/change_state')
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servizio change_state non disponibile.')
            return

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Transizione {transition_id} inviata con successo.')
        else:
            self.get_logger().error('Transizione fallita.')


def main(args=None):
    rclpy.init(args=args)
    supervisor = CameraSupervisorNode()
    rclpy.spin(supervisor)
    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
