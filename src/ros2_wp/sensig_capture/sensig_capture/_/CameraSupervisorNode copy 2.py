import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

class CameraSupervisorNode(Node):
    def __init__(self):
        super().__init__('camera_supervisor_node')

        self.camera_node_name = '/camera1/camera1_node'
        self.state_client = self.create_client(GetState, f'{self.camera_node_name}/get_state')
        self.change_state_client = self.create_client(ChangeState, f'{self.camera_node_name}/change_state')

        self.get_logger().info(f"Attendo i servizi...")

        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servizio get_state non disponibile, riprovo...')
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servizio change_state non disponibile, riprovo...')

        self.get_logger().info("Servizi disponibili. Chiedo lo stato iniziale...")
        self.req = GetState.Request()
        self.future = self.state_client.call_async(self.req)
        self.timer = self.create_timer(0.1, self.monitor_node_state)

    def monitor_node_state(self):
        future = self.future

        if future.done():
            current_state = future.result().current_state.label
            self.get_logger().info(f"Stato attuale: {current_state}")

            if current_state == 'unconfigured':
                self.do_transition(Transition.TRANSITION_CONFIGURE)
            elif current_state == 'inactive':
                self.do_transition(Transition.TRANSITION_ACTIVATE)
            elif current_state == 'active':
                self.get_logger().info("Nodo attivo. Supervisione completata.")
                self.destroy_timer(self.timer)
        else:
            self.get_logger().error("Errore nel recupero dello stato")

    def do_transition(self, transition_id: int):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Transizione {transition_id} completata con successo.")
        else:
            self.get_logger().error(f"Transizione {transition_id} fallita.")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
