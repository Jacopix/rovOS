import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class StereoSyncNode(Node):
    def __init__(self):
        super().__init__('stereo_sync')
        
        # QoS Profile (puoi personalizzarlo)
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Ultimo tempo in cui è arrivato un messaggio
        self.last_time = None

        # Subscriber
        self.subscriber_ = self.create_subscription(
            Image,
            'ip_stream_image',
            self.listener_callback,
            self.qos_profile
        )

    def listener_callback(self, msg):
        # Salva immagine
        print("Data: ", len(msg.data), "byte = ",len(msg.data)/1024," kB = ",len(msg.data)/(1024*1024), " MB")

        # Tempo attuale in secondi
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is not None:
            delta = now - self.last_time
            self.get_logger().info(f'Ricevuto un frame dopo {delta*1000:.1f} ms')
        else:
            self.get_logger().info('Ricevuto primo frame')

        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = StereoSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
