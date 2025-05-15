import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
import numpy as np
from datetime import datetime
from sensor_msgs.msg import CompressedImage
import os
import cv2

class StereoSyncNode(Node):
    def __init__(self):
        super().__init__('stereo_sync')
        
        # QoS Profile
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Ultimo tempo in cui è arrivato un messaggio
        self.last_time = None

        # Subscriber
        self.subscriber_ = self.create_subscription(
            CompressedImage,
            'ip_stream_image',
            self.listener_callback,
            self.qos_profile
        )

        self.publisher_ = self.create_publisher(Image, 'jpeg_stream', self.qos_profile)
        self.bridge = CvBridge()
        self.output_dir = '/tmp/saved_images'
        os.makedirs(self.output_dir, exist_ok=True)




    def listener_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_cv is None:
            self.get_logger().warn("Failed to decode image")
            return

        # Costruisci il nome del file: timestamp o counter
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"frame_{timestamp}.jpg"
        filepath = os.path.join(self.output_dir, filename)

        # Salva l'immagine JPEG
        success = cv2.imwrite(filepath, image_cv)
        if success:
            self.get_logger().info(f"Saved image: {filepath}")
        else:
            self.get_logger().warn("Failed to save image")

def main(args=None):
    rclpy.init(args=args)
    node = StereoSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
