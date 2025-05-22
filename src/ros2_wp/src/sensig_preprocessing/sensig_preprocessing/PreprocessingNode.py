import rclpy 
from rclpy.node import Node
import cv2 
from cv_bridge import CvBridge 
import rclpy.qos
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from dual_camera_msgs.msg import ImagePair  # type: ignore

class PreprocessingNode(Node):
    def __init__(self):
        super().__init__('preprocessing_node') 
        # Setup publisher
        self.qos_profile = qos_profile_sensor_data
        self.publisher_ = self.create_publisher(ImagePair, '/stereo/image_pair', self.qos_profile)
        
        self.counter = 0

        self.r_subscriber_ = Subscriber(self, CompressedImage, '/cam_right/frame', qos_profile=self.qos_profile)
        self.l_subscriber_ = Subscriber(self, CompressedImage, '/cam_left/frame', qos_profile=self.qos_profile)

        queue_size = 1
        max_delay = 0.1
        self.sync = ApproximateTimeSynchronizer(
            [self.l_subscriber_, self.r_subscriber_],
            queue_size,
            max_delay
        )
        self.sync.registerCallback(self.SyncCallback)

    def SyncCallback(self, left_frame, right_frame):
        self.counter += 1
        msg = ImagePair()
        msg.left = left_frame
        msg.right = right_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.counter)
        self.get_logger().info("Test")
        self.publisher_.publish(msg)

    # def timer_callback(self, msg):
    #     return    

def main(args=None):

    # Instantiate camera node
    rclpy.init(args=args)
    node = PreprocessingNode()
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()