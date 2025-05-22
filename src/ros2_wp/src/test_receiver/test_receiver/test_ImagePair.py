import rclpy
import time
from rclpy.node import Node
from dual_camera_msgs.msg import ImagePair # type: ignore
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.qos_profile = qos_profile_sensor_data
        self.subscrier_ = self.create_subscription(ImagePair,'/stereo/image_pair', self.listener_callback, self.qos_profile)
    
    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * pow(10,-9)
        id = msg.header.frame_id
        time_0 = time.time()
        try:
            np_arr_left = np.frombuffer(msg.left.data, dtype=np.uint8)
            frame_left = cv2.imdecode(np_arr_left, cv2.IMREAD_COLOR)

            np_arr_right = np.frombuffer(msg.right.data, dtype=np.uint8)
            frame_right = cv2.imdecode(np_arr_right, cv2.IMREAD_COLOR)

            if frame_left is None or frame_right is None:
                self.get_logger().error("Can't decode frame")
                return
            
            combined = np.hstack((frame_left,frame_right))

            cv2.imshow("Stereo Stream", combined)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

        delta_receive = round((time.time() - timestamp)*1000, 2)
        delta_process = round((time.time() - time_0 )*1000 , 2)
        self.get_logger().info(f"{id}, R: {delta_receive}ms, P: {delta_process}ms")

def main(args=None):
    rclpy.init(args=args)

    node = TestSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()