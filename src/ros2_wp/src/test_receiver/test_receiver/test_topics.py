import debugpy
import time
import rclpy
from rclpy.node import Node
from dual_camera_msgs.msg import ImagePair # type: ignore
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from sensor_msgs.msg import CompressedImage
import cv2

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.qos_profile = qos_profile_sensor_data
        self.subscrier_left_ = self.create_subscription(CompressedImage,'/cam_left/frame', self.left_callback, self.qos_profile)
        # self.subscrier_right_ = self.create_subscription(ImagePair,'/cam_right/frame', self.listener_callback, self.qos_profile)
    
    def left_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * pow(10,-9)
        id = msg.header.frame_id

        #self.get_logger().info(f"Frame {id} at {timestamp}, now: {time.time()}")
        time_0 = time.time()
        try:
            np_arr_left = np.frombuffer(msg.data, dtype=np.uint8)
            frame_left = cv2.imdecode(np_arr_left, cv2.IMREAD_COLOR)
            if frame_left is None:
                    self.get_logger().error("Can't decode frame")
                    return
            cv2.imshow("Stereo Stream", frame_left)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

        #self.get_logger().info(f"Decoded frame in { (time.time() - time_0 )*1000 }ms")
        delta_receive = round((time.time() - timestamp)*1000, 2)
        delta_process = round((time.time() - time_0 )*1000 , 2)
        self.get_logger().info(f"{id}, R: {delta_receive}ms, P: {delta_process}ms")
    def listener_callback(self, msg):
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

def main(args=None):
    rclpy.init(args=args)
    #debug()

    node = TestSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
def debug():
    debugpy.listen(("0.0.0.0", 5678))
    debugpy.wait_for_client()
    debugpy.breakpoint()