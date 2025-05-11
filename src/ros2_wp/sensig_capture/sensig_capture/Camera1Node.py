# Based on https://github.com/pratikPhadte/my_camera

import rclpy 
from rclpy.node import Node
import cv2 
from cv_bridge import CvBridge  # convert between ROS and OpenCV images
from sensor_msgs.msg import Image 
import time


def get_camera_fps(camera: cv2.VideoCapture, num_frames: int = 120):
    """
    Automaticcaly measures the actual frames per second (FPS) of a video capture device.

    Args:
        camera (cv2.VideoCapture): OpenCV video capture object already opened.
        num_frames (int, optional): Number of frames to capture for FPS calculation. Default is 120.

    Returns:
        float: Estimated frames per second, rounded to 2 decimal places.

    Raises:
        ValueError: If the camera is not opened.
        RuntimeError: If a frame cannot be read during the process.
    """

    if not camera.isOpened():
        raise ValueError("Camera is not opened")

    start = time.time()

    for i in range(num_frames):
        ret, frame = camera.read()
        if( i % 10 == 0):
            print("Read ",i," frames")
        if not ret:
            raise RuntimeError("Failed to read frame from camera")

    end = time.time()

    elapsed = end - start
    fps = round(num_frames / elapsed, 2)
    return fps


class CameraNode(Node):
    def __init__(self, URI):
        super().__init__('ip_stream_node') 
        self.frame_counter = 0 # Used to log publish times every 10 frames (better readability)

        # Setup publisher
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(Image, 'ip_stream_image', self.qos_profile)
        
        # Open camera (IP for testing), checks if it's open
        self.URI = URI
        self.cap = cv2.VideoCapture(f'{self.URI}')
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture')
            rclpy.shutdown()
        
        # Setup timer to acquire image given the frame rate 
        fps = get_camera_fps(self.cap)
        self.get_logger().info(f'Frame rate: {fps}')
        self.timer = self.create_timer(1/fps, self.timer_callback)

        # Images are acquired using OpenCV. Instantiate bridge to pass images to ROS2
        self.bridge = CvBridge()

    def timer_callback(self):
        # Start timer
        start_time = time.time() 

        # Get frame, record timestamp
        ret,frame = self.cap.read()
        frm_time = time.time() # 

        if ret:
            self.frame_counter += 1

            # Get message, record timestamp
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")  
            msg_time = time.time()

            # Publish message, record timestamp
            self.publisher_.publish(msg)
            pub_time = time.time()

            # Measure elapsed times between capture, conversion, and publishing
            delta_frame = frm_time - start_time
            delta_msg = msg_time - frm_time
            delta_pub = pub_time - msg_time
            delta_tot = time.time() - start_time

            if self.frame_counter == 10:
                self.get_logger().info(f'\nFrame: {delta_frame*1000:.2f}ms\nmsg: {delta_msg*1000:.2f}ms\npublish: {delta_pub*1000:.2f}ms\ntot: {delta_tot*1000:.2f}ms\n')
                self.get_logger().debug(f'\nFrame: {delta_frame*1000:.2f}ms\nmsg: {delta_msg*1000:.2f}ms\npublish: {delta_pub*1000:.2f}ms\ntot: {delta_tot*1000:.2f}ms\n')
                self.frame_counter = 0 

        else:
            self.get_logger().error('Failed to capture image, trying to restart capture device')
            self.cap = cv2.VideoCapture(f'{self.URI}')
            if not self.cap.isOpened():
                self.get_logger().error('Capture device restart failed. Trying again')


        

def main(args=None):

    # Instantiate camera node
    rclpy.init(args=args)



    # node = CameraNode("http://192.168.1.172:8000/video_feed")
    node = CameraNode("http://192.168.123.39:8000/video_feed")
    try:
        rclpy.spin(node)  # Spin the node to keep it alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Allow the user to exit with Ctrl+C
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()