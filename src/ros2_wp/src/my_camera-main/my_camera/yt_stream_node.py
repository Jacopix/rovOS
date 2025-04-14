import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pafy


# ros2 run rqt_image_view rqt_image_view
# ros2 run my_camera yt_stream_node 


class YouTubeStreamerNode(Node):
    def __init__(self):
        super().__init__('yt_stream_node')
        
        # YouTube video link
        #url = "https://www.youtube.com/watch?v=qi0mY6zVQnY"
        url = "https://www.youtube.com/watch?v=FrULPuxyhWE"
        
        video = pafy.new(url)
        best = video.getbest(preftype="mp4")

        # Capture YouTube stream
        self.cap = cv2.VideoCapture(best.url)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open YouTube stream')
            rclpy.shutdown()
            return

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.delay = int(1000 / self.fps) if self.fps > 0 else 25

        # ROS Image publisher
        self.publisher_ = self.create_publisher(Image, 'yt_stream_image', 1)

        # Timer per leggere i frame
        self.timer = self.create_timer(0.001, self.timer_callback)

        # Per conversione OpenCV <-> ROS Image
        self.bridge = CvBridge()

        # Target width per ridimensionamento
        self.target_width = 640

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from stream')
            return

        # Ridimensiona mantenendo proporzioni
        height, width = frame.shape[:2]
        aspect_ratio = height / width
        new_height = int(self.target_width * aspect_ratio)
        resized_frame = cv2.resize(frame, (self.target_width, new_height))

        # Mostra finestra
        cv2.imshow("YouTube Stream", resized_frame)

        # Pubblica su ROS
        msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
        self.publisher_.publish(msg)

        # Esci se 'q' premuto
        if cv2.waitKey(self.delay) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YouTubeStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
