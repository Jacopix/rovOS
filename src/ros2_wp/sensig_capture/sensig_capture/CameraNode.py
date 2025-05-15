import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from lifecycle_msgs.msg import Transition
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
import cv2


import cv2
import time

from sensor_msgs.msg import CompressedImage
from utils.param_utils import load_node_parameters
from utils.camera_utils import get_camera_fps
from utils.timelogger import TimeLogger


class CameraNode(LifecycleNode):
    def __init__(self):
        super().__init__('camera_lifecycle_node')
        self.get_logger().info("Node instance created.")

        self.cap = None
        self.timer = None
        self.frame_counter = 0
        self.total_frames = 0
        self.time_logger = TimeLogger("frame_times")

    def on_configure(self, state: State):
        self.get_logger().info("🔧 Configuring node...")

        try:
            self.params = load_node_parameters(self, {
                "frame_logged": {
                    "default": 10,
                    "type": int,
                    "validate": lambda x: x > 0 or (_ for _ in ()).throw(ValueError("Must be > 0")) # _ for _ in() is a fancy way to throw an excpetion inside a lambda
                },
                "URI": {
                    "default": '',
                    "type": str,
                    "validate": lambda x: x or (_ for _ in ()).throw(ValueError("URI must not be empty"))
                },
                "quality": {
                    "default": 90,
                    "type": int,
                    "validate": lambda q: 0 < q <= 100 or (_ for _ in ()).throw(ValueError("Quality must be in (0,100]"))
                },
                "fps": {
                    "default": 0,
                    "type": int
                },
                "compression_format": {
                    "default": ".jpg",
                    "type": str
                }
            })

            self.publisher_ = self.create_publisher(CompressedImage, 'ip_stream_image', 10)

        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("🚀 Activating camera...")
        try:
            self._start_camera()
            fps = self.params['fps'] or get_camera_fps(self.cap)
            if fps <= 0:
                raise ValueError("FPS could not be determined.")
            self.get_logger().info(f'Camera FPS: {fps}')

            self.timer = self.create_timer(1 / fps, self._timer_callback)
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State):
        self.get_logger().error("❌ Node entered error state. Attempting recovery...")

        # Cleanup risorse
        if self.cap and self.cap.isOpened():
            self.cap.release()
    
        try:
            self._start_camera()
            self.get_logger().info("🔁 Recovery successful. Returning to inactive state.")
            return Transition.TRANSITION_CALLBACK_SUCCESS
        except Exception as e:
            self.get_logger().error(f"Recovery failed: {e}")
            return Transition.TRANSITION_CALLBACK_FAILURE


    def on_deactivate(self, state: State):
        self.get_logger().info("⏸️ Deactivating node...")
        if self.timer:
            self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("🧹 Cleaning up resources...")
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.timer = None
        self.cap = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info("📴 Shutting down node...")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS

    def _start_camera(self):
        uri = self.params['URI']
        self.cap = cv2.VideoCapture(uri)
        timer_0 = time.time()
        while not self.cap.isOpened() and time.time() - timer_0 < 10:
            self.get_logger().warn("Retrying camera open...")
            time.sleep(1)
            self.cap.open(uri)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera stream")

    def _timer_callback(self):
        self.time_logger.start()
        ret, frame = self.cap.read()
        self.time_logger.record("acquisition_time")

        if not ret:
            self.get_logger().error("Failed to grab frame. Triggering error state.")
            self.trigger_transition()

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.params['quality']]
        try:
            success, encoded_frame = cv2.imencode(self.params['compression_format'], frame, encode_param)
            if not success:
                self.get_logger().error("Encoding failed.")
                return
        except Exception as e:
            self.get_logger().error(f"Encoding exception: {e}")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = self.params['compression_format']
        msg.data = encoded_frame.tobytes()

        self.publisher_.publish(msg)
        self.time_logger.record("publish_time")

        self.frame_counter += 1
        self.total_frames += 1

        if self.frame_counter == self.params['frame_logged']:
            deltas = self.time_logger.log()
            self.get_logger().info(f'Frame {self.total_frames}')
            for name, delta in deltas.items():
                self.get_logger().info(f'{name:<20}: {delta*1000:.2f} ms')
            self.frame_counter = 0

    def trigger_error_transition(self):
        client = self.create_client(ChangeState, f'{self.get_name()}/change_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cannot reach change_state service.")
            return

        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ERROR

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Node transitioned to ERROR state.")
        else:
            self.get_logger().error("Failed to trigger error transition.")

        

def main(args=None):

    # Instantiate camera node
    rclpy.init(args=args)
    node = CameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node.cap and node.cap.isOpened():
            node.cap.release()
        if isinstance(node, CameraNode):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()