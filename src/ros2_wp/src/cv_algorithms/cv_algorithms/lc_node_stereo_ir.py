import rclpy
from cv_bridge import CvBridge
from dual_camera_msgs.msg import StereoIR
from dual_camera_msgs.msg import ImagePair
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from rclpy.qos import qos_profile_sensor_data

import torch

from cv_algorithms.methods_stereo_ir import *

"""
Class to create a Python node to compute and publish
image recognition algorithm on stereovision
"""
class StereoIRNode(Node):
    """
    Define class costants
    """
    TN_STEREO = "/output/cv_stereo_ir"
    TN_INPUT = "/stereo/image_pair"
    QOS_DEPTH_SUB = 10
    QOS_DEPTH_PUB = 10
    PARAM_NAME_DEBUG = "debug"
    PARAM_NAME_SAVE = "save"
    PARAM_NAME_TIME = "time"
    MODEL_PATH = "/home/ubuntu/ROS2_WSs/ws_sensing_rigs_v_three/src/cv_algorithms/cv_algorithms/models/classifier.pth"

    """
    Default class constructor
    """
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    """
    Function to initialize node's parameters
    """
    def init_param(self):
        # Initialize parameters
        param_descr_debug = ParameterDescriptor(description = "Prints additional debug informations.")
        param_descr_save = ParameterDescriptor(description = "Saves odometry messages on ROSbags.")
        param_descr_time = ParameterDescriptor(description = "Prints processing time of the frames.")
        self.declare_parameter(self.PARAM_NAME_DEBUG, False, param_descr_debug)
        self.declare_parameter(self.PARAM_NAME_SAVE, False, param_descr_save)
        self.declare_parameter(self.PARAM_NAME_TIME, False, param_descr_time)

    """
    Function to initialize node's variables
    """
    def init_var(self):
        self.analysed = 0
        self.comp_time = 0
        self.busy = False
        # Initialize QoS profiles
        qos_profile_pub = QoSProfile(depth = self.QOS_DEPTH_PUB)
        #qos_profile_sub = QoSProfile(history = HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        # Publisher
        self.publisher_ = self.create_lifecycle_publisher(StereoIR, self.TN_STEREO, qos_profile_pub)
        # Subscriber
        self.subscription = self.create_subscription(ImagePair, self.TN_INPUT, self.callback_subscribe, qos_profile_sensor_data)
        self.subscription

    """
    Function to initialise image recognition model
    """
    def init_image_recog(self):
        self.bridge = CvBridge()
        self.device = None
        self.model = None
        # 1. Controllo disponibilità GPU/CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
            self.get_logger().info(f"Device utilizzato: {self.device}")
        if self.device.type == "cpu":
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("ATTENZIONE: GPU non disponibile, le performance potrebbero essere ridotte")
        # 2. Controllo presenza modello
        model_ok, model_msg = check_model(self.MODEL_PATH, self.device)
        if not model_ok:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info(f"Modello non trovato: {model_msg}")
            quit()
        if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
            self.get_logger().info(f"Modello trovato: {model_msg}")
        # Carica il modello
        try:
            self.model = EnhancedClassifier(num_classes=2).to(self.device)
            self.model.load_state_dict(torch.load(self.MODEL_PATH, map_location=self.device))
            self.model.eval()
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Modello caricato correttamente")
        except Exception as e:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nel caricamento del modello: {str(e)}")
            quit()

    """
    """
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.init_param()
        self.init_var()
        self.init_image_recog()
        self.get_logger().info("Stereovision I.R. node configured!")
        self.status_param()
        return TransitionCallbackReturn.SUCCESS

    """
    """
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.busy = False
        return super().on_activate(state)

    """
    """
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.busy = True
        return super().on_deactivate(state)

    """
    """
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher_)
        self.destroy_subscriber(self.subscriber_)
        return TransitionCallbackReturn.SUCCESS

    """
    """
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    """
    Function that lists all parameters
    """
    def status_param(self):
        self.get_logger().info(f"debug:\t{self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value}")
        self.get_logger().info(f"save:\t{self.get_parameter(self.PARAM_NAME_SAVE).get_parameter_value().bool_value}")
        self.get_logger().info(f"time:\t{self.get_parameter(self.PARAM_NAME_TIME).get_parameter_value().bool_value}")

    """
    Callback function to publish result
    """
    def callback_publish(self, data):
        if (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Data received, but inactive node!")
        else:
            message = StereoIR()
            message.timestamp = datetime.today().strftime("%Y-%m-%d %H:%M:%S")
            message.label = data
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Published message!")
            if self.get_parameter(self.PARAM_NAME_SAVE).get_parameter_value().bool_value:
                self.get_logger().info("Message saved to rosbag!")
            self.publisher_.publish(message)

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
        if self.busy:
            pass
        elif (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Frames received, but inactive node!")
        else:
            self.busy = True
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Frames received!")
            lf = self.bridge.compressed_imgmsg_to_cv2(msg.left)
            rf = self.bridge.compressed_imgmsg_to_cv2(msg.right)
            if self.get_parameter(self.PARAM_NAME_TIME).get_parameter_value().bool_value:
                self.time = time.time()
            out = self.analyze_frames(lf, rf)
            self.analysed += 2
            if self.get_parameter(self.PARAM_NAME_TIME).get_parameter_value().bool_value:
                self.comp_time += time.time() - self.time
                self.get_logger().info(f"Analysed {self.analysed}\tMean {2 * (self.comp_time / self.analysed)}[s]")
            self.busy = False
            self.callback_publish(out)

    """
    Function that takes 2 frames of a stereo vision image
    and applies an image recognition algorithm given
    the selected model.
    Returns a string (what has been recognized) and a counter
    """
    def analyze_frames(self, left_frame, right_frame):
        label = "Niente"
        if left_frame is None or right_frame is None:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("Frame non disponibile, riprovo...")
        # Pre-elaborazione
        try:
            depth = compute_depth_map_advanced(left_frame, right_frame)
            enhanced_left = enhanced_retinex(left_frame, depth)
        except Exception as e:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nella pre-elaborazione: {str(e)}")
        # Prepara il tensore
        try:
            input_tensor = torch.from_numpy(enhanced_left).float().permute(2, 0, 1).unsqueeze(0).to(self.device) / 255.0
        except Exception as e:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nella preparazione del tensore: {str(e)}")
        # Inferenza
        try:
            with torch.no_grad():
                outputs = self.model(input_tensor)
                _, predicted = torch.max(outputs, 1)
                is_scallop = predicted.item() == 1
                if is_scallop:
                    label = "Capasanta"
        except Exception as e:
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nell'inferenza: {str(e)}")
        return label

def main(args = None):
    rclpy.init(args = args)
    exectr = MultiThreadedExecutor()
    stereoir_node = StereoIRNode("lc_stereoir_node")
    exectr.add_node(stereoir_node)
    try:
        exectr.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nInterrupt detected, killing node...")
    finally:
        stereoir_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
