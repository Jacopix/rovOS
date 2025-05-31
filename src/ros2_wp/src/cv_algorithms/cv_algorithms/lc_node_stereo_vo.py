import rclpy
from cv_bridge import CvBridge
from dual_camera_msgs.msg import StereoVO
from dual_camera_msgs.msg import ImagePair
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from rclpy.qos import qos_profile_sensor_data

import numpy as np

from cv_algorithms.methods_stereo_vo import *

"""
Class to create a Python node to compute and publish
odometry data
"""
class StereoVONode(Node):
    """
    Define class costants
    """
    TN_ODOMETRY = "/output/cv_stereo_vo"
    TN_INPUT = "/stereo/image_pair"
    QOS_DEPTH_SUB = 10
    QOS_DEPTH_PUB = 10
    PARAM_NAME_DEBUG = "debug"
    PARAM_NAME_SAVE = "save"
    PARAM_NAME_TIME = "time"
    FRAME_NUMBER = 1
    P0 = np.array([1.4071030581737341e+03, 0.0, 9.6464705146844085e+02, 0.0, 0.0, 1.4103004488655147e+03, 5.4965615887916658e+02, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape(3, 4)
    P1 = np.array([1.4057201280175375e+03, 0.0, 9.7297533642744793e+02, 1.7286159877160915e+02, 0.0, 1.4096343950827527e+03, 5.5423157044647405e+02, 4.1078470033654471e+00, 0.0, 0.0, 1.0, 1.7567302829839208e-03]).reshape(3, 4)

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
        # Cannot set this params throght config.yaml
        self.declare_parameter(self.PARAM_NAME_DEBUG, True, param_descr_debug)
        self.declare_parameter(self.PARAM_NAME_SAVE, False, param_descr_save)
        self.declare_parameter(self.PARAM_NAME_TIME, False, param_descr_time)

    """
    Function to initialize node's variables
    """
    def init_var(self):
        self.comp_time = 0
        self.analysed = 0
        self.busy = False
        # Initialize QoS profiles
        qos_profile_pub = QoSProfile(depth = self.QOS_DEPTH_PUB)
        #qos_profile_sub = QoSProfile(history = HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        # Publisher
        self.publisher_ = self.create_lifecycle_publisher(StereoVO, self.TN_ODOMETRY, qos_profile_pub)
        # Subscriber
        self.subscription = self.create_subscription(ImagePair, self.TN_INPUT, self.callback_subscribe, qos_profile_sensor_data)
        self.subscription

    """
    Function to initialise odometry model
    """
    def init_odometry(self):
        self.bridge = CvBridge()
        self.left_img_prec = None

    """
    """
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.init_param()
        self.init_var()
        self.init_odometry()
        self.get_logger().info("Stereovision V.O. node configured!")
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
            message = StereoVO()
            message.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            message.translation = [data[0][3], data[1][3], data[2][3]]
            message.rotation = [data[0][0], data[0][1], data[0][2], data[1][0], data[1][1], data[1][2], data[2][0], data[2][1], data[2][2]]
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
            out = self.visual_odometry(lf, rf)
            self.analysed += 2
            if self.get_parameter(self.PARAM_NAME_TIME).get_parameter_value().bool_value:
                self.comp_time += time.time() - self.time
                self.get_logger().info(f"Analysed {self.analysed}\tMean {self.comp_time / self.analysed}[s]")
            self.busy = False
            if out:
                self.callback_publish(out)

    """
    Function that takes in input 2 frames and tries to calculate a trajectory from previous frames
    Output a 3x4 matrix describing the differences between the previous analysis, empty otherwise
    """
    def visual_odometry(self, left_img, right_img, stereo_matcher="bm", detector= "orb", matcher="BF", filter_threshold=None, mask= True, verbose=False, plot=False):
        if self.left_img_prec is None:
            self.left_img_prec = left_img
            if self.get_parameter(self.PARAM_NAME_DEBUG).get_parameter_value().bool_value:
                self.get_logger().info("First left frame acquired!")
            return []

        k_left,r_left,t_left = decomposeProjectionMatrix(self.P0)
        trajectory=np.zeros((self.FRAME_NUMBER, 3,4))
        T_tot=np.eye(4)
        trajectory[0]=(T_tot[:3,:])

        depth= stereo_to_depth(left_img, right_img, self.P0, self.P1)
        kp0,des0 = extract_features(self.left_img_prec)
        kp1,des1 = extract_features(left_img)
        self.left_img_prec = left_img
        unmatched_features= match_features(des0,des1)
        if filter_threshold is not None:
            matched_features=filter_matches(unmatched_features, filter_threshold)
        else:
            matched_features= unmatched_features #problem : list of 2-element list, ransac doesn't want it
        rvec,tvec= estimate_motion(kp0,kp1,k_left, matched_features, depth)
        #computing the transformation matrix [rvec| tvec]
        T_mat= np.hstack([rvec,tvec])
        #computing the homogeneous transformation matrix, so then I can invert it
        T_mat=np.vstack([T_mat, [0,0,0,1]])
        #inverting the transformation matrix
        T_mat=np.linalg.inv(T_mat)
        T_tot = T_tot.dot(T_mat)
        #appending T_tot into trajectory
        trajectory[i+1, :, :] = T_tot[:3][:]
        return trajectory

def main(args = None):
    rclpy.init(args = None)
    exectr = MultiThreadedExecutor()
    stereovo_node = StereoVONode("lc_stereovo_node")
    exectr.add_node(stereovo_node)
    try:
        exectr.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nInterrupt detected, killing node...")
    finally:
        stereovo_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
