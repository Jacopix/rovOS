from geometry_msgs.msg import Pose, Twist

from vortex_utils.python_utils import PoseData, TwistData, quat_to_euler


def pose_from_ros(pose_msg: Pose) -> PoseData:
    quat = pose_msg.orientation
    euler_angles = quat_to_euler(quat.x, quat.y, quat.z, quat.w)
    return PoseData(
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, *euler_angles
    )


def twist_from_ros(twist_msg: Twist) -> TwistData:
    return TwistData(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.linear.z,
        twist_msg.angular.x,
        twist_msg.angular.y,
        twist_msg.angular.z,
    )
