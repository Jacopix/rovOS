import threading
import time

import numpy as np
import pytest
from geometry_msgs.msg import Pose, Twist
from gi.repository import Gst

from vortex_utils.gst_utils import H264Decoder
from vortex_utils.python_utils import (
    PoseData,
    State,
    TwistData,
    euler_to_quat,
    quat_to_euler,
    ssa,
)
from vortex_utils.ros_converter import pose_from_ros, twist_from_ros


def test_ssa_zero():
    assert ssa(0) == 0


def test_ssa_two_pi():
    assert ssa(2 * np.pi) == 0


def test_ssa_positive():
    assert ssa(3.5) == pytest.approx(-2.78, abs=0.01)


def test_ssa_negative():
    assert ssa(-3.5) == pytest.approx(2.78, abs=0.01)


def test_euler_to_quat_zero():
    quat = euler_to_quat(0, 0, 0)
    assert quat == pytest.approx([0, 0, 0, 1], abs=0.01)


def test_euler_to_quat_x():
    quat = euler_to_quat(1, 0, 0)
    assert quat == pytest.approx([0.479, 0, 0, 0.877], abs=0.01)


def test_euler_to_quat_y():
    quat = euler_to_quat(0, 1, 0)
    assert quat == pytest.approx([0, 0.479, 0, 0.877], abs=0.01)


def test_euler_to_quat_z():
    quat = euler_to_quat(0, 0, 1)
    assert quat == pytest.approx([0, 0, 0.479, 0.877], abs=0.01)


def test_euler_to_quat_xyz():
    quat = euler_to_quat(1, 1, 1)
    assert quat == pytest.approx([0.5709, 0.167, 0.5709, 0.565], abs=0.01)


def test_quat_to_euler_identity():
    euler = quat_to_euler(0, 0, 0, 1)
    assert euler == pytest.approx([0, 0, 0], abs=0.01)


def test_quat_to_euler_x():
    euler = quat_to_euler(0.707, 0, 0, 0.707)
    assert euler == pytest.approx([1.57, 0, 0], abs=0.01)


def test_quat_to_euler_z():
    euler = quat_to_euler(0, 0, 0.707, 0.707)
    assert euler == pytest.approx([0, 0, 1.57], abs=0.01)


def test_quat_to_euler_xyz():
    euler = quat_to_euler(0.4207, 0.4207, 0.229, 0.770)
    assert euler == pytest.approx([1, 1, 0], abs=0.01)


def test_quat_to_euler_xyz_negative():
    euler = quat_to_euler(0.4207, -0.4207, -0.229, 0.770)
    assert euler == pytest.approx([1, -1, 0], abs=0.01)


def test_pose_addition():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    assert (pose + pose2) == PoseData(1.1, 2.2, 3.3, 0.2, 0.4, 0.6)


def test_pose_subtraction():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    assert (pose - pose2) == PoseData(0.9, 1.8, 2.7, 0, 0, 0)


def test_pose_multiplication_scalar():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    assert (pose * 2) == PoseData(2, 4, 6, 0.2, 0.4, 0.6)


def test_pose_multiplication_pose():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    pose3 = pose * pose2
    assert pose3.x == pytest.approx(0.1, abs=0.01)
    assert pose3.y == pytest.approx(0.4, abs=0.01)
    assert pose3.z == pytest.approx(0.9, abs=0.01)
    assert pose3.roll == pytest.approx(0.01, abs=0.001)
    assert pose3.pitch == pytest.approx(0.04, abs=0.001)
    assert pose3.yaw == pytest.approx(0.09, abs=0.001)


def test_pose_rotation_matrix_shape():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    rotation_matrix = pose.as_rotation_matrix()
    assert rotation_matrix.shape == (3, 3)


def test_pose_rotation_matrix_values():
    pose = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    rotation_matrix = pose.as_rotation_matrix()
    assert rotation_matrix[0, 0] == pytest.approx(0.935, abs=0.05)
    assert rotation_matrix[0, 1] == pytest.approx(-0.283, abs=0.05)
    assert rotation_matrix[0, 2] == pytest.approx(0.2101, abs=0.05)
    assert rotation_matrix[1, 0] == pytest.approx(0.302, abs=0.05)
    assert rotation_matrix[1, 1] == pytest.approx(0.9505, abs=0.05)
    assert rotation_matrix[1, 2] == pytest.approx(-0.068, abs=0.05)
    assert rotation_matrix[2, 0] == pytest.approx(-0.1805, abs=0.05)
    assert rotation_matrix[2, 1] == pytest.approx(0.127, abs=0.05)
    assert rotation_matrix[2, 2] == pytest.approx(0.975, abs=0.05)


def test_pose_rotation_matrix_values_different_pose():
    pose = PoseData(1, 2, 3, 0.5, -0.5, 0.9)
    rotation_matrix = pose.as_rotation_matrix()
    assert rotation_matrix[0] == pytest.approx(
        [0.54551407, -0.68743404, -0.47942554], abs=0.001
    )
    assert rotation_matrix[1] == pytest.approx(
        [0.5445577, 0.72556086, -0.42073549], abs=0.001
    )
    assert rotation_matrix[2] == pytest.approx(
        [0.6370803, -0.03155774, 0.77015115], abs=0.001
    )


def test_twist_addition():
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    assert (twist1 + twist2) == TwistData(1.1, 2.2, 3.3, 0.2, 0.4, 0.6)


def test_twist_subtraction():
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    assert (twist1 - twist2) == TwistData(0.9, 1.8, 2.7, 0, 0, 0)


def test_twist_multiplication_scalar():
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    assert (twist1 * 2) == TwistData(2, 4, 6, 0.2, 0.4, 0.6)


def test_twist_multiplication_twist():
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    twist3 = twist1 * twist2
    assert twist3.linear_x == pytest.approx(0.1, abs=0.01)
    assert twist3.linear_y == pytest.approx(0.4, abs=0.01)
    assert twist3.linear_z == pytest.approx(0.9, abs=0.01)
    assert twist3.angular_x == pytest.approx(0.01, abs=0.001)
    assert twist3.angular_y == pytest.approx(0.04, abs=0.001)
    assert twist3.angular_z == pytest.approx(0.09, abs=0.001)


def test_state_addition_pose():
    pose1 = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    state1 = State(pose1, twist1)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    state2 = State(pose2, twist2)
    assert (state1 + state2).pose == PoseData(1.1, 2.2, 3.3, 0.2, 0.4, 0.6)


def test_state_subtraction_pose():
    pose1 = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    state1 = State(pose1, twist1)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    state2 = State(pose2, twist2)
    assert (state1 - state2).pose == PoseData(0.9, 1.8, 2.7, 0, 0, 0)


def test_state_addition_twist():
    pose1 = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    state1 = State(pose1, twist1)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    state2 = State(pose2, twist2)
    assert (state1 + state2).twist == TwistData(1.1, 2.2, 3.3, 0.2, 0.4, 0.6)


def test_state_subtraction_twist():
    pose1 = PoseData(1, 2, 3, 0.1, 0.2, 0.3)
    twist1 = TwistData(1, 2, 3, 0.1, 0.2, 0.3)
    state1 = State(pose1, twist1)
    pose2 = PoseData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    twist2 = TwistData(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)
    state2 = State(pose2, twist2)
    assert (state1 - state2).twist == TwistData(0.9, 1.8, 2.7, 0, 0, 0)


@pytest.fixture
def decoder():
    """Fixture to create and clean up an H264Decoder instance."""
    decoder = H264Decoder()
    decoding_thread = threading.Thread(target=decoder.start, daemon=True)
    decoding_thread.start()
    yield decoder
    decoder.stop()
    decoding_thread.join()


def test_h264_decoder_initialization(decoder):
    """Test that the H264Decoder initializes correctly."""
    assert decoder.appsrc is not None, "Appsrc element is missing."
    assert decoder._appsink is not None, "Appsink element is missing."
    assert decoder._pipeline is not None, "Pipeline was not initialized."
    assert decoder._bus is not None, "GStreamer bus was not initialized."


def test_h264_decoder_decodes_frames(decoder):
    """Test if the decoder correctly processes an H.264 stream."""
    test_file = "tests/resources/test_video.h264"

    # Read and push H.264 data
    with open(test_file, "rb") as f:
        raw_data = f.read()

    chunk_size = 64
    for i in range(0, len(raw_data), chunk_size):
        decoder.push_data(raw_data[i : i + chunk_size])

    decoder.appsrc.emit("end-of-stream")

    # Wait for frames to be decoded
    timeout = 5.0
    start_time = time.time()
    while len(decoder.decoded_frames) == 0 and (time.time() - start_time) < timeout:
        time.sleep(0.1)

    assert len(decoder.decoded_frames) > 0, "No frames were decoded."


def test_h264_decoder_frame_properties(decoder):
    """Test if decoded frames have correct properties."""
    test_file = "tests/resources/test_video.h264"

    # Read and push H.264 data
    with open(test_file, "rb") as f:
        raw_data = f.read()

    chunk_size = 64
    for i in range(0, len(raw_data), chunk_size):
        decoder.push_data(raw_data[i : i + chunk_size])

    decoder.appsrc.emit("end-of-stream")

    # Wait for frames to be decoded
    timeout = 5.0
    start_time = time.time()
    while len(decoder.decoded_frames) == 0 and (time.time() - start_time) < timeout:
        time.sleep(0.1)

    assert len(decoder.decoded_frames) > 0, "No frames were decoded."

    frame = decoder.decoded_frames[0]
    assert isinstance(frame, np.ndarray), "Decoded frame is not a numpy array."
    assert frame.ndim == 3, f"Expected 3D array (H, W, C), got shape {frame.shape}."


def test_h264_decoder_stops_cleanly(decoder):
    """Test if the decoder stops without errors."""
    decoder.stop()
    assert decoder._pipeline.get_state(0)[1] == Gst.State.NULL, (
        "Decoder did not shut down properly."
    )


def test_pose_from_ros():
    pose_msg = Pose()
    pose_msg.position.x = 1.0
    pose_msg.position.y = 2.0
    pose_msg.position.z = 3.0
    pose_msg.orientation.x = 0.1
    pose_msg.orientation.y = 0.2
    pose_msg.orientation.z = 0.3
    pose_msg.orientation.w = 0.4
    euler = quat_to_euler(0.1, 0.2, 0.3, 0.4)
    pose = pose_from_ros(pose_msg)
    assert pose.x == 1.0
    assert pose.y == 2.0
    assert pose.z == 3.0
    assert pose.roll == pytest.approx(euler[0], abs=0.01)
    assert pose.pitch == pytest.approx(euler[1], abs=0.01)
    assert pose.yaw == pytest.approx(euler[2], abs=0.01)


def test_twist_from_ros():
    twist_msg = Twist()
    twist_msg.linear.x = 1.0
    twist_msg.linear.y = 2.0
    twist_msg.linear.z = 3.0
    twist_msg.angular.x = 0.1
    twist_msg.angular.y = 0.2
    twist_msg.angular.z = 0.3
    twist = twist_from_ros(twist_msg)
    assert twist.linear_x == 1.0
    assert twist.linear_y == 2.0
    assert twist.linear_z == 3.0
    assert twist.angular_x == 0.1
    assert twist.angular_y == 0.2
    assert twist.angular_z == 0.3
