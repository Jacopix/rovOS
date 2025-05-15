import cv2
import time

def get_camera_fps(camera: cv2.VideoCapture, num_frames: int = 120):
    """
    Automatically measures the actual frames per second (FPS) of a video capture device.
    """
    if not camera.isOpened():
        raise ValueError("Camera is not opened")

    start = time.time()

    for i in range(num_frames):
        ret, frame = camera.read()
        if i % 10 == 0:
            print(f"Read {i} frames")
        if not ret:
            raise RuntimeError("Failed to read frame from camera")

    elapsed = time.time() - start
    fps = round(num_frames / elapsed, 2)
    return fps
