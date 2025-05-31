import os
import cv2
import sys
import time
import select
from ultralytics import YOLO
from datetime import datetime

# Process frame to make it gray-scale and blurred
def gray_and_blur(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21,21), 0)
    return gray

# Checks if current and previous frame are different
def processed_movement_detection(prev_processed_frame,current_processed_frame):
    # Check if current and previous frame are different
    frame_diff = cv2.absdiff(prev_processed_frame, current_processed_frame)
    thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
    # Returns True if thresh is greater than the threshold
    return cv2.countNonZero(thresh) > 5000

# Annotate frame with bounding box, label, class and confidence
def annotate_frame(model, frame_to_annotate, box, color=(0, 225, 0), fontScale=1, thickness=2): # color and thickness are for both box and text
    # Extrapolate info from the model
    cls_id = int(box.cls[0])
    class_name = model.names[cls_id]
    conf = float(box.conf[0])
    x1, y1, x2, y2 = map(int, box.xyxy[0])  # coordinates
    # Draw box
    cv2.rectangle(frame_to_annotate, (x1, y1), (x2, y2), color, thickness)  # (x1,y1) is the top left corner, (x2,y2) the bottom right corner
    # Label with name, class, and confidence
    label = f"{class_name} {conf:.2f}"
    # Get the size of the text
    (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, fontScale, thickness)
    # Calculate the position for the text
    text_x = x1 + 10
    text_y = y1 - 10
    # Check if the text goes beyond the left side of the frame
    if text_x < 0:
        text_x = 0
    # Check if the text goes beyond the top of the frame
    if text_y - text_height < 0:
        text_y = y1 + text_height + 10
    # Check if the text goes beyond the right side of the frame
    if text_x + text_width > frame_to_annotate.shape[1]:
        text_x = frame_to_annotate.shape[1] - text_width - 10
    # Check if the text goes beyond the bottom of the frame
    if text_y + text_height > frame_to_annotate.shape[0]:
        text_y = frame_to_annotate.shape[0] - text_height - 10
    # Draw the text
    cv2.putText(frame_to_annotate, label, (text_x, text_y), cv2.FONT_HERSHEY_PLAIN, fontScale, color, thickness, lineType=cv2.LINE_AA)

def append_to_log(model, box, detection_information):
    cls_id = int(box.cls[0])
    class_name = model.names[cls_id]
    xyxy = box.xyxy[0].tolist()
    conf = float(box.conf[0])
    detection_information["detections"].append({
        "class": class_name,
        "confidence": round(conf, 3),
        "box": {
            "x1": round(xyxy[0], 6),
            "y1": round(xyxy[1], 6),
            "x2": round(xyxy[2], 6),
            "y2": round(xyxy[3], 6)
        }
    })
