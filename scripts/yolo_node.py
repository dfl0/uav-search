#!/usr/bin/env python3

# yolo_node.py
# Publish a static image to the /camera/image_raw ROS topic
# Alex Cass 4/29/2026

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO


class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector_node')

        # Parameters
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/yolo/image_annotated")
        self.detection_topic = rospy.get_param("~detection_topic", "/yolo/detections")
        self.model_path = rospy.get_param("~model_path", "yolo26n.pt")
        self.conf_threshold = rospy.get_param("~conf_threshold", 0.65)
        self.cooldown = rospy.get_param("~cooldown", 2.0)

        # Load YOLOv26 model
        rospy.loginfo(f"Loading YOLO model from {self.model_path}...")
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            rospy.logerr(f"Model load failed: {e}")
            raise

        rospy.loginfo("Model loaded successfully.")

        self.bridge = CvBridge()

        # Establish Publishers
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.detection_pub = rospy.Publisher(self.detection_topic, String, queue_size=10)

        # Establish Subscriber
        self.sub = rospy.Subscriber(self.image_topic, Image, self.callback, queue_size=1)

        self.last_announcement = 0

    def callback(self, msg):
        # Convert ROS to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        # Run the YOLO v26 computer vision model
        try:
            results = self.model(frame, verbose=False)[0]
        except Exception as e:
            rospy.logerr(f"Inference failed: {e}")
            return

        # Assume nobody was detected, then process detection(s)
        person_detected = False

        if results.boxes is not None:
            for box in results.boxes:
                try:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls_id]

                    if conf < self.conf_threshold:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Draw bounding box for output visual
                    color = (0, 255, 0) if label == "person" else (255, 0, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame,
                                f"{label} {conf:.2f}",
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                color,
                                2)

                    if label == "person":
                        person_detected = True

                except Exception as e:
                    rospy.logwarn(f"Box parsing error: {e}")

        # Cooldown-based announcement
        now = time.time()

        # Print whatever YOLO sees to terminal
        if (now - self.last_announcement > self.cooldown):
            print("YOLO Detected: " + label)
            self.last_announcement = now
            
        # Publish when a person is detected
        if person_detected and (now - self.last_announcement > self.cooldown):
            rospy.loginfo("PERSON DETECTED")
            self.detection_pub.publish("PERSON DETECTED")
            self.last_announcement = now

        # Publish annotated image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            out_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(out_msg)
        except Exception as e:
            rospy.logerr(f"Publish failed: {e}")


if __name__ == "__main__":
    try:
        node = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass