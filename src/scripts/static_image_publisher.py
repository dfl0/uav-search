#!/usr/bin/env python3
# static_image_publisher.py
# Publish a static image to the /camera/image_raw ROS topic
# Alex Cass 4/29/2026

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class StaticImagePublisher:
    def __init__(self):
        rospy.init_node('static_image_publisher', anonymous=True)

        # Parameters
        self.image_path = rospy.get_param("~image_path", "")
        self.topic_name = rospy.get_param("~topic", "/camera/image_raw")
        self.publish_rate = rospy.get_param("~rate", 5)  # Hz

        if not os.path.exists(self.image_path):
            rospy.logerr(f"Image not found: {self.image_path}")
            exit(1)

        # Load image once
        self.cv_image = cv2.imread(self.image_path)

        if self.cv_image is None:
            rospy.logerr("Failed to load image.")
            exit(1)

        self.bridge = CvBridge()

        self.pub = rospy.Publisher(self.topic_name, Image, queue_size=1)

        rospy.loginfo(f"Publishing {self.image_path} to {self.topic_name} at {self.publish_rate} Hz")

        self.run()

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8")
                msg.header.stamp = rospy.Time.now()
                self.pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"Publish failed: {e}")

            rate.sleep()


if __name__ == "__main__":
    try:
        StaticImagePublisher()
    except rospy.ROSInterruptException:
        pass