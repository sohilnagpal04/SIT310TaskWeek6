#!/usr/bin/env python3

# Python Libs
import sys
import time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def _init_(self):
        self.cv_bridge = CvBridge()

        # Subscribing to the image topic
        self.image_sub = rospy.Subscriber('/shravel/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        
        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert compressed image message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Define the cropping parameters (adjust according to your requirement)
        top = 200
        bottom = 400
        left = 100
        right = 500

        # Crop the image
        cropped_img = img[top:bottom, left:right]

        # Convert cropped image to HSV color space
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Define range for white color in HSV
        lower_white = np.array([0, 0, 200])  # Lower bound for white color
        upper_white = np.array([255, 50, 255])  # Upper bound for white color

        # Define range for yellow color in HSV
        lower_yellow = np.array([20, 100, 100])  # Lower bound for yellow color
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow color

        # Threshold the HSV image to get only white pixels
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)

        # Threshold the HSV image to get only yellow pixels
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        # Apply Canny Edge Detection to both masks
        edges_white = cv2.Canny(white_mask, 50, 150)
        edges_yellow = cv2.Canny(yellow_mask, 50, 150)

        # Apply Hough Transform to the edge-detected images
        white_lines = self.apply_hough_transform(edges_white)
        yellow_lines = self.apply_hough_transform(edges_yellow)

        # Draw lines found on both Hough Transforms on the cropped image
        self.draw_lines(cropped_img, white_lines)
        self.draw_lines(cropped_img, yellow_lines)

        # Display the image with detected lines
        cv2.imshow('Detected Lines', cropped_img)
        cv2.waitKey(1)

    def apply_hough_transform(self, img):
        # Apply Hough Transform
        lines = cv2.HoughLinesP(img, rho=1, theta=np.pi/180, threshold=100, minLineLength=50, maxLineGap=50)
        return lines

    def draw_lines(self, img, lines):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    def run(self):
        rospy.spin()

if _name_ == "_main_":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
        
    except rospy.ROSInterruptException:
        pass
