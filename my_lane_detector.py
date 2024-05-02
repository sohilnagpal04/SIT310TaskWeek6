#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Subscribing to the image topic
        self.image_sub = rospy.Subscriber('/shravel/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        
        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert compressed image message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Filter white pixels
        white_filtered = self.white_filter(img)

        # Filter yellow pixels
        yellow_filtered = self.yellow_filter(img)

        # Combine white and yellow filtered images
        combined_filtered = cv2.bitwise_or(white_filtered, yellow_filtered)

        # Convert image to grayscale
        gray = cv2.cvtColor(combined_filtered, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Apply Hough Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=50)

        # Output lines on the original image
        output_img = self.output_lines(img, lines)

        # Show image with detected lines
        cv2.imshow('Detected Lines', output_img)
        cv2.waitKey(1)

    def white_filter(self, img):
        # Define lower and upper bounds for white color in BGR
        lower_white = np.array([200, 200, 200], dtype=np.uint8)
        upper_white = np.array([255, 255, 255], dtype=np.uint8)

        # Create mask for white pixels
        white_mask = cv2.inRange(img, lower_white, upper_white)

        # Apply mask to input image
        white_filtered = cv2.bitwise_and(img, img, mask=white_mask)

        return white_filtered

    def yellow_filter(self, img):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Apply mask to input image
        yellow_filtered = cv2.bitwise_and(img, img, mask=yellow_mask)

        return yellow_filtered

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 2, cv2.LINE_AA)  # Draw lines in red
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0), -1)  # Draw green circles at endpoints
                cv2.circle(output, (l[2], l[3]), 2, (0, 255, 0), -1)  # Draw green circles at endpoints
        return output

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
        
    except rospy.ROSInterruptException:
        pass
