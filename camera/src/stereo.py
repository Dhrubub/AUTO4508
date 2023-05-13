#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from depthai_sdk import OakCamera

import cv2
import numpy as np

rospy.init_node('stereo_camera')

def is_appropriate_size(area):
    if (area > 50) and (area < 10000):
        return True
    else:
        return False
    
def locate_cone(input_image):
    RGB_image = input_image
    HSV_image = cv2.cvtColor(RGB_image, cv2.COLOR_BGR2HSV_FULL)
    
    lower_threshold_orange = np.array([0, 50, 50])
    upper_threshold_orange = np.array([30, 255, 255])
    
    orange_only = cv2.inRange(HSV_image, lower_threshold_orange, upper_threshold_orange) 
    
    eroded_orange = cv2.erode((orange_only), cv2.getStructuringElement(1, (11,11)))
    
    eroded_then_dilated_orange = cv2.dilate((eroded_orange), cv2.getStructuringElement(1, (11,11)))
    
    cv2.imshow("RGB image", RGB_image)
    
    totalLabels, label_ids, values, centroids = cv2.connectedComponentsWithStats(eroded_then_dilated_orange, 8, cv2.CV_32S)
    
    labels_of_interest = []
    
    for i in range(totalLabels):
        area = values[i, cv2.CC_STAT_AREA]
        if is_appropriate_size(area) == True:
            labels_of_interest.append(i)
        
    output = np.zeros(eroded_then_dilated_orange.shape, dtype="uint8")
    
    for i in labels_of_interest:
        componentMask = (label_ids == i).astype("uint8") * 255
        output = cv2.bitwise_or(output, componentMask)
    
    return output, len(labels_of_interest)

def camera_example():
    with OakCamera() as oak:
        color = oak.create_camera('color')
        left = oak.create_camera('left')
        right = oak.create_camera('right')
        oak.visualize([color, left, right], fps=True)
        oak.start(blocking=False)
        __, num_orange = locate_cone(color)
        print(num_orange)

if __name__ == "__main__":
    camera_example()
    rospy.spin()
    # rospy.logerr("Hello")
