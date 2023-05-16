#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from depthai_sdk import OakCamera

import cv2
import numpy as np

import depthai as dai
import numpy as np

rospy.init_node('camera_node')

def is_appropriate_size(area):
    if (1000 < area < 50000):
        return True
    else:
        return False
    
def locate_cone(input_image):
    BGR_image = input_image

    HSV_image = cv2.cvtColor(BGR_image, cv2.COLOR_BGR2HSV)

    temp_bgr = np.array([[[10, 10, 180]]], dtype=np.uint8)
    temp_hsv = cv2.cvtColor(temp_bgr, cv2.COLOR_BGR2HSV)

    # rospy.logerr("HSV")
    # rospy.logerr(HSV_image[0])
    # rospy.logerr("BGR")
    # rospy.logerr(BGR_image[0])
    
    lower_threshold_orange = np.array([0, 50, 0])
    upper_threshold_orange = np.array([10, 240, 255])

    lower_threshold_orange2 = np.array([170, 50, 0])
    upper_threshold_orange2 = np.array([180, 240, 255])
    
    orange_only = cv2.inRange(HSV_image, lower_threshold_orange, upper_threshold_orange)
    orange_only2 = cv2.inRange(HSV_image, lower_threshold_orange2, upper_threshold_orange2) 

    orange_only_mask = cv2.bitwise_or(orange_only, orange_only2)
    
    eroded_orange = cv2.erode((orange_only_mask), cv2.getStructuringElement(1, (11,11)))
    
    eroded_then_dilated_orange = cv2.dilate((eroded_orange), cv2.getStructuringElement(1, (11,11)))
    
    cv2.imshow("bgr", BGR_image)
    # cv2.imshow("eroded_then_dilated_orange", eroded_then_dilated_orange)
    
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

def find_cone():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")

    # Properties
    camRgb.setPreviewSize(300, 300)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # Linking
    camRgb.preview.link(xoutRgb.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        print('Connected cameras:', device.getConnectedCameraFeatures())
        # Print out usb speed
        print('Usb speed:', device.getUsbSpeed().name)
        # Bootloader version
        if device.getBootloaderVersion() is not None:
            print('Bootloader version:', device.getBootloaderVersion())
        # Device name
        print('Device name:', device.getDeviceName())

        # Output queue will be used to get the rgb frames from the output defined above
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

            # Retrieve 'bgr' (opencv format) frame
            #cv2.imshow("rgb", inRgb.getCvFrame())


            __, num_orange = locate_cone(inRgb.getCvFrame())
            cv2.imshow("output", __)
            rospy.logerr(num_orange)

            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == "__main__":
    find_cone()
    rospy.spin()
    # rospy.logerr("Hello")