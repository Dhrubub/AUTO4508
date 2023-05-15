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

def camera_example():
    with OakCamera() as oak:
        color = oak.create_camera('color')
        left = oak.create_camera('left')
        right = oak.create_camera('right')
        oak.visualize([color, left, right], fps=True)
        # oak.start(blocking=True)
        oak.start()

        # # while True:
        rospy.logerr(dir(color.stream))




def mobile():
    with OakCamera() as oak:
        color = oak.create_camera('color')
        nn = oak.create_nn('yolov4_coco_608x608', color)
        # nn = oak.create_nn('vehicle-detection-0202', color)
        oak.visualize([nn], fps=True)
        oak.start(blocking=True)

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

def test2():
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
            cv2.imshow("rgb", __)
            rospy.logerr(num_orange)

            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == "__main__":
    test2()
    rospy.spin()
    # rospy.logerr("Hello")