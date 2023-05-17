#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32

from depthai_sdk import OakCamera

import cv2
import numpy as np

import depthai as dai
import numpy as np

rospy.init_node('camera_steer')

camera_publisher = rospy.Publisher('/camera_cmd_vel', Twist, queue_size=1)
cone_publisher = rospy.Publisher('/cone_detected', Bool, queue_size=1)

pose = Twist()

def publish_message(x):
    # Create an Int32 message and assign the value to publish
    message = Int32()
    message.data = x

    # cam_publisher.publish(message)  # Publish the message to the topic

def is_appropriate_size(area):
    if (1000 < area < 50000):
        return True
    else:
        return False
    
def locate_cone(input_image):
    BGR_image = input_image

    HSV_image = cv2.cvtColor(BGR_image, cv2.COLOR_BGR2HSV)
    
    lower_threshold_orange = np.array([0, 50, 0])
    upper_threshold_orange = np.array([1, 240, 255])

    lower_threshold_orange2 = np.array([179, 50, 0])
    upper_threshold_orange2 = np.array([180, 240, 255])
    
    orange_only = cv2.inRange(HSV_image, lower_threshold_orange, upper_threshold_orange)
    orange_only2 = cv2.inRange(HSV_image, lower_threshold_orange2, upper_threshold_orange2) 

    orange_only_mask = cv2.bitwise_or(orange_only, orange_only2)
    
    eroded_orange = cv2.erode((orange_only_mask), cv2.getStructuringElement(1, (11,11)))
    
    eroded_then_dilated_orange = cv2.dilate((eroded_orange), cv2.getStructuringElement(1, (11,11)))
    
    # cv2.imshow("Colour", BGR_image)
    # cv2.imshow("eroded_then_dilated_orange", eroded_then_dilated_orange)
    
    totalLabels, label_ids, values, centroids = cv2.connectedComponentsWithStats(eroded_then_dilated_orange, 8, cv2.CV_32S)
    
    labels_of_interest = []
    
    for i in range(totalLabels):
        area = values[i, cv2.CC_STAT_AREA]
        if is_appropriate_size(area) == True:
            labels_of_interest.append(i)
            
    centroid_x = -1
            
    if len(labels_of_interest) > 1:
            
        area_array = np.zeros((1,len(labels_of_interest)))
        
        for i in range(len(labels_of_interest)):
            # rospy.logerr("height: {}".format(len(area_array)))
            # rospy.logerr("width: {}".format(len(area_array[0])))
            area_array[0][i] = values[labels_of_interest[i], cv2.CC_STAT_AREA]
            
        index_of_largest_cluster = np.argmax(area_array)
        
        # rospy.logerr(index_of_largest_cluster)
        # rospy.logerr(len(labels_of_interest))
        
        cetroid_of_largest_orange = centroids[labels_of_interest[index_of_largest_cluster]]
        
        # rospy.logerr(cetroid_of_largest_orange)
        
        centroid_x = centroids[labels_of_interest[index_of_largest_cluster]][0]
        
        if (100 > centroid_x > 0):
            pose.angular.z = 0.3
            camera_publisher.publish(pose)
            # rospy.logerr("go left")
            
        if (255 > centroid_x > 155):
            pose.angular.z = -0.3
            camera_publisher.publish(pose)
            # rospy.logerr("go right")
            
        if (centroid_x > 100) and (centroid_x < 155):
            pose.angular.z = 0
            camera_publisher.publish(pose)
            # rospy.logerr("go straight")
            
        if (centroid_x == -1):
            pose.angular_z = 0
            camera_publisher.publish(pose)
            # rospy.logerr("No orange object detected")
        
        
    output = np.zeros(eroded_then_dilated_orange.shape, dtype="uint8")
    
    for i in labels_of_interest:
        componentMask = (label_ids == i).astype("uint8") * 255

        output = cv2.bitwise_or(output, componentMask)
    
    return output, len(labels_of_interest), centroid_x

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

        rate = rospy.Rate(50)
        while True:
            inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

            #Retrieve 'bgr' (opencv format) frame
            # cv2.imshow("rgb", inRgb.getCvFrame())

            output, num_orange, orange_x = locate_cone(inRgb.getCvFrame())
        
            # cv2.imshow("output", output)
            
            #rospy.logerr(num_orange)
            msg = Bool()
            msg.data = num_orange > 0
            cone_publisher.publish(msg)
            
            rate.sleep()

            # publish_message(orange_x)

            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == "__main__":
    find_cone()
    rospy.spin()