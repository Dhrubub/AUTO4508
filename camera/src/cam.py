#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Image


from depthai_sdk import OakCamera

from datetime import datetime
import cv2
import numpy as np
import depthai as dai

prevCount = 0
take_photo = False

rospy.init_node('camera_steer')

camera_publisher = rospy.Publisher('/camera_cmd_vel', Twist, queue_size=1)
cone_publisher = rospy.Publisher('/cone_detected', Bool, queue_size=1)

gui_colour_image_publisher = rospy.Publisher('/gui/colour_image', Image, queue_size=1)
gui_output_image_publisher = rospy.Publisher('/gui/output_image', Image, queue_size=1)
gui_capture_publisher = rospy.Publisher('/gui/capture', Image, queue_size=1)

pose = Twist()

lidar_opens = []

def publish_message(x):
    # Create an Int32 message and assign the value to publish
    message = Int32()
    message.data = x

    # cam_publisher.publish(message)  # Publish the message to the topic

def is_appropriate_size(area):
    return (100 < area < 10000)
    
def locate_cone(input_image):
    BGR_image = input_image

    # Assuming your cv2 frame is named 'frame'
    ros_image = Image()
    ros_image.header.stamp = rospy.Time.now()
    ros_image.header.frame_id = 'your_frame_id'
    ros_image.height = BGR_image.shape[0]
    ros_image.width = BGR_image.shape[1]
    ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
    ros_image.data = BGR_image.tobytes()  # Convert the frame to a byte string
    ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size
    
    gui_colour_image_publisher.publish(ros_image)

    full_image = BGR_image

    BGR_image = BGR_image[125:225, :]

    # Keep orange

    HSV_image = cv2.cvtColor(BGR_image, cv2.COLOR_BGR2HSV)
    
    # lower_threshold_orange = np.array([0, 120, 120])
    # upper_threshold_orange = np.array([60, 240, 240])

    # lower_threshold_orange2 = np.array([175, 120, 120])
    # upper_threshold_orange2 = np.array([180, 240, 240])

    lower_threshold_orange = np.array([0, 50, 50])
    upper_threshold_orange = np.array([10, 240, 255])

    lower_threshold_orange2 = np.array([170, 50, 50])
    upper_threshold_orange2 = np.array([180, 240, 255])
    
    orange_only = cv2.inRange(HSV_image, lower_threshold_orange, upper_threshold_orange)
    orange_only2 = cv2.inRange(HSV_image, lower_threshold_orange2, upper_threshold_orange2) 

    orange_only_mask = cv2.bitwise_or(orange_only, orange_only2)
    
    eroded_orange = cv2.erode((orange_only_mask), cv2.getStructuringElement(1, (11,11)))
    
    eroded_then_dilated_orange = cv2.dilate((eroded_orange), cv2.getStructuringElement(1, (11,11)))
    
    # cv2.imshow("Colour", BGR_image)
    # rospy.logerr(f"height: {len(BGR_image)}")
    # rospy.logerr(f"width: {len(BGR_image[0])}")
    # eroded_then_dilated_orange = eroded_then_dilated_orange[125:225, :]



    # cv2.imshow("eroded_then_dilated_orange", eroded_then_dilated_orange)
    totalLabels, label_ids, values, centroids = cv2.connectedComponentsWithStats(eroded_then_dilated_orange, 8, cv2.CV_32S)
    
    labels_of_interest = []
    
    for i in range(totalLabels):
        area = values[i, cv2.CC_STAT_AREA]
        if is_appropriate_size(area) == True:
            labels_of_interest.append(i)
            
    centroid_x = -1
            
    if len(labels_of_interest) > 0:
            
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

        # rospy.logerr(f"centroid_x: {centroid_x}")
        
        if (100 > centroid_x > 0):
            pose.angular.z = 1
            camera_publisher.publish(pose)
            # rospy.logerr("go left")
            
        elif (300 > centroid_x > 200):
            pose.angular.z = -1
            camera_publisher.publish(pose)
            # rospy.logerr("go right")
            
        else:
            pose.angular.z = 0
            camera_publisher.publish(pose)
            # rospy.logerr("go straight")
            
        
    output = np.zeros(eroded_then_dilated_orange.shape, dtype="uint8")
    
    for i in labels_of_interest:
        i = labels_of_interest[index_of_largest_cluster]
        componentMask = (label_ids == i).astype("uint8") * 255

        output = cv2.bitwise_or(output, componentMask)
    
    return output, len(labels_of_interest), centroid_x, full_image

def find_cone():
    global prevCount
    global take_photo
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

            output, num_orange, orange_x, BGR_image = locate_cone(inRgb.getCvFrame())

            if take_photo:
                take_photo = False
                # Get the current time
                current_time = datetime.now()

                # Format the current time as a string
                time_string = current_time.strftime("%Y-%m-%d %H:%M:%S")

                # Print the time string
                # cv2.imshow(time_string, BGR_image)
                # Assuming your cv2 frame is named 'frame'
                ros_image = Image()
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = 'your_frame_id'
                ros_image.height = BGR_image.shape[0]
                ros_image.width = BGR_image.shape[1]
                ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
                ros_image.data = BGR_image.tobytes()  # Convert the frame to a byte string
                ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size

                gui_capture_publisher.publish(ros_image)


            # cv2.imshow("Output", output)
            # print(output.shape)
            # Assuming your cv2 frame is named 'frame'
            # padded_output = np.pad(output, ((0, 200), (0, 0)), mode="constant", constant_values=0)
            padded_output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            # print(padded_output.shape)
            ros_image = Image()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = 'your_frame_id'
            ros_image.height = padded_output.shape[0]
            ros_image.width = padded_output.shape[1]
            ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
            ros_image.data = padded_output.tobytes()  # Convert the frame to a byte string
            ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size

            gui_output_image_publisher.publish(ros_image)
            
            #rospy.logerr(num_orange)
            msg = Bool()
            msg.data = num_orange > 0
            cone_publisher.publish(msg)
            if not num_orange == prevCount:
                # print(num_orange)
                prevCount = num_orange
            
            # rate.sleep()

            # publish_message(orange_x)

            if cv2.waitKey(1) == ord('q'):
                break
            elif cv2.waitKey(1) == ord('r'):
                print("Taking photo")
                take_photo = True

def take_photo_callback(data):
    global take_photo
    if data.data:
        print("Taking photo")
        take_photo = True

if __name__ == "__main__":
    rospy.Subscriber('/target_reached', Bool, take_photo_callback)
    rospy.Subscriber('/take_bucket_photo', Bool, take_photo_callback)

    find_cone()

    rospy.spin()