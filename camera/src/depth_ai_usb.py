#!/usr/bin/python3

import depthai
import cv2
import numpy as np

def is_appropriate_size(area):
    if (area > 1000 and area < 10000):
        return True
    else:
        return False

def locate_cone(image):
    RGB_image = image
    HSV_image = cv2.cvtColor(RGB_image, cv2.COLOR_BGR2HSV_FULL)
    
    lower_threshold_orange = np.array([0, 50, 50])
    upper_threshold_orange = np.array([30, 255, 255])
    
    orange_only = cv2.inRange(HSV_image, lower_threshold_orange, upper_threshold_orange) 
    
    eroded_orange = cv2.erode((orange_only), cv2.getStructuringElement(1, (11,11)))
    
    eroded_then_dilated_orange = cv2.dilate((eroded_orange), cv2.getStructuringElement(1, (11,11)))
    
    cv2.imshow("RGB image", RGB_image)
    
    # cv2.imshow("eroded then dilated orange", eroded_then_dilated_orange)
    
    totalLabels, label_ids, values, centroids = cv2.connectedComponentsWithStats(eroded_then_dilated_orange, 8, cv2.CV_32S)
    
    labels_of_interest = []
    
    for i in range(totalLabels):
        area = values[i, cv2.CC_STAT_AREA]
        if is_appropriate_size(area) == True:
            labels_of_interest.append(i)
        
    #print(labels_of_interest)
        
    output = np.zeros(eroded_then_dilated_orange.shape, dtype="uint8")
    
    for i in labels_of_interest:
        componentMask = (label_ids == i).astype("uint8") * 255
        output = cv2.bitwise_or(output, componentMask)
          
    # cv2.imshow("output", output)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    return output, len(labels_of_interest)

# Create pipeline
pipeline = depthai.Pipeline()

# Configure pipeline to include the video stream from the USB camera
pipeline.setOpenVINOVersion(depthai.OpenVINO.Version.VERSION_2021_4)

cam_rgb = pipeline.createColorCamera()
cam_rgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setInterleaved(True)
cam_rgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)

# Create output queue for video frames
q_video = pipeline.createXLinkOut()
q_video.setStreamName("video")

# Link video output from the camera node to the output queue
cam_rgb.video.link(q_video.input)

# Start the pipeline
with depthai.Device(pipeline) as device:
    # Define a window for displaying the video frames
    cv2.namedWindow("USB Camera", cv2.WINDOW_NORMAL)

    # Retrieve video frames
    q_video = device.getOutputQueue(name="video", maxSize=4, blocking=True)

    # Process video frames
    while True:
        # Try to retrieve the next video frame
        data_packet = q_video.tryGet()
        if data_packet is not None:
            # Retrieve the video frame
            frame = data_packet.getCvFrame()
            
            orange_only, num_orange = locate_cone(frame)

            # Display the frame
            cv2.imshow("USB Camera", orange_only)
            print(num_orange)

        # Check for 'q' key press to exit
        if cv2.waitKey(1) == ord('q'):
            break

    # Destroy the OpenCV window
    cv2.destroyAllWindows()
