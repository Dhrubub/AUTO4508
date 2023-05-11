#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from depthai_sdk import OakCamera

rospy.init_node('stereo_camera')

def camera_example():
    with OakCamera() as oak:
        color = oak.create_camera('color')
        left = oak.create_camera('left')
        right = oak.create_camera('right')
        oak.visualize([color, left, right], fps=True)
        oak.start(blocking=True)



if __name__ == "__main__":
    camera_example()
    # rospy.spin()
    # rospy.logerr("Hello")
