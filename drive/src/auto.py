#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

cmd_publisher = rospy.Publisher('/auto_cmd_vel', Twist, queue_size=1)

def deadman_callback(data):
    if data.data == True:
        pose = Twist()
        pose.linear.x = 0.5
        cmd_publisher.publish(pose)

    else:
        pose = Twist()
        cmd_publisher.publish(pose)

def boolean_subscriber():
    rospy.init_node("auto_node")

    rospy.Subscriber('deadman_state', Bool, deadman_callback)

    rospy.spin()

if __name__ == '__main__':
    boolean_subscriber()
    
