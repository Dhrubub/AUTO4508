#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rospy.init_node("auto_node")

cmd_publisher = rospy.Publisher('/auto_cmd_vel', Twist, queue_size=1)

pose = Twist()

pose.linear.x = 1

def gps_callback(data):
    global pose
    # rospy.logerr(data)
    pose.linear.x = data.linear.x * 0.5
    pose.angular.z = data.angular.z * 0.5



if __name__ == '__main__':
    rate = rospy.Rate(500)

    rospy.Subscriber('/gps_cmd_vel', Twist,  gps_callback)

    while not rospy.is_shutdown():
        cmd_publisher.publish(pose)

        rate.sleep()




    
