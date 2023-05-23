#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Int32, Int32MultiArray
from geometry_msgs.msg import Twist

rospy.init_node("collision_avoidance")

cmd_vel_publisher = rospy.Publisher('/collision_cmd_vel', Twist, queue_size=1)

pose = Twist()

def open_cb(data):
    # front left right
    global pose
    front = data.data[0]
    right = data.data[1]

    pose = Twist()
    if not front:
        pose.angular.z = 1
    
    elif not right:
        pose.linear.x = 0.5

    

if __name__ == '__main__':
    rate = rospy.Rate(50)

    # rospy.Subscriber('/right_open', Bool,  right_open_cb)
    # rospy.Subscriber('/front_open', Bool,  front_open_cb)
    rospy.Subscriber('/lidar_directions_open', Int32MultiArray, open_cb)

    while not rospy.is_shutdown():
        cmd_vel_publisher.publish(pose)

        rate.sleep()