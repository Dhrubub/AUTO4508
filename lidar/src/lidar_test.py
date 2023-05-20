#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("lidar_front")

lidar_front_publisher = rospy.Publisher("/lidar_front", Int32, queue_size=1)


def lidar_cb(data):
    rospy.logerr(dir(data))
    # return

if __name__ == '__main__':
    rate = rospy.Rate(10)

    rospy.Subscriber('/cloud', Int32,  lidar_cb)

    while not rospy.is_shutdown():
        rate.sleep()