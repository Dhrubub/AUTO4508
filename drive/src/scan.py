#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Int32MultiArray, Float32MultiArray

rospy.init_node("scan_node")

cmd_publisher = rospy.Publisher('/scan_cmd_vel', Twist, queue_size=1)
photo_publisher = rospy.Publisher('/take_bucket_photo', Bool, queue_size=1)
scan_complete_publisher = rospy.Publisher('/scan_complete', Bool, queue_size=1)

pose = Twist()

initial_heading = None

pose.angular.z = 0.2

taken_photo = False

def find_bucket(data):
    global pose
    global taken_photo

    dists = data.data
    # print(dists[len(dists)//2-2:len(dists)//2+2])
    if not taken_photo:
        for d in dists[len(dists)//2-2:len(dists)//2+2]:
            if 2 < d < 10:
                pose = Twist()
                msg = Bool()
                msg.data = True
                taken_photo = True
                photo_publisher.publish(msg)

target_reached = False
def target_reached(data):
    global target_reached
    target_reached = data.data
    
def heading_callback(data):
    global pose
    global taken_photo
    global initial_heading
    global target_reached

    if (not initial_heading) and target_reached:
        initial_heading = data.data
    
    if not initial_heading:
        return

    if taken_photo:
        if not (initial_heading - 10 < data.data < initial_heading + 10):
            pose.angular.z = 0.5
        else:
            initial_heading = None
            taken_photo = False
            target_reached = False
            msg = Bool()
            msg.data = True
            scan_complete_publisher.publish(msg)


if __name__ == '__main__':
    rate = rospy.Rate(500)
    rospy.Subscriber('/target_reached', Bool, target_reached)
    rospy.Subscriber('/lidar_front', Float32MultiArray,  find_bucket)
    rospy.Subscriber('/imu_heading', Int32, heading_callback)


    while not rospy.is_shutdown():
        cmd_publisher.publish(pose)

        rate.sleep()




    
