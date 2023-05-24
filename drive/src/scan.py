#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import NavSatFix

import math

rospy.init_node("scan_node")

cmd_publisher = rospy.Publisher('/scan_cmd_vel', Twist, queue_size=1)
photo_publisher = rospy.Publisher('/take_bucket_photo', Bool, queue_size=1)
scan_complete_publisher = rospy.Publisher('/scan_complete', Bool, queue_size=1)
gui_bucket_publisher = rospy.Publisher('/gui/bucket', Float32MultiArray, queue_size=1)

pose = Twist()

initial_heading = None

pose.angular.z = 0.2

taken_photo = False
target_reached = False

find_distance = None

def find_bucket(data):
    global pose
    global taken_photo
    global target_reached
    global find_distance

    dists = data.data
    # print(dists[len(dists)//2-2:len(dists)//2+2])
    if not taken_photo and target_reached:
        for d in dists[len(dists)//2-2:len(dists)//2+2]:
            if 2 < d < 10:
                print(d)
                find_distance = d
                msg = Bool()
                msg.data = True
                taken_photo = True
                photo_publisher.publish(msg)
                print("take photo while scanning")
                break

def target_reached(data):
    global target_reached
    if data.data:
        target_reached = data.data

current_heading = None

def heading_callback(data):
    global pose
    global taken_photo
    global initial_heading
    global target_reached
    global current_heading

    current_heading = data.data

    if (not initial_heading) and target_reached:
        initial_heading = data.data
    
    if not initial_heading:
        return

    if taken_photo:
        # print(initial_heading)
        # if not (initial_heading - 10 < data.data < initial_heading + 10):
        #     pose.angular.z = 0.5
        # else:
            initial_heading = None
            taken_photo = False
            target_reached = False
            msg = Bool()
            msg.data = True
            scan_complete_publisher.publish(msg)


def gps_callback(data):
    global current_heading
    global find_distance
    global taken_photo

    if taken_photo:
        dist = find_distance / 100000
        phi = current_heading / 180 * math.pi

        xdist = dist 
        ydist = 0

        x2 = cos(phi) * xdist - sin(phi) * ydist + data.longitude
        y2 = sin(phi) * xdist + cos(phi) * ydist + data.latitude

        pos = Float32MultiArray()
        pos.msg = [y2, x2]

        gui_bucket_publisher.publish(pos)

        rospy.logerr(find_distance)



if __name__ == '__main__':
    rate = rospy.Rate(500)
    rospy.Subscriber('/target_reached', Bool, target_reached)
    rospy.Subscriber('/lidar_front', Float32MultiArray,  find_bucket)
    rospy.Subscriber('/imu_heading', Int32, heading_callback)
    rospy.Subscriber('/fix', NavSatFix, gps_callback)


    while not rospy.is_shutdown():
        # print(pose)
        cmd_publisher.publish(pose)

        rate.sleep()




    
