#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

rospy.init_node("collision_avoidance")

collision_publisher = rospy.Publisher("/collision_avoidance", Bool, queue_size=1)

def lidar_cb(data):
    
    point_cloud_arr = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    point_cloud_arr = np.array(list(point_cloud_arr))
    
    distance_array = []
    
    for i in range(len(point_cloud_arr)):

        x = point_cloud_arr[i][0]
        y = point_cloud_arr[i][1]
        
        distance = math.sqrt(x**2+y**2)
        
        distance_array.append(distance)
        
    detection_width = 300
    
    front_distances = distance_array[int(405-(detection_width/2)):int(405+(detection_width/2))]
    
    non_zero_front_distances = []
    
    non_zero_front_distances = [num for num in front_distances if num > 0.4]
    
    if len(non_zero_front_distances) > 0:
    
        min_value = min(non_zero_front_distances)
        
        if min_value > 2:
            # rospy.logerr(min_value)
            msg = Bool()
            msg.data = False
            collision_publisher.publish(msg)
        else:
            # rospy.logerr("object_detected")
            msg = Bool()
            msg.data = True
            collision_publisher.publish(msg)


if __name__ == '__main__':
    rate = rospy.Rate(500)

    rospy.Subscriber('/cloud', PointCloud2,  lidar_cb)

    while not rospy.is_shutdown():
        rate.sleep()