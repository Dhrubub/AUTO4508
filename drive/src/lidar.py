#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Int32, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

rospy.init_node("lidar_directions_open")

lidar_publisher = rospy.Publisher("/lidar_directions_open", Int32MultiArray, queue_size = 1)
lidar_front_publisher = rospy.Publisher("/lidar_front", Float32MultiArray, queue_size = 1)
lidar_right_publisher = rospy.Publisher("/lidar_right", Int32MultiArray, queue_size = 1)

def lidar_cb(data):
    
    point_cloud_arr = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    point_cloud_arr = np.array(list(point_cloud_arr))

    distance_array = []
    
    for i in range(len(point_cloud_arr)):

        x = point_cloud_arr[i][0]
        y = point_cloud_arr[i][1]
        
        distance = math.sqrt(x**2+y**2)
        
        distance_array.append(distance)
        
    front_check = [i for i in range(250, 550, 10)]
    right_check = [i for i in range(50, 250, 10)]

    front_dist = [distance_array[x] for x in front_check]
    right_dist = [distance_array[x] for x in right_check]

    
    # front_distances = distance_array[int(405-(detection_width/2)):int(405+(detection_width/2))]
    # front_distances = distance_array[250:550]

    # right_distances = distance_array[50:250]
    
    # non_zero_front_distances = [num for num in front_distances if num > 0.4]
    error_thresh = 0.2


    # front_distances = [d if d > error_thresh else 10 for d in front_distances]
    # right_distances = [d if d > error_thresh else 10 for d in right_distances]
    

    # non_zero_right_distances = [num for num in right_distances if num > 0.4]
    
    array_msg = Int32MultiArray()
    array_front_msg = Float32MultiArray()
    array_front_msg.data = front_dist


    # array_msg.layout.dim[0].label = "lidar_front_left_right"
    # array_msg.layout.dim[0].size = 3
    # array_msg.layout.dim[0].stride = 1
    

    # nparr = [distance if distance > 0.2 else 10000 for distance in front_dist]
    # print(front_check[nparr.index(min(nparr))])
    
    # rospy.logerr(np.where(np_distance_array == np_distance_array.min()))

    array_msg.data = [False, False]

    # window = 3
    # front_avg = 100
    # for i in range(len(front_distances)):
    #     if (i+window >= len(front_distances)): break
    #     avg = np.average(front_distances[i:i+window])
    #     front_avg = min(avg, front_avg)

    # right_avg = 100
    # for i in range(len(right_distances)):
    #     if (i+window >= len(right_distances)): break
    #     avg = np.average(right_distances[i:i+window])
    #     right_avg = min(avg, right_avg)

    
    array_msg.data[0] = not np.any([error_thresh < d < 1.5 for d in front_dist])

    array_msg.data[1] = not np.any([error_thresh < d < 1.5 for d in right_dist])
    # array_msg.data[1] = not error_thresh < right_avg < 1

    lidar_front_publisher.publish(array_front_msg)
    lidar_publisher.publish(array_msg)

if __name__ == '__main__':
    rate = rospy.Rate(10)

    rospy.Subscriber('/cloud', PointCloud2,  lidar_cb)

    while not rospy.is_shutdown():
        
        rate.sleep()