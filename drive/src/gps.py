#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, QuaternionStamped
from std_msgs.msg import Bool

import math

from sensor_msgs.msg import NavSatFix

rospy.init_node("gps_node")

cmd_publisher = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)


init_lat = None
init_long = None

target_lat = -31.980523
target_lon = 115.817174

# -31.980522833333332 long: 115.81717366666666

# -31.980485, 115.8172141

# -31.980600, 115.8172141


def calculate_distance(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Calculate the differences in latitude and longitude
    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    # Calculate the straight-line distance using the Euclidean formula
    distance = math.sqrt(delta_lat**2 + delta_lon**2)

    return distance

def is_distance_below_1m(lat1, lon1, lat2, lon2):
    distance = calculate_distance(lat1, lon1, lat2, lon2)
    if distance < 0.00001:  # 1 meter is approximately 0.00001 in latitude/longitude units
        return True
    else:
        return False


def get_angle_to_target(current_lat, current_lon, target_lat, target_lon):
    # Calculate the differences in latitude and longitude
    delta_lat = target_lat - current_lat
    delta_lon = target_lon - current_lon

    # if is_distance_below_1m(current_lat, current_lon, target_lat, target_lon):
        # return 0


    # Calculate the angle to the target using atan2
    angle_rad = math.atan2(delta_lon, delta_lat)
    angle_deg = math.degrees(angle_rad)

    return angle_deg


def gps_callback(data):
    # rospy.logerr(f"lat: {data.latitude} long: {data.longitude}")
    # rospy.logerr(dir(data))
    rot = get_angle_to_target(data.latitude, data.longitude, target_lat, target_lon)
    # rospy.logerr(f"Target angle: {rot}\n")
    pose = Twist()
    # rot = 1
    if (rot > 0):
        pose.angular.z = 1
    elif (rot < 0):
        pose.angular.z = -1

    cmd_publisher.publish(pose)


def heading_callback(data):
    rospy.logerr(data)

if __name__ == '__main__':
    rospy.Subscriber('/fix', NavSatFix, gps_callback)
    # rospy.Subscriber('/heading', QuaternionStamped, heading_callback)

    rospy.spin()    
