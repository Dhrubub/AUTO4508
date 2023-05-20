#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32

import math

from sensor_msgs.msg import NavSatFix

rospy.init_node("gps_node")

cmd_publisher = rospy.Publisher('/gps_cmd_vel', Twist, queue_size=1)

# targets = {
#     "rallying_point": {'lon': 115.8171661, 'lat': -31.98057735},
#     "somewhere_along_the_way": {'lon': 115.8171783, 'lat': -31.98038732},
#     "maybe_a_scoreboard": {'lon': 115.8171703, 'lat': -31.98017884},
#     "home_sweet_home" : {'lon': 115.8171315, 'lat': -31.98082892},
#     "let_s_have_a_little_stroll": {'lon': 115.8174657, 'lat': -31.98081842},
#     "supporter_united": {'lon': 115.8175647, 'lat': -31.98041252},
#     "coffee_time": {'lon': 115.8197862, 'lat': -31.98052211},
#     "drain": {'lon': 115.8172018, 'lat': -31.9804611},
# }

targets = {
    "one": {'lat': -31.980622, 'lon': 115.817494},
    "two": {'lat': -31.98049, 'lon': 115.81756,},
    "three": {'lat': -31.98055, 'lon': 115.81767},
    "four" : {'lat': -31.98042, 'lon': 115.81779},
    "drain": {'lat': -31.98051, 'lon': 115.81719},
}


# path_names = ["supporter_united", "maybe_a_scoreboard", "drain"]
path_names = ["one", "two", "three", "four", "drain"]
# pre_compute_headings = []
    


path = [targets[p] for p in path_names]

current_target = 0

init_pos = False

init_lat = None
init_long = None

heading_angle = None

# target_lat = -31.980523
# target_lon = 115.817174

# -31.980522833333332 long: 115.81717366666666

# -31.980485, 115.8172141

# -31.980600, 115.8172141

can_cone_follow_publisher = rospy.Publisher("/can_cone_follow", Bool, queue_size=1)
target_reached_publisher = rospy.Publisher("/target_reached", Bool, queue_size=1)
all_targets_reached_publisher = rospy.Publisher("/all_targets_reached", Bool, queue_size=1)


def calculate_distance(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude to radians
    lat1_rad = lat1
    lon1_rad = lon1
    lat2_rad = lat2
    lon2_rad = lon2

    # Calculate the differences in latitude and longitude
    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    # Calculate the straight-line distance using the Euclidean formula
    distance = math.sqrt(delta_lat**2 + delta_lon**2)

    return distance


def is_distance_below_5m(lat1, lon1, lat2, lon2):
    distance = calculate_distance(lat1, lon1, lat2, lon2)
    return distance < 0.00005  # 1 meter is approximately 0.00001 in latitude/longitude units


def is_distance_below_2m(lat1, lon1, lat2, lon2):
    distance = calculate_distance(lat1, lon1, lat2, lon2)
    rospy.logerr(distance*100000)
    return distance < 0.000015  # 1 meter is approximately 0.00001 in latitude/longitude units


def get_angle_to_target(current_lat, current_lon, target_lat, target_lon):
    # Calculate the differences in latitude and longitude
    global current_target

    delta_lat = target_lat - current_lat
    delta_lon = target_lon - current_lon


    msg = Bool()
    msg.data = is_distance_below_5m(current_lat, current_lon, target_lat, target_lon)
    can_cone_follow_publisher.publish(msg)

    msg = Bool()
    msg.data = False

    if (is_distance_below_2m(current_lat, current_lon, target_lat, target_lon)):
        msg.data = True
        current_target += 1
        if current_target >= len(path):
            all_targets_reached_publisher.publish(msg)
        
        rospy.logerr("REACHED!")

    target_reached_publisher.publish(msg)


    # Calculate the angle to the target using atan2
    angle_rad = math.atan2(delta_lat, delta_lon)
    angle_deg = math.degrees(angle_rad)

    return angle_deg


def gps_callback(data):
    global init_pos
    global path
    global heading_angle
    global current_target
    # rospy.logerr(f"lat: {data.latitude} long: {data.longitude}")
    # rospy.logerr(current_target)
    if not init_pos:
        if (data.latitude and data.longitude):
            pos = {"lon": data.longitude, "lat": data.latitude}
            path.append(pos)
            init_pos = True
            # rospy.logerr(len(path))

    if current_target >= len(path):
        return
    # rospy.logerr(dir(data))
    target_lat = path[current_target]['lat']
    target_long = path[current_target]['lon']

    # target_lat = -31.9804611
    # target_long = 115.8172018

    bearing = get_angle_to_target(data.latitude, data.longitude, target_lat, target_long)
    

    if heading_angle == None:
        return

    # rot = (bearing - 90) - heading_angle
    rot = bearing - heading_angle
    # rot = bearing - ((450 - heading_angle) % 360)

    # rot = bearing * -1 - (heading_angle - 90)

    if (rot >= 180): rot -= 360
    elif (rot < -180): rot += 360

    # rospy.logerr(f"Angle: {bearing}, heading_angle: {heading_angle}, rot: {rot}")
    # rospy.logerr(f"Target angle: {rot}\n")
    
    pose = Twist()
    pose.linear.x = 1

    if (rot >= 5):
        pose.angular.z = 0.5

        if rot < 20:
            pose.angular.z = 0.5
        elif rot > 90:
            pose.linear.x = 0.5


    elif (rot <= -5):
        pose.angular.z = -1

        if rot > -20:
            pose.angular.z = -0.5
        elif rot < -90:
            pose.linear.x = 0.5

    cmd_publisher.publish(pose)


def heading_callback(data):
    global heading_angle
    heading_angle = data.data

if __name__ == '__main__':
    rospy.Subscriber('/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/imu_heading', Int32, heading_callback)

    rospy.spin()    
