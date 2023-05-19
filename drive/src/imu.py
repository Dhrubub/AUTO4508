#!/usr/bin/python3
import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
import math

from std_msgs.msg import Int32


import matplotlib.pyplot as plt

rospy.init_node("imu_node")

heading_publisher = rospy.Publisher("/imu_heading", Int32, queue_size=1)

def convert_to_bearing(yaw):
    bearing = 90 - math.degrees(yaw)
    if bearing < 0:
        bearing += 360

    return bearing


data_y = []
data_x = []



def imu_callback(data):
    global data_y
    global data_x

    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w

    # # Assuming you have the quaternion values
    quaternion = (x, y, z, w)

    # # Convert quaternion to Euler angles
    euler = euler_from_quaternion(quaternion)

    # # Extract roll, pitch, and yaw angles
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # yaw_degrees = int(convert_to_bearing(yaw))

    y = int(yaw* 180/math.pi)

    # if (y < 0): y+= 360

    # data_y.append(y)
    # data_x.append(len(data_x))

    # # Print the Euler angles
    # print("Roll: ", roll)
    # print("Pitch: ", pitch)
    # print("Yawn Degrees: ", y)

    msg = Int32()
    msg.data = y
    heading_publisher.publish(msg)



if __name__ == '__main__':
    rate = rospy.Rate(50)

    rospy.Subscriber('/imu/data', Imu, imu_callback)

    # limit = 1500

    while not rospy.is_shutdown():
        # print(limit)

        rate.sleep()
        # limit -= 1

    # minimum = min(len(data_x), len(data_y))
    # plt.plot(data_x[:minimum], data_y[:minimum])
    # plt.show()
