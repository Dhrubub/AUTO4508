#!/usr/bin/python3
import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
import math

rospy.init_node("imu2_node")

def imu_callback(data):

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

    yaw_degrees = int(yaw*(180/math.pi))
    # # Print the Euler angles
    # print("Roll: ", roll)
    # print("Pitch: ", pitch)
    print("Yaw Degrees: ", yaw_degrees)



if __name__ == '__main__':
    rate = rospy.Rate(50)

    rospy.Subscriber('/imu/data', Imu, imu_callback)

    while not rospy.is_shutdown():

        # rate.sleep()
        pass

