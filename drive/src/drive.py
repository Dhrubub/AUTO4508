#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

def sendCmdVel(linear_velocity, angular_velocity):

    # Create a Twist message to store the linear and angular velocities
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = linear_velocity
    cmd_vel_msg.angular.z = angular_velocity

    # Publish the message

    # Log the published velocities
    rospy.loginfo("Published linear velocity: %s, angular velocity: %s", linear_velocity, angular_velocity)
    return cmd_vel_msg

    


if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher', anonymous=False)
    cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(2)

    limit = 0

    while not rospy.is_shutdown() and limit < 10:
        # Set desired linear and angular velocities
        linear_velocity = 0.5  # meters per second
        angular_velocity = 0  # radians per second

        # Call the send_cmd_vel function
        cmd_vel_pub.publish(sendCmdVel(linear_velocity, angular_velocity))
        rate.sleep()
        limit += 1



    # try:
    #     # Set desired linear and angular velocities
    #     linear_velocity = 0.2  # meters per second
    #     angular_velocity = 0  # radians per second

    #     # Call the send_cmd_vel function
    #     cmd_vel_pub.publish(sendCmdVel(linear_velocity, angular_velocity))

    #     rate.sleep()

    #     linear_velocity = 0
    #     angular_velocity = 0

    #     cmd_vel_pub.publish(sendCmdVel(linear_velocity, angular_velocity))

    # except rospy.ROSInterruptException:
    #     pass