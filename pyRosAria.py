import rospy
from geometry_msgs.msg import Twist
import time

def sendCmdVel(linear_velocity, angular_velocity):
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    # Create a Twist message to store the linear and angular velocities
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = linear_velocity
    cmd_vel_msg.angular.z = angular_velocity

    # Publish the message
    cmd_vel_pub.publish(cmd_vel_msg)

    # Log the published velocities
    rospy.loginfo("Published linear velocity: %s, angular velocity: %s", linear_velocity, angular_velocity)

if __name__ == '__main__':
    try:
        # Set desired linear and angular velocities
        linear_velocity = 0.5  # meters per second
        angular_velocity = 0  # radians per second

        # Call the send_cmd_vel function
        sendCmdVel(linear_velocity, angular_velocity)

        time.sleep(5)

        linear_velocity = 0
        angular_velocity = 0

        sendCmdVel(linear_velocity, angular_velocity)

    except rospy.ROSInterruptException:
        pass
