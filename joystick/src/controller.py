#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

SELECT = 8
R2 = 7
L2 = 6

LEFT = (3, 1)
RIGHT = (3, -1)

FORWARD = (1, 1)
REVERSE = (1, -1)

is_manual = True

rospy.init_node('joy_controller')
rospy.logerr('hello')

cmd_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

pose = Twist()

def manual_input(buttons, axes):
    l2 = buttons[L2] == 1
    r2 = buttons[R2] == 1

    forward = axes[FORWARD[0]] == FORWARD[1]
    reverse = axes[REVERSE[0]] == REVERSE[1]

    right = axes[RIGHT[0]] == RIGHT[1]
    left = axes[LEFT[0]] == LEFT[1]

    pose = Twist()

    
    speed = 1
    turbo = l2 and r2
    if (turbo): speed = 2

    if (right): pose.angular.z = -speed
    if (left): pose.angular.z = speed
    if (forward): pose.linear.x = speed
    if (reverse): pose.linear.x = -speed



def joy_callback(data):
    buttons = data.buttons
    axes = data.axes
    select = buttons[8]

    if (select): is_manual != is_manual

    if (is_manual):
        rospy.loginfo("Manual")
        manual_input(buttons, axes)
    else:
        rospy.loginfo("Automatic")
        l2 = buttons[L2] == 1
        r2 = buttons[R2] == 1
        move = l2 and r2

        if select or not move:
            pose = Twist()
            cmd_publisher.publish(pose)


if __name__ == '__main__':

    rospy.Subscriber('/joy', Joy, joy_callback)

    if is_manual:
        cmd_publisher.publish(pose)

    rospy.spin()


