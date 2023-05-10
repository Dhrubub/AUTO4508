#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SELECT = 8
R2 = 7
L2 = 6

LEFT = (3, 1)
RIGHT = (3, -1)

FORWARD = (1, 1)
REVERSE = (1, -1)

is_manual = True

rospy.init_node('joy_controller')

cmd_publisher = rospy.Publisher('/man_cmd_vel', Twist, queue_size=1)
manual_toggle_publisher = rospy.Publisher('/manual_toggle', Bool, queue_size=1)
deadman_publisher = rospy.Publisher('/deadman_state', Bool, queue_size=1)

pose = Twist()

def manual_input(buttons, axes):
    global pose
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

    if (select):
        msg = Bool()
        msg.data = True
        manual_toggle_publisher.publish(msg)
    
    manual_input(buttons, axes)

    l2 = buttons[L2] == 1
    r2 = buttons[R2] == 1
    move = l2 and r2

    msg = Bool()
    msg.data = move
    deadman_publisher.publish(msg)


if __name__ == '__main__':

    rospy.Subscriber('/joy', Joy, joy_callback)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        cmd_publisher.publish(pose)
        rate.sleep()



