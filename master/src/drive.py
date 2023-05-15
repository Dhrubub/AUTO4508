#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


rospy.init_node('master')

class State:
    def __init__(self):
        self.is_manual = True
        self.deadman = False
        self.pose = Twist()

        # additional not used currently
        self.is_scanning = False

    def manual_toggle(self):
        self.is_manual = not self.is_manual

    def set_scan(self):
        self.pose = Twist()
        self.is_scanning = True

    def set_pose(self, p):
        self.pose = p

# State
state = State()

# Publishers
rosaria_cmd_vel_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

def man_cmd_vel(data):
    if state.is_manual:
        state.set_pose(data)

def auto_cmd_vel(data):
    if not state.is_manual:
        if state.deadman:
            state.set_pose(data)
        else:
            state.set_pose(Twist())

def manual_toggle(data):
    if data.data:
        state.set_pose(Twist())
        state.manual_toggle()

def deadman_callback(data):
    state.deadman = data.data

if __name__ == "__main__":
    rate = rospy.Rate(50)

    # Subscribers
    rospy.Subscriber('/auto_cmd_vel', Twist, auto_cmd_vel)
    rospy.Subscriber('/man_cmd_vel', Twist, man_cmd_vel)
    rospy.Subscriber('/manual_toggle', Bool, manual_toggle)
    rospy.Subscriber('/deadman_state', Bool, deadman_callback)

    while not rospy.is_shutdown():
        rosaria_cmd_vel_publisher.publish(state.pose)
        # rospy.logerr(state.pose.linear.x)
        rate.sleep()
