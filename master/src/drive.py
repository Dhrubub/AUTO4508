#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from enum import Enum


rospy.init_node('master')

class CurrentState(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    CONE_FOLLOW = "CONE_FOLLOW"
    SCAN = "SCAN"

class State:
    def __init__(self):
        self.deadman = False
        self.pose = Twist()

        self.current_state = CurrentState.MANUAL


    def set_manual(self, data):
        if data:
            self.current_state = CurrentState.MANUAL
        else:
            self.current_state = CurrentState.AUTO

    def set_state(self, data):
        self.current_state = data

    def set_scan(self):
        self.pose = Twist()

    def set_pose(self, p):
        self.pose = p

# State
state = State()

# Publishers
rosaria_cmd_vel_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)

def man_cmd_vel(data):
    if state.current_state == CurrentState.MANUAL:
        state.set_pose(data)

def auto_cmd_vel(data):
    if state.current_state == CurrentState.AUTO:
        if state.deadman:
            state.set_pose(data)
        else:
            state.set_pose(Twist())

def manual_toggle(data):
    # Set to manual if not manual
    if data.data and not state.current_state == CurrentState.MANUAL:
        state.set_pose(Twist())
        state.set_state(CurrentState.MANUAL)

    # Set to auto if manual
    elif not data.data and state.current_state == CurrentState.MANUAL:
        state.set_pose(Twist())
        state.set_state(CurrentState.AUTO)

def deadman_callback(data):
    state.deadman = data.data

def cone_detected(data):
    if not state.current_state == CurrentState.CONE_FOLLOW and not state.current_state == CurrentState.MANUAL:
        if data.data == True:
        # rospy.logerr(data.data)
        
            state.set_state(CurrentState.CONE_FOLLOW)
            pose = Twist()
            pose.linear.x = 0.3
            state.set_pose(pose)
        # rospy.logerr(state.current_state)
        elif data.data == False:
            state.set_state(CurrentState.AUTO)
            # rospy.logerr(state.current_state)

def camera_cmd_vel(data):
    if state.current_state == CurrentState.CONE_FOLLOW:
        pose = Twist()
        pose.angular.z = data.angular.z
        pose.linear.x = 0.3
        state.set_pose(pose)
    # rospy.logerr(state.current_state)


if __name__ == "__main__":
    rate = rospy.Rate(50)

    # Subscribers
    rospy.Subscriber('/auto_cmd_vel', Twist, auto_cmd_vel)
    rospy.Subscriber('/man_cmd_vel', Twist, man_cmd_vel)
    rospy.Subscriber('/manual_toggle', Bool, manual_toggle)
    rospy.Subscriber('/deadman_state', Bool, deadman_callback)
    rospy.Subscriber('/cone_detected', Bool, cone_detected)
    rospy.Subscriber('/camera_cmd_vel', Twist, camera_cmd_vel)


    while not rospy.is_shutdown():
        if not state.current_state == CurrentState.MANUAL and state.deadman or state.current_state == CurrentState.MANUAL:
            rosaria_cmd_vel_publisher.publish(state.pose)
        rospy.logerr(state.current_state)

        rate.sleep()
