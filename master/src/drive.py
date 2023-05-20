#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from enum import Enum

import time


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

        self.can_cone_follow = False
        self.collision_avoidance = False
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
    # rospy.logerr(data.data)
    if not data.data and state.current_state == CurrentState.CONE_FOLLOW:
        state.set_state(CurrentState.AUTO)
    
    if not state.current_state == CurrentState.CONE_FOLLOW and \
        not state.current_state == CurrentState.MANUAL and \
        state.can_cone_follow:

        if data.data:
        # rospy.logerr(data.data)

            # rospy.logerr(f"you have seen a new cone!")
        
            state.set_state(CurrentState.CONE_FOLLOW)

        # rospy.logerr(state.current_state)

            # rospy.logerr(state.current_state)

def camera_cmd_vel(data):
    if state.current_state == CurrentState.CONE_FOLLOW:
        pose = Twist()
        # rospy.logerr(f"camera updated steering")
        pose.angular.z = data.angular.z * 0.2
        pose.linear.x = 0.5
        state.set_pose(pose)
    # rospy.logerr(state.current_state)

def can_cone_follow(data):
    # rospy.logerr(data.data)
    state.can_cone_follow = data.data

def target_reached(data):
    if data.data:
        if state.current_state == CurrentState.AUTO or state.current_state == CurrentState.CONE_FOLLOW:
            state.set_state(CurrentState.SCAN)
            rospy.logerr("SCANNING")
            state.set_pose(Twist())
            time.sleep(5)
            rospy.logerr("SCANNING COMPLETE")
            # For Testing
            state.set_state(CurrentState.AUTO)

def all_targets_reached(data):
    if data.data:
        rospy.logerr("Completed all targets!")

def collision_avoidance(data):
    state.collision_avoidance = data.data

if __name__ == "__main__":
    rate = rospy.Rate(50)

    # Subscribers
    rospy.Subscriber('/auto_cmd_vel', Twist, auto_cmd_vel)
    rospy.Subscriber('/man_cmd_vel', Twist, man_cmd_vel)
    rospy.Subscriber('/manual_toggle', Bool, manual_toggle)
    rospy.Subscriber('/deadman_state', Bool, deadman_callback)
    rospy.Subscriber('/cone_detected', Bool, cone_detected)
    rospy.Subscriber('/camera_cmd_vel', Twist, camera_cmd_vel)
    rospy.Subscriber('/can_cone_follow', Bool, can_cone_follow)
    rospy.Subscriber('/target_reached', Bool, target_reached)
    rospy.Subscriber('/all_targets_reached', Bool, all_targets_reached)
    rospy.Subscriber('/collision_avoidance', Bool, collision_avoidance)


    # Publishers
    gui_current_state_publisher = rospy.Publisher('/gui/current_state', String, queue_size=1)


    prevState = CurrentState.MANUAL

    while not rospy.is_shutdown():
        if not state.current_state == CurrentState.MANUAL and state.deadman or state.current_state == CurrentState.MANUAL:
            if state.collision_avoidance and not state.current_state == CurrentState.MANUAL:
                pose = state.pose
                pose.linear.x = 0
                rosaria_cmd_vel_publisher.publish(pose)

            else:
                rosaria_cmd_vel_publisher.publish(state.pose)

        if not prevState == state.current_state:
            rospy.logerr(state.current_state)
            msg = String()
            msg.data = state.current_state.name
            gui_current_state_publisher.publish(msg)
            prevState = state.current_state

        rate.sleep()
