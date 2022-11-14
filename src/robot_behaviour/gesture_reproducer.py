#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# System imports
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class Gesture:

    def __init__(self):
        rospy.init_node('robot_behaviour', anonymous=True)
        self.client = SimpleActionClient('/play_motion', PlayMotionAction)

    def wait_for_valid_time(self, timeout):
        """Wait for a valid time (non-zero), this is important
    when using a simulated clock"""
        # Loop until:
        # * ros master shutdowns
        # * control+C is pressed (handled in is_shutdown())
        # * timeout is achieved
        # * time is valid
        start_time = time.time()
        while not rospy.is_shutdown():
            if not rospy.Time.now().is_zero():
                return
            if time.time() - start_time > timeout:
                rospy.logerr("Timed-out waiting for valid time.")
                exit(0)
            time.sleep(0.1)
        # If control+C is pressed the loop breaks, we can exit
        exit(0)

    def reproduce_motion(self, action_name):
        '''
    Reproduce action passed as input action_name
    '''
        rospy.loginfo("Starting run_motion_python application...")
        self.wait_for_valid_time(10.0)
        rospy.loginfo("Waiting for Action Server...")
        self.client.wait_for_server()

        goal = PlayMotionGoal()
        goal.motion_name = action_name
        goal.skip_planning = False
        goal.priority = 0  # Optional

        rospy.loginfo(f'Sending goal with motion: {action_name}')
        self.client.send_goal(goal)
        rospy.loginfo("Execute action without waiting for result...")


def main():
    robot_gesture = Gesture()
    robot_gesture.reproduce_motion("nod_no")


if __name__ == '__main__':
    main()
