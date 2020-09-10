#! /usr/bin/python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sammy Pfeiffer

# Tiago predefined motions are in /opt/pal/erbium/share/tiago_bringup/config

# System imports
import sys
import time
# ROS imports
import rospy
from actionlib import SimpleActionClient, GoalStatus

import sys
sys.path.append("/home/aandriella/pal/cognitive_game_ws/devel/lib/python2.7/dist-packages")

print(sys.path)
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import std_msgs.msg
# important for the srv
from std_srvs.srv import *
#import for getting joints
from sensor_msgs.msg import JointState


#min and max values for joints
min_joint_1 = -1.41
max_joint_1 = 1.31
min_joint_2 = -0.284
max_joint_2 = 4.509
min_joint_3 = -0.862
max_joint_3 = 1.845
min_joint_4 = -1.1
max_joint_4 = 2.31



class Gesture:

  def __init__(self):
    rospy.init_node('big_hero', anonymous=True)
    self.client = SimpleActionClient('/play_motion', PlayMotionAction)
    self.simulated_magnet = False#rospy.get_param('~magnet_simulation')
    self.simulated_actions = False#rospy.get_param('~actions_simulation')

  #callback for getting effort on each single joint
    rospy.Subscriber("/joint_states", JointState, self.get_joints_callback)

  def get_joints_callback(self, data):
    valueHasChanged = False
    # rospy.loginfo(rospy.get_caller_id())
    # print("effort values", data.effort[0:7])
    if data.effort[0] >= min_joint_1 and data.effort[0] <= max_joint_1:
      pass
    else:
      return True
      print("***************************************")
      print("joint 1 over ", data.effort[0])
      print("***************************************")
    if data.effort[1] >= min_joint_2 and data.effort[1] <= max_joint_2:
      pass
    else:
      return True
      print("***************************************")
      print("joint 2 over ", data.effort[1])
      print("***************************************")
    if data.effort[2] >= min_joint_3 and data.effort[2] <= max_joint_3:
      pass
    else:
      return True
      print("***************************************")
      print("joint 3 over ", data.effort[2])
      print("***************************************")
    if data.effort[3] >= min_joint_4 and data.effort[3] <= max_joint_4:
      pass
    else:
      return True
      print("***************************************")
      print("joint 4 over ", data.effort[3])
      print("***************************************")

  def activate_magnet(self):
    '''
    Function to activate the magnet
    '''
    if self.simulated_magnet == True:
      return True

    rospy.loginfo("Activating magnet...")
    rospy.wait_for_service('/set_magnet')
    try:
      activate = rospy.ServiceProxy('/set_magnet', SetBool)
      return activate(True)
    #except rospy.ServiceException as e:
    except:
      for i in range(10):
        rospy.logerr("Service call failed: ")
        rospy.loginfo("Try to reconnect")
        rospy.wait_for_service('/set_magnet')
        try:
          activate = rospy.ServiceProxy('/set_magnet', SetBool)
          return activate(True)
        except:
          rospy.loginfo("try again iteration ", i)
      return False

  def deactivate_magnet(self):
    '''
    Function to deactivate the magnet
    '''
    if self.simulated_magnet == True:
      return True

    rospy.loginfo("Deactivating magnet...")
    rospy.wait_for_service('/set_magnet')
    try:
      activate = rospy.ServiceProxy('/set_magnet', SetBool)
      return activate(False)
    #except rospy.ServiceException as e:
    except:
      for i in range(10):
        rospy.logerr("Service call failed: ")
        rospy.loginfo("Try to reconnect")
        rospy.wait_for_service('/set_magnet')
        try:
          activate = rospy.ServiceProxy('/set_magnet', SetBool)
          return activate(True)
        except:
          rospy.loginfo("try again iteration ", i)
      return False

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

  def get_status_string(self, status_code):
    return GoalStatus.to_string(status_code)

  def convert(self, cell):
    '''
    convert the location of the board to the predefined movements of the tiago
    g= grasp
    o = offer
    c =
    s = solution
    h = suggest_subset
    '''
    return 46 - cell - 5 * int((cell - 1) / 5)

  def wave(self):
    '''
    Wave the user
    '''
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "wave"
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: wave")
    self.client.send_goal(goal)
    rospy.loginfo("Execute action without waiting for result...")

  def initial_pos(self):
    '''
    Sending the robot to its initial location
    :
    '''
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "rest"
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: init")
    self.client.send_goal(goal)
    rospy.loginfo("Execute action without waiting for result...")


  def head_noddling_yes(self):
    '''
    Robot nods its head up and down
    '''
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "head_yes"
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: noddling_yes")
    self.client.send_goal(goal)

    rospy.loginfo("Execute actions without waiting for result...")

  def head_noddling_no(self):
    '''
    Robot nods its head left nd right
    '''
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "head_no"
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: noddling_no")
    self.client.send_goal(goal)

    rospy.loginfo("Execute action without waiting for result...")

  def pick(self, cell):
    '''
    Robot picks the token in location cell
    Args:
      cell: location of the board
    Return:
      true if the pick has been successfully executed
    '''
    # cell 1 is g45 for the robot view point
    conv_cell = int(self.convert(cell))
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("conv_cell "+str(conv_cell))
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()
    goal = PlayMotionGoal()
    goal.motion_name = "p" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional
    rospy.loginfo("Sending goal with motion: " + "p" + str(conv_cell))
    self.client.send_goal(goal)
    rospy.loginfo("Waiting for result...")
    action_ok = self.client.wait_for_result(rospy.Duration(30.0))
    state = self.client.get_state()
    if action_ok:
      rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
      if action_ok:
        activate = self.activate_magnet()
        if activate == False:
          return False
        else:
          rospy.loginfo("Starting run_motion_python application...")
          self.wait_for_valid_time(10.0)
          rospy.loginfo("Waiting for Action Server...")
          self.client.wait_for_server()
          goal = PlayMotionGoal()
          goal.motion_name = "c" + str(conv_cell)
          goal.skip_planning = False
          goal.priority = 0  # Optional
          rospy.loginfo("Sending goal with motion: " + "c" + str(conv_cell))
          self.client.send_goal(goal)

          rospy.loginfo("Waiting for result...")
          action_ok = self.client.wait_for_result(rospy.Duration(30.0))
          state = self.client.get_state()
          if action_ok:
            rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
            return True
    else:
      rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
      return False

  def place(self, cell):
    '''
    Robot places the token in the cell location
    Args:
      cell: the location on the baord
    Return:
      true if the place has been successfully executed
    '''
    # cell 1 is g45 for the robot view point
    conv_cell = int(self.convert(cell))
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "p" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + "p" + str(conv_cell))
    self.client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    action_ok = self.client.wait_for_result(rospy.Duration(30.0))
    state = self.client.get_state()
    if action_ok:
      rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
      if action_ok:
        deactivate = self.deactivate_magnet()
        if deactivate == False:
          return False
        else:
          rospy.loginfo("Starting run_motion_python application...")
          self.wait_for_valid_time(10.0)
          # client = SimpleActionClient('/play_motion', PlayMotionAction)
          rospy.loginfo("Waiting for Action Server...")
          self.client.wait_for_server()

          goal = PlayMotionGoal()
          goal.motion_name = "c" + str(conv_cell)
          goal.skip_planning = False
          goal.priority = 0  # Optional

          rospy.loginfo("Sending goal with motion: " + "c" + str(conv_cell))
          self.client.send_goal(goal)

          rospy.loginfo("Waiting for result...")
          action_ok = self.client.wait_for_result(rospy.Duration(30.0))
          state = self.client.get_state()
          if action_ok:
            rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
            return True
    else:
      rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
      return False

  def pick_and_place(self, from_, to_):
    '''
    Method to move a token from a location to another
    Args:
      from_: location
      to_: location
    :Return:
      true if the pick and place has been successfully executed
    '''
    pick_success = self.pick(from_)
    if pick_success:
      place_success = self.place(to_)
      return place_success

  def suggest_row(self, row):
    '''
    Suggest the row where the solution is
    Args:
      row: row of the board
    Return:
       none
    '''
    # cell 1 for first row, 2 for the second and so on
    conv_cell = int(row)
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "sr" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + "sr" + str(conv_cell))
    self.client.send_goal(goal)
    rospy.loginfo("Execute action without waiting for result...")


  def suggest_subset(self, cell):
    '''
    Suggest a subset of three tokens on a row
    Args:
      cell: the solution location
    Return:
      None
    '''
    # cell 1 is g45 for the robot view point
    conv_cell = int(self.convert(cell))
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "ss" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + "ss" + str(conv_cell))
    self.client.send_goal(goal)
    rospy.loginfo("Execute action without waiting for result...")


  def suggest_solution(self, cell):
    '''
    Robot points at the location of cell
    Args:
      cell: solution location
    Return:
       None
    '''
    # cell 1 is g45 for the robot view point
    conv_cell = int(self.convert(cell))
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")
    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "s" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + "s" + str(conv_cell))
    self.client.send_goal(goal)
    rospy.loginfo("Execute action without waiting for result...")


  def offer_token(self, token_loc, reproduce_text, text):
    # cell 1 is g45 for the robot view point
    conv_cell = int(self.convert(token_loc))
    rospy.loginfo("Starting run_motion_python application...")
    self.wait_for_valid_time(10.0)
    # client = SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Waiting for Action Server...")

    self.client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = "p" + str(conv_cell)
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + "p" + str(conv_cell))
    self.client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    action_ok = self.client.wait_for_result(rospy.Duration(30.0))
    state = self.client.get_state()
    if action_ok:
      rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
      if action_ok:
        activate = self.activate_magnet()
        if activate == False:
          return False
        else:
          rospy.loginfo("Starting run_motion_python application...")
          self.wait_for_valid_time(10.0)
          # client = SimpleActionClient('/play_motion', PlayMotionAction)
          rospy.loginfo("Waiting for Action Server...")
          self.client.wait_for_server()

          goal = PlayMotionGoal()
          goal.motion_name = "o" + str(conv_cell)
          goal.skip_planning = False
          goal.priority = 0  # Optional

          rospy.loginfo("Sending goal with motion: " + "o" + str(conv_cell))
          self.client.send_goal(goal)

          reproduce_text(text)

          rospy.loginfo("Waiting for result...")
          action_ok = self.client.wait_for_result(rospy.Duration(30.0))
          state = self.client.get_state()
          if action_ok:
            rospy.loginfo("Action finished succesfully with state: " + str(self.get_status_string(state)))
            rospy.sleep(2.0)
            deactivate = self.deactivate_magnet()
            if deactivate == False:
              return False
            return True
    else:
      rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
      return False

  def cancel_motion(self):
    if (self.client.get_state()) == 1:
      rospy.loginfo("canceling action")
      self.client.cancel_goal()
      rospy.loginfo("goal has been canceled")
      print("cancel action")

  def get_action_state(self):
    state = self.client.get_state()
    if state != 3:
      #action still running
      return 0
    else:
      #action completed
      return 1

# def main():
#   robot_gesture = Gesture()
#   robot_speech = Speech("en_GB")
#
#   #tiago.wave()
#   robot_gesture.pick_and_place(18, 1)
#   rospy.sleep(2)
#   robot_gesture.pick_and_place(1, 18)
#   rospy.sleep(2)
#   robot_gesture.pick_and_place(12, 2)
#   rospy.sleep(2)
#   robot_gesture.pick_and_place(2, 12)
#   rospy.sleep(2)
#   robot_gesture.pick_and_place(15, 3)
#   rospy.sleep(2)
#   robot_gesture.pick_and_place(3, 15)
#   rospy.sleep(2)
#
#
# if __name__ == '__main__':
#   main()



