#!/usr/bin/env python
import rospy
import sys
import copy
import tf
from hrclib_client_v6 import odyssey_Interface
from std_msgs.msg import String
from std_msgs.msg import Bool
import geometry_msgs.msg 
import math
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time
# import zl_ods_contact

_upper_body_joints = ["right_shoulder_pan_joint",
                      "right_shoulder_lift_joint",
                      "right_arm_half_joint",
                      "right_elbow_joint",
                      "right_wrist_spherical_1_joint",
                      "right_wrist_spherical_2_joint",
                      "right_wrist_3_joint",
                      "left_shoulder_pan_joint",
                      "left_shoulder_lift_joint",
                      "left_arm_half_joint",
                      "left_elbow_joint",
                      "left_wrist_spherical_1_joint",
                      "left_wrist_spherical_2_joint",
                      "left_wrist_3_joint",
                      "linear_joint",
                      "pan_joint",
                      "tilt_joint"]
                      
# Head looking straight
default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
                       1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
                       0.25, 0, 0]
pose1 = [0, -1.5, 0.1, -1.57, 0.0, 1.57, -1.69,
                       0, 1.5, -0.1, 1.57, 0.0, -1.57, 1.69,
                       0.25, 0, 0]

#temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("zl_dancing_2_moveit",
                    anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')
    gripper_closed = 0.00
    gripper_open = 0.165
    
    larm_group = moveit_commander.MoveGroupCommander("left_arm")
    rarm_group = moveit_commander.MoveGroupCommander("right_arm")
    upper_body = moveit_commander.MoveGroupCommander("upper_body")
    head_group = moveit_commander.MoveGroupCommander("head")

    move_group = MoveGroupInterface("upper_body", "base_link")
    lmove_group = MoveGroupInterface("left_arm", "base_link")
    rmove_group = MoveGroupInterface("right_arm", "base_link")
    hmove_group = MoveGroupInterface("head","base_link")
    print("Done spinning up MoveIt!")
    # upper_body.go(joints=[])
    # larm_group.
    planning_state = rospy.wait_for_message("/dancing_planning_state",Bool)
    print(planning_state.data)
    # time.sleep(10)
    # planning_state_state.data = True
    # succ = True
    if (planning_state.data == True):
    # if (succ == True):
        time.sleep(108)
        head_goal = head_group.get_current_joint_values()
        head_goal[1] = -0.6
        head_group.go(head_goal)
        head_goal[1]= 0.6
        head_group.go(head_goal)
        head_goal[1]= 0
        head_group.go(head_goal)
        time.sleep(5)
        lgripper.command(gripper_closed,block=False)
        rgripper.command(gripper_closed, block=False)
        time.sleep(4)
        lgripper.command(gripper_open,block=False)
        rgripper.command(gripper_open, block=False)
        time.sleep(10)
        lgripper.command(gripper_closed,block=False)
        rgripper.command(gripper_closed, block=False)
        time.sleep(4)
        lgripper.command(gripper_open,block=False)
        rgripper.command(gripper_open, block=False)