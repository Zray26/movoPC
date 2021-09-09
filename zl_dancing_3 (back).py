#!/usr/bin/env python
A = [-1.55421128248, -1.1, 0.0, -1.4, -0.048272129981, -0.3, -2.7] #pos1 [0, 0.4, 0, 1.2, 0, -0.8, 1]
B = [-1.55425711313, -0.4, 4.00252722139e-05, -0.5, -0.0481304607423, -0.304701036852, -2.69997092895] # pos2 [0, 0.7, 0, 0.9, 0, 0, 0]
C = [-1.5543041769, -0.3, 1.34736233486e-05, -0.9, -0.0481572457238, -0.309260889935, -2.69990890996] # pos3 [0, 0.1, 0, -0.4, 0, 0, 0]
D = [-1.5543041769, -0.3, 1.34736233486e-05, -1.2, -0.0481572457238, -0.9, -2.69990890996] #final pos #pos4 [0, 0, 0, -0.3, 0, -0.6, 0]
default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
    1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
    0.25, 0, 0]
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
#!/usr/bin/env python
import rospy
import sys
import copy
import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
import math
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time

if __name__=="__main__":
    rospy.init_node("zl_dancing_3",
                    anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    # tuck = move_to_tuck()
    # tuck.move_to_initial_pose()
    # del tuck
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
    lgripper.command(gripper_closed,block=False)
    rgripper.command(gripper_closed, block=False)
    print("Done spinning up MoveIt!")
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, default_pose_tucked, 0.005, wait=True)
        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    lgripper.command(gripper_open,block=False)
    rgripper.command(gripper_open, block=False)
    ods = odyssey_Interface()
    # ods._L0_dual_jp_move_safe_relate(
    #     jp_r=[0, 0, 0, 0, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
    #     jp_l=[0, 0, 0, 0, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
    #     duration=5) 
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0.4, -0.1, 1.2, 0, -0.8, 1.4], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, -0.4, 0.1, -1.2, 0, 0.8, -1.4], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=4) #initial pose
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, 0, 0, 0, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=3) #wait 
    #raw_input()

    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0.7, 0, 0.9, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, -0.7, 0, -0.9, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=2)
    #raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0.1, 0, -0.4, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, -0.1, 0, 0.4, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=1.5) 
    #raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, -0.3, 0, -0.6, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, 0, 0, 0.3, 0, 0.6, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=1.5) #move up
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, 0, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, 0, 0, 0, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=10) #wait 
    #raw_input()

    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, -0.8, 0, -0.2, 0, 0.6, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, 0.8, 0, 0.2, 0, -0.6, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=3) #movedown

    #raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0.7, 0, 0.9, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, -0.7, 0, -0.9, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=2) 
    #raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0.1, 0, -0.4, 0, 0, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, -0.1, 0, 0.4, 0, 0, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=1.5) 
    #raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, 0, 0, -0.3, 0, -0.6, 0], rmaxforce=[20, 20, 20, 20, 20, 20],
        jp_l=[0, 0, 0, 0.3, 0, 0.6, 0], lmaxforce=[20, 20, 20, 20, 20, 20],
        duration=1.5) #move up
