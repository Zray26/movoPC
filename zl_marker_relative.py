#!/usr/bin/env python

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
# import turtlesim.srv
global marker_x
global marker_y
global marker_z
global right_x
global right_y
global right_z
global left_x
global left_y
global left_z
marker_x=0
marker_y=0
marker_z=0
right_x=0
right_y=0
right_z=0
left_x=0
left_y=0
left_z=0

def marker_cb(data):
    print("marker_x is ")
    marker_x = data.translation.x
    marker_y = data.translation.y
    marker_z = data.translation.z
    print(marker_x)

def right_cb(data):
    right_x = data.translation.x
    right_y = data.translation.y
    right_z = data.translation.z

def left_cb(data):
    print("left x is ")
    left_x = data.translation.x
    left_y = data.translation.y
    left_z = data.translation.z
    left_trans = geometry_msgs.msg.Transform()
    left_trans.translation.x= marker_x - left_x
    left_trans.translation.y= marker_y - left_y
    left_trans.translation.z= marker_z - left_z
    print(left_trans.translation.x)
    # print(left_x)
    left_transform_info.publish(left_trans)

if __name__ == '__main__':
    rospy.init_node('marker_to_arm')
    rospy.Subscriber("/marker",geometry_msgs.msg.Transform,marker_cb,queue_size = 1,buff_size = 2**24)
    rospy.Subscriber("/left_ee_distance",geometry_msgs.msg.Transform,left_cb,queue_size = 1,buff_size = 2**24)
    rospy.Subscriber("/right_ee_distance",geometry_msgs.msg.Transform,right_cb,queue_size = 1,buff_size = 2**24)
    left_transform_info = rospy.Publisher('left_to_marker', geometry_msgs.msg.Transform,queue_size=1)
    right_transform_info = rospy.Publisher('right_to_marker', geometry_msgs.msg.Transform,queue_size=1)
    rospy.spin()
    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # marker_transform_info = rospy.Publisher('marker', geometry_msgs.msg.Transform,queue_size=1)

    rate = rospy.Rate(150)
    while not rospy.is_shutdown():
        # print(left_x)
        
        # print(tra ns,rot)
        left_trans = geometry_msgs.msg.Transform()
        left_trans.translation.x= marker_x - left_x
        left_trans.translation.y= marker_y - left_y
        left_trans.translation.z= marker_z - left_z

        right_trans = geometry_msgs.msg.Transform()
        right_trans.translation.x= marker_x - right_x
        right_trans.translation.y= marker_y - right_y
        right_trans.translation.z= marker_z - right_z

        left_transform_info.publish(left_trans)
        right_transform_info.publish(right_trans)

        rate.sleep()