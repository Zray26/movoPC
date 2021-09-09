import rospy
# import tf
from hrclib_readonly.hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
pick_pos_dict={'h': (9.853695701167453e-06, -0.6027460694313049), 'lin': 0.4585753381252289, 'l': (-0.13635384781205717, 0.9114743399306715, 0.850791817648235, 1.2031456067289128, -0.3104654609270394, -1.3138586727237413, -0.8174240070861964), 'r': (0.2505741638044654, -0.9006784175037083, -0.4091691351442108, -1.670779516757796, 0.6212879185982039, 1.0429657882244618, -2.4777225202159308)}
import math
# class temp_pickbolt_client(object):
#     def __init__(self):
#         rospy.init_node('transform_subscriber', anonymous= True)
#         rospy.Subscriber("/marker",geometry_msgs.msg.Transform,self.marker_CallBack)
        
#         self.msg=None
#     def marker_CallBack(self,msg):
#         print(msg)
#         self.msg=msg

# temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("Pick_Bolt_Test")
    

    # ods.grip("right",1)
    # ods.grip("right",0)
    # ods.grip("right",1)



############################ Pick Bolt ##################################################################
    ods=odyssey_Interface()

    ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.3],
                                  [0.1+math.pi/2, math.pi / 2, -math.pi / 2],
                                  [20 for i in range(6)])
    ods.grip("right",0)# g open
    print("Initial pose reached, ready to pick")
    input()
    marker_4 = rospy.wait_for_message("/marker4",geometry_msgs.msg.Transform)
    trans_4 = marker4.translation
    rots_4 = marker4.rotation
        # rospy.init_node("wait_for_msg")

    # print(trans)
    ods._L0_single_task_move_safe("right",[trans_4.x - 0.005, trans_4.y - 0.065,  0.93824+0.3],
                                  [0.1+math.pi/2, math.pi / 2, -math.pi / 2],
                                  [20 for i in range(6)])
    print("Above the bolt,ready to go down")
    input()
    ods._L0_single_task_move_safe("right",[trans_4.x - 0.005, trans_4.y - 0.065,  trans_4.z+0.1],
                                  [0.1+math.pi/2, math.pi / 2, -math.pi / 2],
                                  [20 for i in range(6)])
    input()
    ods.grip("right",1)
    # ods.single_move_relate(arm="right",move=[0,0,-0.09],maxforce=[25 for i in range(6)],time=2)
    # ods.grip("right",0)
    # ods.single_move_relate(arm="right",move=[0,0,0.09],maxforce=[25 for i in range(6)],time=2)
    
#########################################################################################################

############################### Follow Marker ###########################################################
    # while not rospy.is_shutdown():
    #     ods=odyssey_Interface()
    #     ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                             [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                             [20 for i in range(6)])
    #     # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    #     ods._L0_single_task_move_safe("right",[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.3],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    #     rospy.Rate(0.1).sleep()
#########################################################################################################


#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#     #ods.go_upper_default_jp(posdict=ods.gval.default_pose_ready_insert_and_screw,hard=False,f=15)
#     print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
# #    ods.get_rpose
#     print('The position of marker0:', ods.marker0)#[0.9548131752342998, -0.30152793713899934, 0.9200864503752137]
# #    ods.arm_cart_move(arm="right",pos=[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.1],orn=[0.2+math.pi/2, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

# #    ods._L0_single_task_move_safe(arm="right",pos=[ods.rpose[0],ods.rpose[1],ods.rpose[2]+0.1],orn=[0, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

#     ods._L0_single_task_move_safe("right",[0.7263, -0.309544, 0.93824+0.2],
#                                   [0.1, math.pi / 2, -math.pi / 2],
#                                   [50 for i in range(6)],hard=True)
                                
#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#    ods._L0_single_task_move_safe("right",[0.74,-0.25,1.17],[0, math.pi / 2, -math.pi / 2],  [20 for i in range(6)])
    # print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
    #tp=temp_pickbolt_client()
       
#    print(ods.lpose)
#    print(ods.rpose)
#    print(ods.get_jp_dict())
    # ods.get
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    # ods.get_jp_dict()
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)

    # ods.set_grippers(0)
    # ods.set_grippers(1)
    # ods.grip(rl="left",v=1)
    # ods.grip(rl="left",v=0)
    # ods.grip(rl="left",v=1)



def subscriber():
    rospy.init_node('transform_subscriber', anonymous= True)
    rospy.Subscriber()