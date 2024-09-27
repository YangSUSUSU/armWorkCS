import rospy
import time
from llm_msgs.msg import hand_pose_req, pose_action_status, robot_state
from llm_msgs.srv import set_force, set_forceRequest, set_forceResponse, set_speed, set_speedRequest, set_speedResponse, set_angle, set_angleRequest, set_angleResponse, get_angle_act, get_angle_actResponse
# print(set_angle.__dict__)

if __name__ == "__main__":    
    
    # 创建节点
    rospy.init_node('client_node')
 
    def callback(msg):
        if msg.gripper_move_enable == 1:
            if msg.gripper_side == 0:
                rospy.wait_for_service("/inspire_hand/set_angle/left_hand")
                rospy.wait_for_service("/inspire_hand/set_force/left_hand")
                rospy.wait_for_service("/inspire_hand/set_speed/left_hand")
                rospy.wait_for_service("/inspire_hand/set_angle/left_hand")
                rospy.wait_for_service("/inspire_hand/get_angle/left_hand")

                set_force_left_hand = rospy.ServiceProxy('/inspire_hand/set_force/left_hand', set_force)
                set_speed_left_hand = rospy.ServiceProxy('/inspire_hand/set_speed/left_hand', set_speed)
                set_angle_left_hand = rospy.ServiceProxy('/inspire_hand/set_angle/left_hand', set_angle)
                get_angle_left_hand = rospy.ServiceProxy('/inspire_hand/get_angle/left_hand', get_angle_act)
                
                left_hand_force = set_forceRequest()
                left_hand_speed = set_speedRequest()
                left_hand_angle = set_angleRequest()
                left_hand_force.force0Ratio = 0.999
                left_hand_force.force1Ratio = 0.999
                left_hand_force.force2Ratio = 0.999
                left_hand_force.force3Ratio = 0.999
                left_hand_force.force4Ratio = 0.999
                left_hand_force.force5Ratio = 0.999
                if msg.finger_status == 0:
                    left_hand_speed.speed0Ratio = 0.9
                    left_hand_speed.speed1Ratio = 0.9
                    left_hand_speed.speed2Ratio = 0.9
                    left_hand_speed.speed3Ratio = 0.9
                    left_hand_speed.speed4Ratio = 0.9
                    left_hand_speed.speed5Ratio = 0.9
                else:
                    left_hand_speed.speed0Ratio = 0.3
                    left_hand_speed.speed1Ratio = 0.3
                    left_hand_speed.speed2Ratio = 0.3
                    left_hand_speed.speed3Ratio = 0.3
                    left_hand_speed.speed4Ratio = 0.3
                    left_hand_speed.speed5Ratio = 0.3
                

                left_hand_angle.angle0Ratio = msg.little_finger
                left_hand_angle.angle1Ratio = msg.ring_finger
                left_hand_angle.angle2Ratio = msg.middle_finger
                left_hand_angle.angle3Ratio = msg.index_finger
                left_hand_angle.angle4Ratio = msg.thumb_bending
                left_hand_angle.angle5Ratio = msg.thumb_rotating

                for idx in range(5):
                    force_resp = set_force_left_hand(left_hand_force)
                    speed_resp = set_speed_left_hand(left_hand_speed)
                    angle_resp = set_angle_left_hand(left_hand_angle)
                    print(f"{idx}th left hand cmd send, msg: {left_hand_angle}")
                    time.sleep(0.02)
                print(f"left_hand_angle:{left_hand_angle}")

                print(f"force_resp:{force_resp.force_accepted}")
                if not (force_resp.force_accepted and speed_resp.speed_accepted and angle_resp.angle_accepted):
                	raise Exception("Communication Lost")
                # client(cur_ang_ratio)
            elif msg.gripper_side == 1:
                rospy.wait_for_service("/inspire_hand/set_angle/right_hand")
                rospy.wait_for_service("/inspire_hand/set_force/right_hand")
                rospy.wait_for_service("/inspire_hand/set_speed/right_hand")
                rospy.wait_for_service("/inspire_hand/set_angle/right_hand")
                rospy.wait_for_service("/inspire_hand/get_angle/right_hand")

                set_force_right_hand = rospy.ServiceProxy('/inspire_hand/set_force/right_hand', set_force)
                set_speed_right_hand = rospy.ServiceProxy('/inspire_hand/set_speed/right_hand', set_speed)
                set_angle_right_hand = rospy.ServiceProxy('/inspire_hand/set_angle/right_hand', set_angle)
                get_angle_right_hand = rospy.ServiceProxy('/inspire_hand/get_angle/right_hand', get_angle_act)
                
                right_hand_force = set_forceRequest()
                right_hand_speed = set_speedRequest()
                right_hand_angle = set_angleRequest()
                right_hand_force.force0Ratio = 0.999
                right_hand_force.force1Ratio = 0.999
                right_hand_force.force2Ratio = 0.999
                right_hand_force.force3Ratio = 0.999
                right_hand_force.force4Ratio = 0.999
                right_hand_force.force5Ratio = 0.999
                if msg.finger_status == 0:
                    right_hand_speed.speed0Ratio = 0.9
                    right_hand_speed.speed1Ratio = 0.9
                    right_hand_speed.speed2Ratio = 0.9
                    right_hand_speed.speed3Ratio = 0.9
                    right_hand_speed.speed4Ratio = 0.9
                    right_hand_speed.speed5Ratio = 0.9
                else:
                    right_hand_speed.speed0Ratio = 0.3
                    right_hand_speed.speed1Ratio = 0.3
                    right_hand_speed.speed2Ratio = 0.3
                    right_hand_speed.speed3Ratio = 0.3
                    right_hand_speed.speed4Ratio = 0.3
                    right_hand_speed.speed5Ratio = 0.3
                right_hand_angle.angle0Ratio = msg.little_finger
                right_hand_angle.angle1Ratio = msg.ring_finger
                right_hand_angle.angle2Ratio = msg.middle_finger
                right_hand_angle.angle3Ratio = msg.index_finger
                right_hand_angle.angle4Ratio = msg.thumb_bending
                right_hand_angle.angle5Ratio = msg.thumb_rotating
                print(f"right_hand_angle:{right_hand_angle}")
                for idx in range(5):
                    force_resp = set_force_right_hand(right_hand_force)
                    speed_resp = set_speed_right_hand(right_hand_speed)
                    angle_resp = set_angle_right_hand(right_hand_angle)
                    print(f"{idx}th right hand cmd send, msg: {right_hand_angle}")
                    time.sleep(0.01)

                print(f"force_resp:{force_resp.force_accepted}")
                if not (force_resp.force_accepted and speed_resp.speed_accepted and angle_resp.angle_accepted):
                	raise Exception("Communication Lost")
            elif msg.gripper_side == 2:
                pass

    # 创建节点
    rospy.init_node('client_node')
    
    status_sub= rospy.Subscriber("hand_pose_req",
                                       hand_pose_req,
                                       callback,
                                       queue_size = 10)
    
    rate = rospy.Rate(20)  # 设置循环频率为 10Hz

    while not rospy.is_shutdown():  # 当节点未被关闭时执行循环
        # 在这里执行节点的主要操作
        #print(time.time())
        # rospy.wait_for_service("/inspire_hand/get_angle/left_hand")
        # rospy.wait_for_service("/inspire_hand/get_angle/right_hand")
        # get_angle_left_hand = rospy.ServiceProxy('/inspire_hand/get_angle/left_hand', get_angle_act)
        # get_angle_right_hand = rospy.ServiceProxy('/inspire_hand/get_angle/right_hand', get_angle_act)
        # left_act_angle = get_angle_left_hand()
        # right_act_angle = get_angle_right_hand()
        # print(f"left_act_angle:{left_act_angle}, right_act_angle:{right_act_angle}")
        
        rate.sleep()  # 控制循环频率塞代码
   
