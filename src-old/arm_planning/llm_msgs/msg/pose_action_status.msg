std_msgs/Header header
# 相应的hand_pose_req msg中header的seq,表示是对该帧pose的相应
uint32 resp_frame_id     
# 0-失败， 1-成功， 2-默认值(default），其他数值暂不可用，留待后续扩展   
uint8 hand_move_success      
uint8 head_move_success     
uint8 gripper_move_success 
uint8 waist_move_success 

