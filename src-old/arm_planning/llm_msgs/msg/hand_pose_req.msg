std_msgs/Header header     

# 以下分别为手臂、头部、手指对应的控制数据，相应数据仅当对应使能位有效时才有意义
# 例如: 当hand_move_enable为1(enable状态)时, pose_req的数据才需要被执行，才有意义

# 手臂移动使能位-hand_move_enable, 手臂目标位姿-pose_req, 手臂复位-hand_reset(0-非复位位姿，1-复位位姿)
uint8 hand_move_enable    
uint8 hand_side 
uint8 hand_reset          
geometry_msgs/Pose pose_req 

# 头部移动使能位-move_enable, 头部目标位姿-(R, P, Y）
uint8 head_move_enable    
float64[] rpy

# 手部手指移动使能位-gripper_move_enable, 手指开度-[figure_1, figure_2, figure_3, figure_4, figure_5]
uint8 gripper_move_enable 
uint8 gripper_side  
uint8 finger_status
float32 little_finger
float32 ring_finger
float32 middle_finger
float32 index_finger
float32 thumb_bending
float32 thumb_rotating

# 腰部转动
uint8 waist_enable # 0-disable, 1-enable, 2-default, 其他数值暂不可用，留待后续
float32 waist_rotate_ang # 腰部旋转角度-rad

