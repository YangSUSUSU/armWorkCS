;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::hand_pose_req)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'hand_pose_req (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::HAND_POSE_REQ")
  (make-package "LLM_MSGS::HAND_POSE_REQ"))

(in-package "ROS")
;;//! \htmlinclude hand_pose_req.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass llm_msgs::hand_pose_req
  :super ros::object
  :slots (_header _hand_move_enable _hand_side _hand_reset _pose_req _head_move_enable _rpy _gripper_move_enable _gripper_side _finger_status _little_finger _ring_finger _middle_finger _index_finger _thumb_bending _thumb_rotating _waist_enable _waist_rotate_ang ))

(defmethod llm_msgs::hand_pose_req
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:hand_move_enable __hand_move_enable) 0)
    ((:hand_side __hand_side) 0)
    ((:hand_reset __hand_reset) 0)
    ((:pose_req __pose_req) (instance geometry_msgs::Pose :init))
    ((:head_move_enable __head_move_enable) 0)
    ((:rpy __rpy) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:gripper_move_enable __gripper_move_enable) 0)
    ((:gripper_side __gripper_side) 0)
    ((:finger_status __finger_status) 0)
    ((:little_finger __little_finger) 0.0)
    ((:ring_finger __ring_finger) 0.0)
    ((:middle_finger __middle_finger) 0.0)
    ((:index_finger __index_finger) 0.0)
    ((:thumb_bending __thumb_bending) 0.0)
    ((:thumb_rotating __thumb_rotating) 0.0)
    ((:waist_enable __waist_enable) 0)
    ((:waist_rotate_ang __waist_rotate_ang) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _hand_move_enable (round __hand_move_enable))
   (setq _hand_side (round __hand_side))
   (setq _hand_reset (round __hand_reset))
   (setq _pose_req __pose_req)
   (setq _head_move_enable (round __head_move_enable))
   (setq _rpy __rpy)
   (setq _gripper_move_enable (round __gripper_move_enable))
   (setq _gripper_side (round __gripper_side))
   (setq _finger_status (round __finger_status))
   (setq _little_finger (float __little_finger))
   (setq _ring_finger (float __ring_finger))
   (setq _middle_finger (float __middle_finger))
   (setq _index_finger (float __index_finger))
   (setq _thumb_bending (float __thumb_bending))
   (setq _thumb_rotating (float __thumb_rotating))
   (setq _waist_enable (round __waist_enable))
   (setq _waist_rotate_ang (float __waist_rotate_ang))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:hand_move_enable
   (&optional __hand_move_enable)
   (if __hand_move_enable (setq _hand_move_enable __hand_move_enable)) _hand_move_enable)
  (:hand_side
   (&optional __hand_side)
   (if __hand_side (setq _hand_side __hand_side)) _hand_side)
  (:hand_reset
   (&optional __hand_reset)
   (if __hand_reset (setq _hand_reset __hand_reset)) _hand_reset)
  (:pose_req
   (&rest __pose_req)
   (if (keywordp (car __pose_req))
       (send* _pose_req __pose_req)
     (progn
       (if __pose_req (setq _pose_req (car __pose_req)))
       _pose_req)))
  (:head_move_enable
   (&optional __head_move_enable)
   (if __head_move_enable (setq _head_move_enable __head_move_enable)) _head_move_enable)
  (:rpy
   (&optional __rpy)
   (if __rpy (setq _rpy __rpy)) _rpy)
  (:gripper_move_enable
   (&optional __gripper_move_enable)
   (if __gripper_move_enable (setq _gripper_move_enable __gripper_move_enable)) _gripper_move_enable)
  (:gripper_side
   (&optional __gripper_side)
   (if __gripper_side (setq _gripper_side __gripper_side)) _gripper_side)
  (:finger_status
   (&optional __finger_status)
   (if __finger_status (setq _finger_status __finger_status)) _finger_status)
  (:little_finger
   (&optional __little_finger)
   (if __little_finger (setq _little_finger __little_finger)) _little_finger)
  (:ring_finger
   (&optional __ring_finger)
   (if __ring_finger (setq _ring_finger __ring_finger)) _ring_finger)
  (:middle_finger
   (&optional __middle_finger)
   (if __middle_finger (setq _middle_finger __middle_finger)) _middle_finger)
  (:index_finger
   (&optional __index_finger)
   (if __index_finger (setq _index_finger __index_finger)) _index_finger)
  (:thumb_bending
   (&optional __thumb_bending)
   (if __thumb_bending (setq _thumb_bending __thumb_bending)) _thumb_bending)
  (:thumb_rotating
   (&optional __thumb_rotating)
   (if __thumb_rotating (setq _thumb_rotating __thumb_rotating)) _thumb_rotating)
  (:waist_enable
   (&optional __waist_enable)
   (if __waist_enable (setq _waist_enable __waist_enable)) _waist_enable)
  (:waist_rotate_ang
   (&optional __waist_rotate_ang)
   (if __waist_rotate_ang (setq _waist_rotate_ang __waist_rotate_ang)) _waist_rotate_ang)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _hand_move_enable
    1
    ;; uint8 _hand_side
    1
    ;; uint8 _hand_reset
    1
    ;; geometry_msgs/Pose _pose_req
    (send _pose_req :serialization-length)
    ;; uint8 _head_move_enable
    1
    ;; float64[] _rpy
    (* 8    (length _rpy)) 4
    ;; uint8 _gripper_move_enable
    1
    ;; uint8 _gripper_side
    1
    ;; uint8 _finger_status
    1
    ;; float32 _little_finger
    4
    ;; float32 _ring_finger
    4
    ;; float32 _middle_finger
    4
    ;; float32 _index_finger
    4
    ;; float32 _thumb_bending
    4
    ;; float32 _thumb_rotating
    4
    ;; uint8 _waist_enable
    1
    ;; float32 _waist_rotate_ang
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _hand_move_enable
       (write-byte _hand_move_enable s)
     ;; uint8 _hand_side
       (write-byte _hand_side s)
     ;; uint8 _hand_reset
       (write-byte _hand_reset s)
     ;; geometry_msgs/Pose _pose_req
       (send _pose_req :serialize s)
     ;; uint8 _head_move_enable
       (write-byte _head_move_enable s)
     ;; float64[] _rpy
     (write-long (length _rpy) s)
     (dotimes (i (length _rpy))
       (sys::poke (elt _rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _gripper_move_enable
       (write-byte _gripper_move_enable s)
     ;; uint8 _gripper_side
       (write-byte _gripper_side s)
     ;; uint8 _finger_status
       (write-byte _finger_status s)
     ;; float32 _little_finger
       (sys::poke _little_finger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ring_finger
       (sys::poke _ring_finger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _middle_finger
       (sys::poke _middle_finger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _index_finger
       (sys::poke _index_finger (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thumb_bending
       (sys::poke _thumb_bending (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thumb_rotating
       (sys::poke _thumb_rotating (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _waist_enable
       (write-byte _waist_enable s)
     ;; float32 _waist_rotate_ang
       (sys::poke _waist_rotate_ang (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _hand_move_enable
     (setq _hand_move_enable (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hand_side
     (setq _hand_side (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hand_reset
     (setq _hand_reset (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Pose _pose_req
     (send _pose_req :deserialize buf ptr-) (incf ptr- (send _pose_req :serialization-length))
   ;; uint8 _head_move_enable
     (setq _head_move_enable (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64[] _rpy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rpy (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; uint8 _gripper_move_enable
     (setq _gripper_move_enable (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gripper_side
     (setq _gripper_side (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _finger_status
     (setq _finger_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _little_finger
     (setq _little_finger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ring_finger
     (setq _ring_finger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _middle_finger
     (setq _middle_finger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _index_finger
     (setq _index_finger (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thumb_bending
     (setq _thumb_bending (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thumb_rotating
     (setq _thumb_rotating (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _waist_enable
     (setq _waist_enable (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _waist_rotate_ang
     (setq _waist_rotate_ang (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get llm_msgs::hand_pose_req :md5sum-) "b6b3b996b8f56c6f6190cb1b04d5143a")
(setf (get llm_msgs::hand_pose_req :datatype-) "llm_msgs/hand_pose_req")
(setf (get llm_msgs::hand_pose_req :definition-)
      "std_msgs/Header header     

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


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :llm_msgs/hand_pose_req "b6b3b996b8f56c6f6190cb1b04d5143a")


