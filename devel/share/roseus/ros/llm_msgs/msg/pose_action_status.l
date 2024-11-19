;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::pose_action_status)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'pose_action_status (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::POSE_ACTION_STATUS")
  (make-package "LLM_MSGS::POSE_ACTION_STATUS"))

(in-package "ROS")
;;//! \htmlinclude pose_action_status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass llm_msgs::pose_action_status
  :super ros::object
  :slots (_header _resp_frame_id _hand_move_success _head_move_success _gripper_move_success _waist_move_success ))

(defmethod llm_msgs::pose_action_status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:resp_frame_id __resp_frame_id) 0)
    ((:hand_move_success __hand_move_success) 0)
    ((:head_move_success __head_move_success) 0)
    ((:gripper_move_success __gripper_move_success) 0)
    ((:waist_move_success __waist_move_success) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _resp_frame_id (round __resp_frame_id))
   (setq _hand_move_success (round __hand_move_success))
   (setq _head_move_success (round __head_move_success))
   (setq _gripper_move_success (round __gripper_move_success))
   (setq _waist_move_success (round __waist_move_success))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:resp_frame_id
   (&optional __resp_frame_id)
   (if __resp_frame_id (setq _resp_frame_id __resp_frame_id)) _resp_frame_id)
  (:hand_move_success
   (&optional __hand_move_success)
   (if __hand_move_success (setq _hand_move_success __hand_move_success)) _hand_move_success)
  (:head_move_success
   (&optional __head_move_success)
   (if __head_move_success (setq _head_move_success __head_move_success)) _head_move_success)
  (:gripper_move_success
   (&optional __gripper_move_success)
   (if __gripper_move_success (setq _gripper_move_success __gripper_move_success)) _gripper_move_success)
  (:waist_move_success
   (&optional __waist_move_success)
   (if __waist_move_success (setq _waist_move_success __waist_move_success)) _waist_move_success)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _resp_frame_id
    4
    ;; uint8 _hand_move_success
    1
    ;; uint8 _head_move_success
    1
    ;; uint8 _gripper_move_success
    1
    ;; uint8 _waist_move_success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _resp_frame_id
       (write-long _resp_frame_id s)
     ;; uint8 _hand_move_success
       (write-byte _hand_move_success s)
     ;; uint8 _head_move_success
       (write-byte _head_move_success s)
     ;; uint8 _gripper_move_success
       (write-byte _gripper_move_success s)
     ;; uint8 _waist_move_success
       (write-byte _waist_move_success s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _resp_frame_id
     (setq _resp_frame_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _hand_move_success
     (setq _hand_move_success (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _head_move_success
     (setq _head_move_success (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gripper_move_success
     (setq _gripper_move_success (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _waist_move_success
     (setq _waist_move_success (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get llm_msgs::pose_action_status :md5sum-) "d102c2efc0ecfb1e3d8d0927cd612413")
(setf (get llm_msgs::pose_action_status :datatype-) "llm_msgs/pose_action_status")
(setf (get llm_msgs::pose_action_status :definition-)
      "std_msgs/Header header
# 相应的hand_pose_req msg中header的seq,表示是对该帧pose的相应
uint32 resp_frame_id     
# 0-失败， 1-成功， 2-默认值(default），其他数值暂不可用，留待后续扩展   
uint8 hand_move_success      
uint8 head_move_success     
uint8 gripper_move_success 
uint8 waist_move_success 


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

")



(provide :llm_msgs/pose_action_status "d102c2efc0ecfb1e3d8d0927cd612413")


