;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::robot_state)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'robot_state (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::ROBOT_STATE")
  (make-package "LLM_MSGS::ROBOT_STATE"))

(in-package "ROS")
;;//! \htmlinclude robot_state.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass llm_msgs::robot_state
  :super ros::object
  :slots (_header _head_pose ))

(defmethod llm_msgs::robot_state
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:head_pose __head_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _head_pose __head_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:head_pose
   (&optional __head_pose)
   (if __head_pose (setq _head_pose __head_pose)) _head_pose)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _head_pose
    (* 8    (length _head_pose)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _head_pose
     (write-long (length _head_pose) s)
     (dotimes (i (length _head_pose))
       (sys::poke (elt _head_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _head_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _head_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _head_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get llm_msgs::robot_state :md5sum-) "138e1f0b512da903c8d31155a12f0ec5")
(setf (get llm_msgs::robot_state :datatype-) "llm_msgs/robot_state")
(setf (get llm_msgs::robot_state :definition-)
      "std_msgs/Header header

# 头部姿态数据
float64[] head_pose #X, Y, Z, R, P, Y

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



(provide :llm_msgs/robot_state "138e1f0b512da903c8d31155a12f0ec5")


