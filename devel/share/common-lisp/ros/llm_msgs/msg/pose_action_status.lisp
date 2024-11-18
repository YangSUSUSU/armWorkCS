; Auto-generated. Do not edit!


(cl:in-package llm_msgs-msg)


;//! \htmlinclude pose_action_status.msg.html

(cl:defclass <pose_action_status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (resp_frame_id
    :reader resp_frame_id
    :initarg :resp_frame_id
    :type cl:integer
    :initform 0)
   (hand_move_success
    :reader hand_move_success
    :initarg :hand_move_success
    :type cl:fixnum
    :initform 0)
   (head_move_success
    :reader head_move_success
    :initarg :head_move_success
    :type cl:fixnum
    :initform 0)
   (gripper_move_success
    :reader gripper_move_success
    :initarg :gripper_move_success
    :type cl:fixnum
    :initform 0)
   (waist_move_success
    :reader waist_move_success
    :initarg :waist_move_success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass pose_action_status (<pose_action_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose_action_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose_action_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-msg:<pose_action_status> is deprecated: use llm_msgs-msg:pose_action_status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:header-val is deprecated.  Use llm_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'resp_frame_id-val :lambda-list '(m))
(cl:defmethod resp_frame_id-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:resp_frame_id-val is deprecated.  Use llm_msgs-msg:resp_frame_id instead.")
  (resp_frame_id m))

(cl:ensure-generic-function 'hand_move_success-val :lambda-list '(m))
(cl:defmethod hand_move_success-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:hand_move_success-val is deprecated.  Use llm_msgs-msg:hand_move_success instead.")
  (hand_move_success m))

(cl:ensure-generic-function 'head_move_success-val :lambda-list '(m))
(cl:defmethod head_move_success-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:head_move_success-val is deprecated.  Use llm_msgs-msg:head_move_success instead.")
  (head_move_success m))

(cl:ensure-generic-function 'gripper_move_success-val :lambda-list '(m))
(cl:defmethod gripper_move_success-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:gripper_move_success-val is deprecated.  Use llm_msgs-msg:gripper_move_success instead.")
  (gripper_move_success m))

(cl:ensure-generic-function 'waist_move_success-val :lambda-list '(m))
(cl:defmethod waist_move_success-val ((m <pose_action_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:waist_move_success-val is deprecated.  Use llm_msgs-msg:waist_move_success instead.")
  (waist_move_success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose_action_status>) ostream)
  "Serializes a message object of type '<pose_action_status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resp_frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resp_frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resp_frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resp_frame_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_move_success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'head_move_success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_move_success)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waist_move_success)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose_action_status>) istream)
  "Deserializes a message object of type '<pose_action_status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'resp_frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'resp_frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'resp_frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'resp_frame_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_move_success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'head_move_success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_move_success)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waist_move_success)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose_action_status>)))
  "Returns string type for a message object of type '<pose_action_status>"
  "llm_msgs/pose_action_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose_action_status)))
  "Returns string type for a message object of type 'pose_action_status"
  "llm_msgs/pose_action_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose_action_status>)))
  "Returns md5sum for a message object of type '<pose_action_status>"
  "d102c2efc0ecfb1e3d8d0927cd612413")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose_action_status)))
  "Returns md5sum for a message object of type 'pose_action_status"
  "d102c2efc0ecfb1e3d8d0927cd612413")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose_action_status>)))
  "Returns full string definition for message of type '<pose_action_status>"
  (cl:format cl:nil "std_msgs/Header header~%# 相应的hand_pose_req msg中header的seq,表示是对该帧pose的相应~%uint32 resp_frame_id     ~%# 0-失败， 1-成功， 2-默认值(default），其他数值暂不可用，留待后续扩展   ~%uint8 hand_move_success      ~%uint8 head_move_success     ~%uint8 gripper_move_success ~%uint8 waist_move_success ~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose_action_status)))
  "Returns full string definition for message of type 'pose_action_status"
  (cl:format cl:nil "std_msgs/Header header~%# 相应的hand_pose_req msg中header的seq,表示是对该帧pose的相应~%uint32 resp_frame_id     ~%# 0-失败， 1-成功， 2-默认值(default），其他数值暂不可用，留待后续扩展   ~%uint8 hand_move_success      ~%uint8 head_move_success     ~%uint8 gripper_move_success ~%uint8 waist_move_success ~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose_action_status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose_action_status>))
  "Converts a ROS message object to a list"
  (cl:list 'pose_action_status
    (cl:cons ':header (header msg))
    (cl:cons ':resp_frame_id (resp_frame_id msg))
    (cl:cons ':hand_move_success (hand_move_success msg))
    (cl:cons ':head_move_success (head_move_success msg))
    (cl:cons ':gripper_move_success (gripper_move_success msg))
    (cl:cons ':waist_move_success (waist_move_success msg))
))
