; Auto-generated. Do not edit!


(cl:in-package llm_msgs-msg)


;//! \htmlinclude hand_pose_req.msg.html

(cl:defclass <hand_pose_req> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (hand_move_enable
    :reader hand_move_enable
    :initarg :hand_move_enable
    :type cl:fixnum
    :initform 0)
   (hand_side
    :reader hand_side
    :initarg :hand_side
    :type cl:fixnum
    :initform 0)
   (hand_reset
    :reader hand_reset
    :initarg :hand_reset
    :type cl:fixnum
    :initform 0)
   (pose_req
    :reader pose_req
    :initarg :pose_req
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (head_move_enable
    :reader head_move_enable
    :initarg :head_move_enable
    :type cl:fixnum
    :initform 0)
   (rpy
    :reader rpy
    :initarg :rpy
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (gripper_move_enable
    :reader gripper_move_enable
    :initarg :gripper_move_enable
    :type cl:fixnum
    :initform 0)
   (gripper_side
    :reader gripper_side
    :initarg :gripper_side
    :type cl:fixnum
    :initform 0)
   (finger_status
    :reader finger_status
    :initarg :finger_status
    :type cl:fixnum
    :initform 0)
   (little_finger
    :reader little_finger
    :initarg :little_finger
    :type cl:float
    :initform 0.0)
   (ring_finger
    :reader ring_finger
    :initarg :ring_finger
    :type cl:float
    :initform 0.0)
   (middle_finger
    :reader middle_finger
    :initarg :middle_finger
    :type cl:float
    :initform 0.0)
   (index_finger
    :reader index_finger
    :initarg :index_finger
    :type cl:float
    :initform 0.0)
   (thumb_bending
    :reader thumb_bending
    :initarg :thumb_bending
    :type cl:float
    :initform 0.0)
   (thumb_rotating
    :reader thumb_rotating
    :initarg :thumb_rotating
    :type cl:float
    :initform 0.0)
   (waist_enable
    :reader waist_enable
    :initarg :waist_enable
    :type cl:fixnum
    :initform 0)
   (waist_rotate_ang
    :reader waist_rotate_ang
    :initarg :waist_rotate_ang
    :type cl:float
    :initform 0.0))
)

(cl:defclass hand_pose_req (<hand_pose_req>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_pose_req>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_pose_req)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-msg:<hand_pose_req> is deprecated: use llm_msgs-msg:hand_pose_req instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:header-val is deprecated.  Use llm_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'hand_move_enable-val :lambda-list '(m))
(cl:defmethod hand_move_enable-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:hand_move_enable-val is deprecated.  Use llm_msgs-msg:hand_move_enable instead.")
  (hand_move_enable m))

(cl:ensure-generic-function 'hand_side-val :lambda-list '(m))
(cl:defmethod hand_side-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:hand_side-val is deprecated.  Use llm_msgs-msg:hand_side instead.")
  (hand_side m))

(cl:ensure-generic-function 'hand_reset-val :lambda-list '(m))
(cl:defmethod hand_reset-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:hand_reset-val is deprecated.  Use llm_msgs-msg:hand_reset instead.")
  (hand_reset m))

(cl:ensure-generic-function 'pose_req-val :lambda-list '(m))
(cl:defmethod pose_req-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:pose_req-val is deprecated.  Use llm_msgs-msg:pose_req instead.")
  (pose_req m))

(cl:ensure-generic-function 'head_move_enable-val :lambda-list '(m))
(cl:defmethod head_move_enable-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:head_move_enable-val is deprecated.  Use llm_msgs-msg:head_move_enable instead.")
  (head_move_enable m))

(cl:ensure-generic-function 'rpy-val :lambda-list '(m))
(cl:defmethod rpy-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:rpy-val is deprecated.  Use llm_msgs-msg:rpy instead.")
  (rpy m))

(cl:ensure-generic-function 'gripper_move_enable-val :lambda-list '(m))
(cl:defmethod gripper_move_enable-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:gripper_move_enable-val is deprecated.  Use llm_msgs-msg:gripper_move_enable instead.")
  (gripper_move_enable m))

(cl:ensure-generic-function 'gripper_side-val :lambda-list '(m))
(cl:defmethod gripper_side-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:gripper_side-val is deprecated.  Use llm_msgs-msg:gripper_side instead.")
  (gripper_side m))

(cl:ensure-generic-function 'finger_status-val :lambda-list '(m))
(cl:defmethod finger_status-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:finger_status-val is deprecated.  Use llm_msgs-msg:finger_status instead.")
  (finger_status m))

(cl:ensure-generic-function 'little_finger-val :lambda-list '(m))
(cl:defmethod little_finger-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:little_finger-val is deprecated.  Use llm_msgs-msg:little_finger instead.")
  (little_finger m))

(cl:ensure-generic-function 'ring_finger-val :lambda-list '(m))
(cl:defmethod ring_finger-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:ring_finger-val is deprecated.  Use llm_msgs-msg:ring_finger instead.")
  (ring_finger m))

(cl:ensure-generic-function 'middle_finger-val :lambda-list '(m))
(cl:defmethod middle_finger-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:middle_finger-val is deprecated.  Use llm_msgs-msg:middle_finger instead.")
  (middle_finger m))

(cl:ensure-generic-function 'index_finger-val :lambda-list '(m))
(cl:defmethod index_finger-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:index_finger-val is deprecated.  Use llm_msgs-msg:index_finger instead.")
  (index_finger m))

(cl:ensure-generic-function 'thumb_bending-val :lambda-list '(m))
(cl:defmethod thumb_bending-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:thumb_bending-val is deprecated.  Use llm_msgs-msg:thumb_bending instead.")
  (thumb_bending m))

(cl:ensure-generic-function 'thumb_rotating-val :lambda-list '(m))
(cl:defmethod thumb_rotating-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:thumb_rotating-val is deprecated.  Use llm_msgs-msg:thumb_rotating instead.")
  (thumb_rotating m))

(cl:ensure-generic-function 'waist_enable-val :lambda-list '(m))
(cl:defmethod waist_enable-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:waist_enable-val is deprecated.  Use llm_msgs-msg:waist_enable instead.")
  (waist_enable m))

(cl:ensure-generic-function 'waist_rotate_ang-val :lambda-list '(m))
(cl:defmethod waist_rotate_ang-val ((m <hand_pose_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-msg:waist_rotate_ang-val is deprecated.  Use llm_msgs-msg:waist_rotate_ang instead.")
  (waist_rotate_ang m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_pose_req>) ostream)
  "Serializes a message object of type '<hand_pose_req>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_move_enable)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_side)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_reset)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_req) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'head_move_enable)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rpy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'rpy))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_move_enable)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_side)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'little_finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ring_finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'middle_finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'index_finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thumb_bending))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thumb_rotating))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waist_enable)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'waist_rotate_ang))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_pose_req>) istream)
  "Deserializes a message object of type '<hand_pose_req>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_move_enable)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_side)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hand_reset)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_req) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'head_move_enable)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rpy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rpy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_move_enable)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gripper_side)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger_status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'little_finger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ring_finger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'middle_finger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'index_finger) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thumb_bending) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thumb_rotating) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waist_enable)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'waist_rotate_ang) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_pose_req>)))
  "Returns string type for a message object of type '<hand_pose_req>"
  "llm_msgs/hand_pose_req")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_pose_req)))
  "Returns string type for a message object of type 'hand_pose_req"
  "llm_msgs/hand_pose_req")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_pose_req>)))
  "Returns md5sum for a message object of type '<hand_pose_req>"
  "b6b3b996b8f56c6f6190cb1b04d5143a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_pose_req)))
  "Returns md5sum for a message object of type 'hand_pose_req"
  "b6b3b996b8f56c6f6190cb1b04d5143a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_pose_req>)))
  "Returns full string definition for message of type '<hand_pose_req>"
  (cl:format cl:nil "std_msgs/Header header     ~%~%# 以下分别为手臂、头部、手指对应的控制数据，相应数据仅当对应使能位有效时才有意义~%# 例如: 当hand_move_enable为1(enable状态)时, pose_req的数据才需要被执行，才有意义~%~%# 手臂移动使能位-hand_move_enable, 手臂目标位姿-pose_req, 手臂复位-hand_reset(0-非复位位姿，1-复位位姿)~%uint8 hand_move_enable    ~%uint8 hand_side ~%uint8 hand_reset          ~%geometry_msgs/Pose pose_req ~%~%# 头部移动使能位-move_enable, 头部目标位姿-(R, P, Y）~%uint8 head_move_enable    ~%float64[] rpy~%~%# 手部手指移动使能位-gripper_move_enable, 手指开度-[figure_1, figure_2, figure_3, figure_4, figure_5]~%uint8 gripper_move_enable ~%uint8 gripper_side  ~%uint8 finger_status~%float32 little_finger~%float32 ring_finger~%float32 middle_finger~%float32 index_finger~%float32 thumb_bending~%float32 thumb_rotating~%~%# 腰部转动~%uint8 waist_enable # 0-disable, 1-enable, 2-default, 其他数值暂不可用，留待后续~%float32 waist_rotate_ang # 腰部旋转角度-rad~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_pose_req)))
  "Returns full string definition for message of type 'hand_pose_req"
  (cl:format cl:nil "std_msgs/Header header     ~%~%# 以下分别为手臂、头部、手指对应的控制数据，相应数据仅当对应使能位有效时才有意义~%# 例如: 当hand_move_enable为1(enable状态)时, pose_req的数据才需要被执行，才有意义~%~%# 手臂移动使能位-hand_move_enable, 手臂目标位姿-pose_req, 手臂复位-hand_reset(0-非复位位姿，1-复位位姿)~%uint8 hand_move_enable    ~%uint8 hand_side ~%uint8 hand_reset          ~%geometry_msgs/Pose pose_req ~%~%# 头部移动使能位-move_enable, 头部目标位姿-(R, P, Y）~%uint8 head_move_enable    ~%float64[] rpy~%~%# 手部手指移动使能位-gripper_move_enable, 手指开度-[figure_1, figure_2, figure_3, figure_4, figure_5]~%uint8 gripper_move_enable ~%uint8 gripper_side  ~%uint8 finger_status~%float32 little_finger~%float32 ring_finger~%float32 middle_finger~%float32 index_finger~%float32 thumb_bending~%float32 thumb_rotating~%~%# 腰部转动~%uint8 waist_enable # 0-disable, 1-enable, 2-default, 其他数值暂不可用，留待后续~%float32 waist_rotate_ang # 腰部旋转角度-rad~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_pose_req>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_req))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rpy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     1
     4
     4
     4
     4
     4
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_pose_req>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_pose_req
    (cl:cons ':header (header msg))
    (cl:cons ':hand_move_enable (hand_move_enable msg))
    (cl:cons ':hand_side (hand_side msg))
    (cl:cons ':hand_reset (hand_reset msg))
    (cl:cons ':pose_req (pose_req msg))
    (cl:cons ':head_move_enable (head_move_enable msg))
    (cl:cons ':rpy (rpy msg))
    (cl:cons ':gripper_move_enable (gripper_move_enable msg))
    (cl:cons ':gripper_side (gripper_side msg))
    (cl:cons ':finger_status (finger_status msg))
    (cl:cons ':little_finger (little_finger msg))
    (cl:cons ':ring_finger (ring_finger msg))
    (cl:cons ':middle_finger (middle_finger msg))
    (cl:cons ':index_finger (index_finger msg))
    (cl:cons ':thumb_bending (thumb_bending msg))
    (cl:cons ':thumb_rotating (thumb_rotating msg))
    (cl:cons ':waist_enable (waist_enable msg))
    (cl:cons ':waist_rotate_ang (waist_rotate_ang msg))
))
