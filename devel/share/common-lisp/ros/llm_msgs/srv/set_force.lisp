; Auto-generated. Do not edit!


(cl:in-package llm_msgs-srv)


;//! \htmlinclude set_force-request.msg.html

(cl:defclass <set_force-request> (roslisp-msg-protocol:ros-message)
  ((force0Ratio
    :reader force0Ratio
    :initarg :force0Ratio
    :type cl:float
    :initform 0.0)
   (force1Ratio
    :reader force1Ratio
    :initarg :force1Ratio
    :type cl:float
    :initform 0.0)
   (force2Ratio
    :reader force2Ratio
    :initarg :force2Ratio
    :type cl:float
    :initform 0.0)
   (force3Ratio
    :reader force3Ratio
    :initarg :force3Ratio
    :type cl:float
    :initform 0.0)
   (force4Ratio
    :reader force4Ratio
    :initarg :force4Ratio
    :type cl:float
    :initform 0.0)
   (force5Ratio
    :reader force5Ratio
    :initarg :force5Ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass set_force-request (<set_force-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_force-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_force-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_force-request> is deprecated: use llm_msgs-srv:set_force-request instead.")))

(cl:ensure-generic-function 'force0Ratio-val :lambda-list '(m))
(cl:defmethod force0Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force0Ratio-val is deprecated.  Use llm_msgs-srv:force0Ratio instead.")
  (force0Ratio m))

(cl:ensure-generic-function 'force1Ratio-val :lambda-list '(m))
(cl:defmethod force1Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force1Ratio-val is deprecated.  Use llm_msgs-srv:force1Ratio instead.")
  (force1Ratio m))

(cl:ensure-generic-function 'force2Ratio-val :lambda-list '(m))
(cl:defmethod force2Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force2Ratio-val is deprecated.  Use llm_msgs-srv:force2Ratio instead.")
  (force2Ratio m))

(cl:ensure-generic-function 'force3Ratio-val :lambda-list '(m))
(cl:defmethod force3Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force3Ratio-val is deprecated.  Use llm_msgs-srv:force3Ratio instead.")
  (force3Ratio m))

(cl:ensure-generic-function 'force4Ratio-val :lambda-list '(m))
(cl:defmethod force4Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force4Ratio-val is deprecated.  Use llm_msgs-srv:force4Ratio instead.")
  (force4Ratio m))

(cl:ensure-generic-function 'force5Ratio-val :lambda-list '(m))
(cl:defmethod force5Ratio-val ((m <set_force-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force5Ratio-val is deprecated.  Use llm_msgs-srv:force5Ratio instead.")
  (force5Ratio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_force-request>) ostream)
  "Serializes a message object of type '<set_force-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force0Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force1Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force2Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force3Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force4Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force5Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_force-request>) istream)
  "Deserializes a message object of type '<set_force-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force0Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force1Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force2Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force3Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force4Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force5Ratio) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_force-request>)))
  "Returns string type for a service object of type '<set_force-request>"
  "llm_msgs/set_forceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_force-request)))
  "Returns string type for a service object of type 'set_force-request"
  "llm_msgs/set_forceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_force-request>)))
  "Returns md5sum for a message object of type '<set_force-request>"
  "6b0fdeb4ed7ee4c97030abfd78488ebb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_force-request)))
  "Returns md5sum for a message object of type 'set_force-request"
  "6b0fdeb4ed7ee4c97030abfd78488ebb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_force-request>)))
  "Returns full string definition for message of type '<set_force-request>"
  (cl:format cl:nil "float32 force0Ratio~%float32 force1Ratio~%float32 force2Ratio~%float32 force3Ratio~%float32 force4Ratio~%float32 force5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_force-request)))
  "Returns full string definition for message of type 'set_force-request"
  (cl:format cl:nil "float32 force0Ratio~%float32 force1Ratio~%float32 force2Ratio~%float32 force3Ratio~%float32 force4Ratio~%float32 force5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_force-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_force-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_force-request
    (cl:cons ':force0Ratio (force0Ratio msg))
    (cl:cons ':force1Ratio (force1Ratio msg))
    (cl:cons ':force2Ratio (force2Ratio msg))
    (cl:cons ':force3Ratio (force3Ratio msg))
    (cl:cons ':force4Ratio (force4Ratio msg))
    (cl:cons ':force5Ratio (force5Ratio msg))
))
;//! \htmlinclude set_force-response.msg.html

(cl:defclass <set_force-response> (roslisp-msg-protocol:ros-message)
  ((force_accepted
    :reader force_accepted
    :initarg :force_accepted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_force-response (<set_force-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_force-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_force-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_force-response> is deprecated: use llm_msgs-srv:set_force-response instead.")))

(cl:ensure-generic-function 'force_accepted-val :lambda-list '(m))
(cl:defmethod force_accepted-val ((m <set_force-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:force_accepted-val is deprecated.  Use llm_msgs-srv:force_accepted instead.")
  (force_accepted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_force-response>) ostream)
  "Serializes a message object of type '<set_force-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'force_accepted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_force-response>) istream)
  "Deserializes a message object of type '<set_force-response>"
    (cl:setf (cl:slot-value msg 'force_accepted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_force-response>)))
  "Returns string type for a service object of type '<set_force-response>"
  "llm_msgs/set_forceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_force-response)))
  "Returns string type for a service object of type 'set_force-response"
  "llm_msgs/set_forceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_force-response>)))
  "Returns md5sum for a message object of type '<set_force-response>"
  "6b0fdeb4ed7ee4c97030abfd78488ebb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_force-response)))
  "Returns md5sum for a message object of type 'set_force-response"
  "6b0fdeb4ed7ee4c97030abfd78488ebb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_force-response>)))
  "Returns full string definition for message of type '<set_force-response>"
  (cl:format cl:nil "bool force_accepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_force-response)))
  "Returns full string definition for message of type 'set_force-response"
  (cl:format cl:nil "bool force_accepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_force-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_force-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_force-response
    (cl:cons ':force_accepted (force_accepted msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_force)))
  'set_force-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_force)))
  'set_force-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_force)))
  "Returns string type for a service object of type '<set_force>"
  "llm_msgs/set_force")