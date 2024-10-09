; Auto-generated. Do not edit!


(cl:in-package llm_msgs-srv)


;//! \htmlinclude set_speed-request.msg.html

(cl:defclass <set_speed-request> (roslisp-msg-protocol:ros-message)
  ((speed0Ratio
    :reader speed0Ratio
    :initarg :speed0Ratio
    :type cl:float
    :initform 0.0)
   (speed1Ratio
    :reader speed1Ratio
    :initarg :speed1Ratio
    :type cl:float
    :initform 0.0)
   (speed2Ratio
    :reader speed2Ratio
    :initarg :speed2Ratio
    :type cl:float
    :initform 0.0)
   (speed3Ratio
    :reader speed3Ratio
    :initarg :speed3Ratio
    :type cl:float
    :initform 0.0)
   (speed4Ratio
    :reader speed4Ratio
    :initarg :speed4Ratio
    :type cl:float
    :initform 0.0)
   (speed5Ratio
    :reader speed5Ratio
    :initarg :speed5Ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass set_speed-request (<set_speed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_speed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_speed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_speed-request> is deprecated: use llm_msgs-srv:set_speed-request instead.")))

(cl:ensure-generic-function 'speed0Ratio-val :lambda-list '(m))
(cl:defmethod speed0Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed0Ratio-val is deprecated.  Use llm_msgs-srv:speed0Ratio instead.")
  (speed0Ratio m))

(cl:ensure-generic-function 'speed1Ratio-val :lambda-list '(m))
(cl:defmethod speed1Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed1Ratio-val is deprecated.  Use llm_msgs-srv:speed1Ratio instead.")
  (speed1Ratio m))

(cl:ensure-generic-function 'speed2Ratio-val :lambda-list '(m))
(cl:defmethod speed2Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed2Ratio-val is deprecated.  Use llm_msgs-srv:speed2Ratio instead.")
  (speed2Ratio m))

(cl:ensure-generic-function 'speed3Ratio-val :lambda-list '(m))
(cl:defmethod speed3Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed3Ratio-val is deprecated.  Use llm_msgs-srv:speed3Ratio instead.")
  (speed3Ratio m))

(cl:ensure-generic-function 'speed4Ratio-val :lambda-list '(m))
(cl:defmethod speed4Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed4Ratio-val is deprecated.  Use llm_msgs-srv:speed4Ratio instead.")
  (speed4Ratio m))

(cl:ensure-generic-function 'speed5Ratio-val :lambda-list '(m))
(cl:defmethod speed5Ratio-val ((m <set_speed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed5Ratio-val is deprecated.  Use llm_msgs-srv:speed5Ratio instead.")
  (speed5Ratio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_speed-request>) ostream)
  "Serializes a message object of type '<set_speed-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed0Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed1Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed2Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed3Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed4Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed5Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_speed-request>) istream)
  "Deserializes a message object of type '<set_speed-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed0Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed1Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed2Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed3Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed4Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed5Ratio) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_speed-request>)))
  "Returns string type for a service object of type '<set_speed-request>"
  "llm_msgs/set_speedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_speed-request)))
  "Returns string type for a service object of type 'set_speed-request"
  "llm_msgs/set_speedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_speed-request>)))
  "Returns md5sum for a message object of type '<set_speed-request>"
  "4d1d00d67ce0ba0765e4ad67b563d391")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_speed-request)))
  "Returns md5sum for a message object of type 'set_speed-request"
  "4d1d00d67ce0ba0765e4ad67b563d391")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_speed-request>)))
  "Returns full string definition for message of type '<set_speed-request>"
  (cl:format cl:nil "float32 speed0Ratio~%float32 speed1Ratio~%float32 speed2Ratio~%float32 speed3Ratio~%float32 speed4Ratio~%float32 speed5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_speed-request)))
  "Returns full string definition for message of type 'set_speed-request"
  (cl:format cl:nil "float32 speed0Ratio~%float32 speed1Ratio~%float32 speed2Ratio~%float32 speed3Ratio~%float32 speed4Ratio~%float32 speed5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_speed-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_speed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_speed-request
    (cl:cons ':speed0Ratio (speed0Ratio msg))
    (cl:cons ':speed1Ratio (speed1Ratio msg))
    (cl:cons ':speed2Ratio (speed2Ratio msg))
    (cl:cons ':speed3Ratio (speed3Ratio msg))
    (cl:cons ':speed4Ratio (speed4Ratio msg))
    (cl:cons ':speed5Ratio (speed5Ratio msg))
))
;//! \htmlinclude set_speed-response.msg.html

(cl:defclass <set_speed-response> (roslisp-msg-protocol:ros-message)
  ((speed_accepted
    :reader speed_accepted
    :initarg :speed_accepted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_speed-response (<set_speed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_speed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_speed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_speed-response> is deprecated: use llm_msgs-srv:set_speed-response instead.")))

(cl:ensure-generic-function 'speed_accepted-val :lambda-list '(m))
(cl:defmethod speed_accepted-val ((m <set_speed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:speed_accepted-val is deprecated.  Use llm_msgs-srv:speed_accepted instead.")
  (speed_accepted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_speed-response>) ostream)
  "Serializes a message object of type '<set_speed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'speed_accepted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_speed-response>) istream)
  "Deserializes a message object of type '<set_speed-response>"
    (cl:setf (cl:slot-value msg 'speed_accepted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_speed-response>)))
  "Returns string type for a service object of type '<set_speed-response>"
  "llm_msgs/set_speedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_speed-response)))
  "Returns string type for a service object of type 'set_speed-response"
  "llm_msgs/set_speedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_speed-response>)))
  "Returns md5sum for a message object of type '<set_speed-response>"
  "4d1d00d67ce0ba0765e4ad67b563d391")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_speed-response)))
  "Returns md5sum for a message object of type 'set_speed-response"
  "4d1d00d67ce0ba0765e4ad67b563d391")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_speed-response>)))
  "Returns full string definition for message of type '<set_speed-response>"
  (cl:format cl:nil "bool speed_accepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_speed-response)))
  "Returns full string definition for message of type 'set_speed-response"
  (cl:format cl:nil "bool speed_accepted~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_speed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_speed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_speed-response
    (cl:cons ':speed_accepted (speed_accepted msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_speed)))
  'set_speed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_speed)))
  'set_speed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_speed)))
  "Returns string type for a service object of type '<set_speed>"
  "llm_msgs/set_speed")