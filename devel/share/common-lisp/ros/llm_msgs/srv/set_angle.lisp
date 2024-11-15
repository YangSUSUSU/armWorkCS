; Auto-generated. Do not edit!


(cl:in-package llm_msgs-srv)


;//! \htmlinclude set_angle-request.msg.html

(cl:defclass <set_angle-request> (roslisp-msg-protocol:ros-message)
  ((angle0Ratio
    :reader angle0Ratio
    :initarg :angle0Ratio
    :type cl:float
    :initform 0.0)
   (angle1Ratio
    :reader angle1Ratio
    :initarg :angle1Ratio
    :type cl:float
    :initform 0.0)
   (angle2Ratio
    :reader angle2Ratio
    :initarg :angle2Ratio
    :type cl:float
    :initform 0.0)
   (angle3Ratio
    :reader angle3Ratio
    :initarg :angle3Ratio
    :type cl:float
    :initform 0.0)
   (angle4Ratio
    :reader angle4Ratio
    :initarg :angle4Ratio
    :type cl:float
    :initform 0.0)
   (angle5Ratio
    :reader angle5Ratio
    :initarg :angle5Ratio
    :type cl:float
    :initform 0.0))
)

(cl:defclass set_angle-request (<set_angle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_angle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_angle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_angle-request> is deprecated: use llm_msgs-srv:set_angle-request instead.")))

(cl:ensure-generic-function 'angle0Ratio-val :lambda-list '(m))
(cl:defmethod angle0Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle0Ratio-val is deprecated.  Use llm_msgs-srv:angle0Ratio instead.")
  (angle0Ratio m))

(cl:ensure-generic-function 'angle1Ratio-val :lambda-list '(m))
(cl:defmethod angle1Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle1Ratio-val is deprecated.  Use llm_msgs-srv:angle1Ratio instead.")
  (angle1Ratio m))

(cl:ensure-generic-function 'angle2Ratio-val :lambda-list '(m))
(cl:defmethod angle2Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle2Ratio-val is deprecated.  Use llm_msgs-srv:angle2Ratio instead.")
  (angle2Ratio m))

(cl:ensure-generic-function 'angle3Ratio-val :lambda-list '(m))
(cl:defmethod angle3Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle3Ratio-val is deprecated.  Use llm_msgs-srv:angle3Ratio instead.")
  (angle3Ratio m))

(cl:ensure-generic-function 'angle4Ratio-val :lambda-list '(m))
(cl:defmethod angle4Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle4Ratio-val is deprecated.  Use llm_msgs-srv:angle4Ratio instead.")
  (angle4Ratio m))

(cl:ensure-generic-function 'angle5Ratio-val :lambda-list '(m))
(cl:defmethod angle5Ratio-val ((m <set_angle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle5Ratio-val is deprecated.  Use llm_msgs-srv:angle5Ratio instead.")
  (angle5Ratio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_angle-request>) ostream)
  "Serializes a message object of type '<set_angle-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle0Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle1Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle2Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle3Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle4Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle5Ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_angle-request>) istream)
  "Deserializes a message object of type '<set_angle-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle0Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle1Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle2Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle3Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle4Ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle5Ratio) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_angle-request>)))
  "Returns string type for a service object of type '<set_angle-request>"
  "llm_msgs/set_angleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_angle-request)))
  "Returns string type for a service object of type 'set_angle-request"
  "llm_msgs/set_angleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_angle-request>)))
  "Returns md5sum for a message object of type '<set_angle-request>"
  "3f54cd874965b27feb654abae7abbdcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_angle-request)))
  "Returns md5sum for a message object of type 'set_angle-request"
  "3f54cd874965b27feb654abae7abbdcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_angle-request>)))
  "Returns full string definition for message of type '<set_angle-request>"
  (cl:format cl:nil "float32 angle0Ratio~%float32 angle1Ratio~%float32 angle2Ratio~%float32 angle3Ratio~%float32 angle4Ratio~%float32 angle5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_angle-request)))
  "Returns full string definition for message of type 'set_angle-request"
  (cl:format cl:nil "float32 angle0Ratio~%float32 angle1Ratio~%float32 angle2Ratio~%float32 angle3Ratio~%float32 angle4Ratio~%float32 angle5Ratio~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_angle-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_angle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'set_angle-request
    (cl:cons ':angle0Ratio (angle0Ratio msg))
    (cl:cons ':angle1Ratio (angle1Ratio msg))
    (cl:cons ':angle2Ratio (angle2Ratio msg))
    (cl:cons ':angle3Ratio (angle3Ratio msg))
    (cl:cons ':angle4Ratio (angle4Ratio msg))
    (cl:cons ':angle5Ratio (angle5Ratio msg))
))
;//! \htmlinclude set_angle-response.msg.html

(cl:defclass <set_angle-response> (roslisp-msg-protocol:ros-message)
  ((angle_accepted
    :reader angle_accepted
    :initarg :angle_accepted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass set_angle-response (<set_angle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_angle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_angle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<set_angle-response> is deprecated: use llm_msgs-srv:set_angle-response instead.")))

(cl:ensure-generic-function 'angle_accepted-val :lambda-list '(m))
(cl:defmethod angle_accepted-val ((m <set_angle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:angle_accepted-val is deprecated.  Use llm_msgs-srv:angle_accepted instead.")
  (angle_accepted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_angle-response>) ostream)
  "Serializes a message object of type '<set_angle-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'angle_accepted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_angle-response>) istream)
  "Deserializes a message object of type '<set_angle-response>"
    (cl:setf (cl:slot-value msg 'angle_accepted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_angle-response>)))
  "Returns string type for a service object of type '<set_angle-response>"
  "llm_msgs/set_angleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_angle-response)))
  "Returns string type for a service object of type 'set_angle-response"
  "llm_msgs/set_angleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_angle-response>)))
  "Returns md5sum for a message object of type '<set_angle-response>"
  "3f54cd874965b27feb654abae7abbdcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_angle-response)))
  "Returns md5sum for a message object of type 'set_angle-response"
  "3f54cd874965b27feb654abae7abbdcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_angle-response>)))
  "Returns full string definition for message of type '<set_angle-response>"
  (cl:format cl:nil "bool angle_accepted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_angle-response)))
  "Returns full string definition for message of type 'set_angle-response"
  (cl:format cl:nil "bool angle_accepted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_angle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_angle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'set_angle-response
    (cl:cons ':angle_accepted (angle_accepted msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'set_angle)))
  'set_angle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'set_angle)))
  'set_angle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_angle)))
  "Returns string type for a service object of type '<set_angle>"
  "llm_msgs/set_angle")