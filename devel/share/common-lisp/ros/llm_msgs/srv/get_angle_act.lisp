; Auto-generated. Do not edit!


(cl:in-package llm_msgs-srv)


;//! \htmlinclude get_angle_act-request.msg.html

(cl:defclass <get_angle_act-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass get_angle_act-request (<get_angle_act-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_angle_act-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_angle_act-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<get_angle_act-request> is deprecated: use llm_msgs-srv:get_angle_act-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_angle_act-request>) ostream)
  "Serializes a message object of type '<get_angle_act-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_angle_act-request>) istream)
  "Deserializes a message object of type '<get_angle_act-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_angle_act-request>)))
  "Returns string type for a service object of type '<get_angle_act-request>"
  "llm_msgs/get_angle_actRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_angle_act-request)))
  "Returns string type for a service object of type 'get_angle_act-request"
  "llm_msgs/get_angle_actRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_angle_act-request>)))
  "Returns md5sum for a message object of type '<get_angle_act-request>"
  "5a40d1cc41b6533f906a6a90344676b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_angle_act-request)))
  "Returns md5sum for a message object of type 'get_angle_act-request"
  "5a40d1cc41b6533f906a6a90344676b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_angle_act-request>)))
  "Returns full string definition for message of type '<get_angle_act-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_angle_act-request)))
  "Returns full string definition for message of type 'get_angle_act-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_angle_act-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_angle_act-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_angle_act-request
))
;//! \htmlinclude get_angle_act-response.msg.html

(cl:defclass <get_angle_act-response> (roslisp-msg-protocol:ros-message)
  ((curangleRatio
    :reader curangleRatio
    :initarg :curangleRatio
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass get_angle_act-response (<get_angle_act-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_angle_act-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_angle_act-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<get_angle_act-response> is deprecated: use llm_msgs-srv:get_angle_act-response instead.")))

(cl:ensure-generic-function 'curangleRatio-val :lambda-list '(m))
(cl:defmethod curangleRatio-val ((m <get_angle_act-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:curangleRatio-val is deprecated.  Use llm_msgs-srv:curangleRatio instead.")
  (curangleRatio m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_angle_act-response>) ostream)
  "Serializes a message object of type '<get_angle_act-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'curangleRatio))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_angle_act-response>) istream)
  "Deserializes a message object of type '<get_angle_act-response>"
  (cl:setf (cl:slot-value msg 'curangleRatio) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'curangleRatio)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_angle_act-response>)))
  "Returns string type for a service object of type '<get_angle_act-response>"
  "llm_msgs/get_angle_actResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_angle_act-response)))
  "Returns string type for a service object of type 'get_angle_act-response"
  "llm_msgs/get_angle_actResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_angle_act-response>)))
  "Returns md5sum for a message object of type '<get_angle_act-response>"
  "5a40d1cc41b6533f906a6a90344676b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_angle_act-response)))
  "Returns md5sum for a message object of type 'get_angle_act-response"
  "5a40d1cc41b6533f906a6a90344676b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_angle_act-response>)))
  "Returns full string definition for message of type '<get_angle_act-response>"
  (cl:format cl:nil "float32[6] curangleRatio~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_angle_act-response)))
  "Returns full string definition for message of type 'get_angle_act-response"
  (cl:format cl:nil "float32[6] curangleRatio~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_angle_act-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'curangleRatio) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_angle_act-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_angle_act-response
    (cl:cons ':curangleRatio (curangleRatio msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_angle_act)))
  'get_angle_act-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_angle_act)))
  'get_angle_act-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_angle_act)))
  "Returns string type for a service object of type '<get_angle_act>"
  "llm_msgs/get_angle_act")