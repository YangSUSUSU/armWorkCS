; Auto-generated. Do not edit!


(cl:in-package llm_msgs-srv)


;//! \htmlinclude get_status-request.msg.html

(cl:defclass <get_status-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass get_status-request (<get_status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<get_status-request> is deprecated: use llm_msgs-srv:get_status-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_status-request>) ostream)
  "Serializes a message object of type '<get_status-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_status-request>) istream)
  "Deserializes a message object of type '<get_status-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_status-request>)))
  "Returns string type for a service object of type '<get_status-request>"
  "llm_msgs/get_statusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_status-request)))
  "Returns string type for a service object of type 'get_status-request"
  "llm_msgs/get_statusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_status-request>)))
  "Returns md5sum for a message object of type '<get_status-request>"
  "7f7cdabc561edd46a5ab52db3c6d6e89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_status-request)))
  "Returns md5sum for a message object of type 'get_status-request"
  "7f7cdabc561edd46a5ab52db3c6d6e89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_status-request>)))
  "Returns full string definition for message of type '<get_status-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_status-request)))
  "Returns full string definition for message of type 'get_status-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_status-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_status-request
))
;//! \htmlinclude get_status-response.msg.html

(cl:defclass <get_status-response> (roslisp-msg-protocol:ros-message)
  ((statusvalue
    :reader statusvalue
    :initarg :statusvalue
    :type (cl:vector cl:integer)
   :initform (cl:make-array 6 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass get_status-response (<get_status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name llm_msgs-srv:<get_status-response> is deprecated: use llm_msgs-srv:get_status-response instead.")))

(cl:ensure-generic-function 'statusvalue-val :lambda-list '(m))
(cl:defmethod statusvalue-val ((m <get_status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader llm_msgs-srv:statusvalue-val is deprecated.  Use llm_msgs-srv:statusvalue instead.")
  (statusvalue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_status-response>) ostream)
  "Serializes a message object of type '<get_status-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'statusvalue))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_status-response>) istream)
  "Deserializes a message object of type '<get_status-response>"
  (cl:setf (cl:slot-value msg 'statusvalue) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'statusvalue)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_status-response>)))
  "Returns string type for a service object of type '<get_status-response>"
  "llm_msgs/get_statusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_status-response)))
  "Returns string type for a service object of type 'get_status-response"
  "llm_msgs/get_statusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_status-response>)))
  "Returns md5sum for a message object of type '<get_status-response>"
  "7f7cdabc561edd46a5ab52db3c6d6e89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_status-response)))
  "Returns md5sum for a message object of type 'get_status-response"
  "7f7cdabc561edd46a5ab52db3c6d6e89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_status-response>)))
  "Returns full string definition for message of type '<get_status-response>"
  (cl:format cl:nil "uint32[6] statusvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_status-response)))
  "Returns full string definition for message of type 'get_status-response"
  (cl:format cl:nil "uint32[6] statusvalue~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_status-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'statusvalue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_status-response
    (cl:cons ':statusvalue (statusvalue msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_status)))
  'get_status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_status)))
  'get_status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_status)))
  "Returns string type for a service object of type '<get_status>"
  "llm_msgs/get_status")