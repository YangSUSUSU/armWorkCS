;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::get_status)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'get_status (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::GET_STATUS")
  (make-package "LLM_MSGS::GET_STATUS"))
(unless (find-package "LLM_MSGS::GET_STATUSREQUEST")
  (make-package "LLM_MSGS::GET_STATUSREQUEST"))
(unless (find-package "LLM_MSGS::GET_STATUSRESPONSE")
  (make-package "LLM_MSGS::GET_STATUSRESPONSE"))

(in-package "ROS")





(defclass llm_msgs::get_statusRequest
  :super ros::object
  :slots ())

(defmethod llm_msgs::get_statusRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass llm_msgs::get_statusResponse
  :super ros::object
  :slots (_statusvalue ))

(defmethod llm_msgs::get_statusResponse
  (:init
   (&key
    ((:statusvalue __statusvalue) (make-array 6 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _statusvalue __statusvalue)
   self)
  (:statusvalue
   (&optional __statusvalue)
   (if __statusvalue (setq _statusvalue __statusvalue)) _statusvalue)
  (:serialization-length
   ()
   (+
    ;; uint32[6] _statusvalue
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[6] _statusvalue
     (dotimes (i 6)
       (write-long (elt _statusvalue i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[6] _statusvalue
   (dotimes (i (length _statusvalue))
     (setf (elt _statusvalue i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass llm_msgs::get_status
  :super ros::object
  :slots ())

(setf (get llm_msgs::get_status :md5sum-) "7f7cdabc561edd46a5ab52db3c6d6e89")
(setf (get llm_msgs::get_status :datatype-) "llm_msgs/get_status")
(setf (get llm_msgs::get_status :request) llm_msgs::get_statusRequest)
(setf (get llm_msgs::get_status :response) llm_msgs::get_statusResponse)

(defmethod llm_msgs::get_statusRequest
  (:response () (instance llm_msgs::get_statusResponse :init)))

(setf (get llm_msgs::get_statusRequest :md5sum-) "7f7cdabc561edd46a5ab52db3c6d6e89")
(setf (get llm_msgs::get_statusRequest :datatype-) "llm_msgs/get_statusRequest")
(setf (get llm_msgs::get_statusRequest :definition-)
      "---
uint32[6] statusvalue
")

(setf (get llm_msgs::get_statusResponse :md5sum-) "7f7cdabc561edd46a5ab52db3c6d6e89")
(setf (get llm_msgs::get_statusResponse :datatype-) "llm_msgs/get_statusResponse")
(setf (get llm_msgs::get_statusResponse :definition-)
      "---
uint32[6] statusvalue
")



(provide :llm_msgs/get_status "7f7cdabc561edd46a5ab52db3c6d6e89")


