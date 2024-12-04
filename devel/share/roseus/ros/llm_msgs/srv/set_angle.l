;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::set_angle)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'set_angle (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::SET_ANGLE")
  (make-package "LLM_MSGS::SET_ANGLE"))
(unless (find-package "LLM_MSGS::SET_ANGLEREQUEST")
  (make-package "LLM_MSGS::SET_ANGLEREQUEST"))
(unless (find-package "LLM_MSGS::SET_ANGLERESPONSE")
  (make-package "LLM_MSGS::SET_ANGLERESPONSE"))

(in-package "ROS")





(defclass llm_msgs::set_angleRequest
  :super ros::object
  :slots (_angle0Ratio _angle1Ratio _angle2Ratio _angle3Ratio _angle4Ratio _angle5Ratio ))

(defmethod llm_msgs::set_angleRequest
  (:init
   (&key
    ((:angle0Ratio __angle0Ratio) 0.0)
    ((:angle1Ratio __angle1Ratio) 0.0)
    ((:angle2Ratio __angle2Ratio) 0.0)
    ((:angle3Ratio __angle3Ratio) 0.0)
    ((:angle4Ratio __angle4Ratio) 0.0)
    ((:angle5Ratio __angle5Ratio) 0.0)
    )
   (send-super :init)
   (setq _angle0Ratio (float __angle0Ratio))
   (setq _angle1Ratio (float __angle1Ratio))
   (setq _angle2Ratio (float __angle2Ratio))
   (setq _angle3Ratio (float __angle3Ratio))
   (setq _angle4Ratio (float __angle4Ratio))
   (setq _angle5Ratio (float __angle5Ratio))
   self)
  (:angle0Ratio
   (&optional __angle0Ratio)
   (if __angle0Ratio (setq _angle0Ratio __angle0Ratio)) _angle0Ratio)
  (:angle1Ratio
   (&optional __angle1Ratio)
   (if __angle1Ratio (setq _angle1Ratio __angle1Ratio)) _angle1Ratio)
  (:angle2Ratio
   (&optional __angle2Ratio)
   (if __angle2Ratio (setq _angle2Ratio __angle2Ratio)) _angle2Ratio)
  (:angle3Ratio
   (&optional __angle3Ratio)
   (if __angle3Ratio (setq _angle3Ratio __angle3Ratio)) _angle3Ratio)
  (:angle4Ratio
   (&optional __angle4Ratio)
   (if __angle4Ratio (setq _angle4Ratio __angle4Ratio)) _angle4Ratio)
  (:angle5Ratio
   (&optional __angle5Ratio)
   (if __angle5Ratio (setq _angle5Ratio __angle5Ratio)) _angle5Ratio)
  (:serialization-length
   ()
   (+
    ;; float32 _angle0Ratio
    4
    ;; float32 _angle1Ratio
    4
    ;; float32 _angle2Ratio
    4
    ;; float32 _angle3Ratio
    4
    ;; float32 _angle4Ratio
    4
    ;; float32 _angle5Ratio
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle0Ratio
       (sys::poke _angle0Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle1Ratio
       (sys::poke _angle1Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle2Ratio
       (sys::poke _angle2Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle3Ratio
       (sys::poke _angle3Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle4Ratio
       (sys::poke _angle4Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle5Ratio
       (sys::poke _angle5Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle0Ratio
     (setq _angle0Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle1Ratio
     (setq _angle1Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle2Ratio
     (setq _angle2Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle3Ratio
     (setq _angle3Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle4Ratio
     (setq _angle4Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle5Ratio
     (setq _angle5Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass llm_msgs::set_angleResponse
  :super ros::object
  :slots (_angle_accepted ))

(defmethod llm_msgs::set_angleResponse
  (:init
   (&key
    ((:angle_accepted __angle_accepted) nil)
    )
   (send-super :init)
   (setq _angle_accepted __angle_accepted)
   self)
  (:angle_accepted
   (&optional (__angle_accepted :null))
   (if (not (eq __angle_accepted :null)) (setq _angle_accepted __angle_accepted)) _angle_accepted)
  (:serialization-length
   ()
   (+
    ;; bool _angle_accepted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _angle_accepted
       (if _angle_accepted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _angle_accepted
     (setq _angle_accepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass llm_msgs::set_angle
  :super ros::object
  :slots ())

(setf (get llm_msgs::set_angle :md5sum-) "3f54cd874965b27feb654abae7abbdcb")
(setf (get llm_msgs::set_angle :datatype-) "llm_msgs/set_angle")
(setf (get llm_msgs::set_angle :request) llm_msgs::set_angleRequest)
(setf (get llm_msgs::set_angle :response) llm_msgs::set_angleResponse)

(defmethod llm_msgs::set_angleRequest
  (:response () (instance llm_msgs::set_angleResponse :init)))

(setf (get llm_msgs::set_angleRequest :md5sum-) "3f54cd874965b27feb654abae7abbdcb")
(setf (get llm_msgs::set_angleRequest :datatype-) "llm_msgs/set_angleRequest")
(setf (get llm_msgs::set_angleRequest :definition-)
      "float32 angle0Ratio
float32 angle1Ratio
float32 angle2Ratio
float32 angle3Ratio
float32 angle4Ratio
float32 angle5Ratio
---
bool angle_accepted
")

(setf (get llm_msgs::set_angleResponse :md5sum-) "3f54cd874965b27feb654abae7abbdcb")
(setf (get llm_msgs::set_angleResponse :datatype-) "llm_msgs/set_angleResponse")
(setf (get llm_msgs::set_angleResponse :definition-)
      "float32 angle0Ratio
float32 angle1Ratio
float32 angle2Ratio
float32 angle3Ratio
float32 angle4Ratio
float32 angle5Ratio
---
bool angle_accepted
")



(provide :llm_msgs/set_angle "3f54cd874965b27feb654abae7abbdcb")


