;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::set_speed)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'set_speed (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::SET_SPEED")
  (make-package "LLM_MSGS::SET_SPEED"))
(unless (find-package "LLM_MSGS::SET_SPEEDREQUEST")
  (make-package "LLM_MSGS::SET_SPEEDREQUEST"))
(unless (find-package "LLM_MSGS::SET_SPEEDRESPONSE")
  (make-package "LLM_MSGS::SET_SPEEDRESPONSE"))

(in-package "ROS")





(defclass llm_msgs::set_speedRequest
  :super ros::object
  :slots (_speed0Ratio _speed1Ratio _speed2Ratio _speed3Ratio _speed4Ratio _speed5Ratio ))

(defmethod llm_msgs::set_speedRequest
  (:init
   (&key
    ((:speed0Ratio __speed0Ratio) 0.0)
    ((:speed1Ratio __speed1Ratio) 0.0)
    ((:speed2Ratio __speed2Ratio) 0.0)
    ((:speed3Ratio __speed3Ratio) 0.0)
    ((:speed4Ratio __speed4Ratio) 0.0)
    ((:speed5Ratio __speed5Ratio) 0.0)
    )
   (send-super :init)
   (setq _speed0Ratio (float __speed0Ratio))
   (setq _speed1Ratio (float __speed1Ratio))
   (setq _speed2Ratio (float __speed2Ratio))
   (setq _speed3Ratio (float __speed3Ratio))
   (setq _speed4Ratio (float __speed4Ratio))
   (setq _speed5Ratio (float __speed5Ratio))
   self)
  (:speed0Ratio
   (&optional __speed0Ratio)
   (if __speed0Ratio (setq _speed0Ratio __speed0Ratio)) _speed0Ratio)
  (:speed1Ratio
   (&optional __speed1Ratio)
   (if __speed1Ratio (setq _speed1Ratio __speed1Ratio)) _speed1Ratio)
  (:speed2Ratio
   (&optional __speed2Ratio)
   (if __speed2Ratio (setq _speed2Ratio __speed2Ratio)) _speed2Ratio)
  (:speed3Ratio
   (&optional __speed3Ratio)
   (if __speed3Ratio (setq _speed3Ratio __speed3Ratio)) _speed3Ratio)
  (:speed4Ratio
   (&optional __speed4Ratio)
   (if __speed4Ratio (setq _speed4Ratio __speed4Ratio)) _speed4Ratio)
  (:speed5Ratio
   (&optional __speed5Ratio)
   (if __speed5Ratio (setq _speed5Ratio __speed5Ratio)) _speed5Ratio)
  (:serialization-length
   ()
   (+
    ;; float32 _speed0Ratio
    4
    ;; float32 _speed1Ratio
    4
    ;; float32 _speed2Ratio
    4
    ;; float32 _speed3Ratio
    4
    ;; float32 _speed4Ratio
    4
    ;; float32 _speed5Ratio
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _speed0Ratio
       (sys::poke _speed0Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed1Ratio
       (sys::poke _speed1Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed2Ratio
       (sys::poke _speed2Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed3Ratio
       (sys::poke _speed3Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed4Ratio
       (sys::poke _speed4Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed5Ratio
       (sys::poke _speed5Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _speed0Ratio
     (setq _speed0Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed1Ratio
     (setq _speed1Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed2Ratio
     (setq _speed2Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed3Ratio
     (setq _speed3Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed4Ratio
     (setq _speed4Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed5Ratio
     (setq _speed5Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass llm_msgs::set_speedResponse
  :super ros::object
  :slots (_speed_accepted ))

(defmethod llm_msgs::set_speedResponse
  (:init
   (&key
    ((:speed_accepted __speed_accepted) nil)
    )
   (send-super :init)
   (setq _speed_accepted __speed_accepted)
   self)
  (:speed_accepted
   (&optional (__speed_accepted :null))
   (if (not (eq __speed_accepted :null)) (setq _speed_accepted __speed_accepted)) _speed_accepted)
  (:serialization-length
   ()
   (+
    ;; bool _speed_accepted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _speed_accepted
       (if _speed_accepted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _speed_accepted
     (setq _speed_accepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass llm_msgs::set_speed
  :super ros::object
  :slots ())

(setf (get llm_msgs::set_speed :md5sum-) "4d1d00d67ce0ba0765e4ad67b563d391")
(setf (get llm_msgs::set_speed :datatype-) "llm_msgs/set_speed")
(setf (get llm_msgs::set_speed :request) llm_msgs::set_speedRequest)
(setf (get llm_msgs::set_speed :response) llm_msgs::set_speedResponse)

(defmethod llm_msgs::set_speedRequest
  (:response () (instance llm_msgs::set_speedResponse :init)))

(setf (get llm_msgs::set_speedRequest :md5sum-) "4d1d00d67ce0ba0765e4ad67b563d391")
(setf (get llm_msgs::set_speedRequest :datatype-) "llm_msgs/set_speedRequest")
(setf (get llm_msgs::set_speedRequest :definition-)
      "float32 speed0Ratio
float32 speed1Ratio
float32 speed2Ratio
float32 speed3Ratio
float32 speed4Ratio
float32 speed5Ratio
---
bool speed_accepted

")

(setf (get llm_msgs::set_speedResponse :md5sum-) "4d1d00d67ce0ba0765e4ad67b563d391")
(setf (get llm_msgs::set_speedResponse :datatype-) "llm_msgs/set_speedResponse")
(setf (get llm_msgs::set_speedResponse :definition-)
      "float32 speed0Ratio
float32 speed1Ratio
float32 speed2Ratio
float32 speed3Ratio
float32 speed4Ratio
float32 speed5Ratio
---
bool speed_accepted

")



(provide :llm_msgs/set_speed "4d1d00d67ce0ba0765e4ad67b563d391")


