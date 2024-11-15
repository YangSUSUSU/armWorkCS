;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::set_force)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'set_force (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::SET_FORCE")
  (make-package "LLM_MSGS::SET_FORCE"))
(unless (find-package "LLM_MSGS::SET_FORCEREQUEST")
  (make-package "LLM_MSGS::SET_FORCEREQUEST"))
(unless (find-package "LLM_MSGS::SET_FORCERESPONSE")
  (make-package "LLM_MSGS::SET_FORCERESPONSE"))

(in-package "ROS")





(defclass llm_msgs::set_forceRequest
  :super ros::object
  :slots (_force0Ratio _force1Ratio _force2Ratio _force3Ratio _force4Ratio _force5Ratio ))

(defmethod llm_msgs::set_forceRequest
  (:init
   (&key
    ((:force0Ratio __force0Ratio) 0.0)
    ((:force1Ratio __force1Ratio) 0.0)
    ((:force2Ratio __force2Ratio) 0.0)
    ((:force3Ratio __force3Ratio) 0.0)
    ((:force4Ratio __force4Ratio) 0.0)
    ((:force5Ratio __force5Ratio) 0.0)
    )
   (send-super :init)
   (setq _force0Ratio (float __force0Ratio))
   (setq _force1Ratio (float __force1Ratio))
   (setq _force2Ratio (float __force2Ratio))
   (setq _force3Ratio (float __force3Ratio))
   (setq _force4Ratio (float __force4Ratio))
   (setq _force5Ratio (float __force5Ratio))
   self)
  (:force0Ratio
   (&optional __force0Ratio)
   (if __force0Ratio (setq _force0Ratio __force0Ratio)) _force0Ratio)
  (:force1Ratio
   (&optional __force1Ratio)
   (if __force1Ratio (setq _force1Ratio __force1Ratio)) _force1Ratio)
  (:force2Ratio
   (&optional __force2Ratio)
   (if __force2Ratio (setq _force2Ratio __force2Ratio)) _force2Ratio)
  (:force3Ratio
   (&optional __force3Ratio)
   (if __force3Ratio (setq _force3Ratio __force3Ratio)) _force3Ratio)
  (:force4Ratio
   (&optional __force4Ratio)
   (if __force4Ratio (setq _force4Ratio __force4Ratio)) _force4Ratio)
  (:force5Ratio
   (&optional __force5Ratio)
   (if __force5Ratio (setq _force5Ratio __force5Ratio)) _force5Ratio)
  (:serialization-length
   ()
   (+
    ;; float32 _force0Ratio
    4
    ;; float32 _force1Ratio
    4
    ;; float32 _force2Ratio
    4
    ;; float32 _force3Ratio
    4
    ;; float32 _force4Ratio
    4
    ;; float32 _force5Ratio
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _force0Ratio
       (sys::poke _force0Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force1Ratio
       (sys::poke _force1Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force2Ratio
       (sys::poke _force2Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force3Ratio
       (sys::poke _force3Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force4Ratio
       (sys::poke _force4Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force5Ratio
       (sys::poke _force5Ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _force0Ratio
     (setq _force0Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force1Ratio
     (setq _force1Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force2Ratio
     (setq _force2Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force3Ratio
     (setq _force3Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force4Ratio
     (setq _force4Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force5Ratio
     (setq _force5Ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass llm_msgs::set_forceResponse
  :super ros::object
  :slots (_force_accepted ))

(defmethod llm_msgs::set_forceResponse
  (:init
   (&key
    ((:force_accepted __force_accepted) nil)
    )
   (send-super :init)
   (setq _force_accepted __force_accepted)
   self)
  (:force_accepted
   (&optional (__force_accepted :null))
   (if (not (eq __force_accepted :null)) (setq _force_accepted __force_accepted)) _force_accepted)
  (:serialization-length
   ()
   (+
    ;; bool _force_accepted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _force_accepted
       (if _force_accepted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _force_accepted
     (setq _force_accepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass llm_msgs::set_force
  :super ros::object
  :slots ())

(setf (get llm_msgs::set_force :md5sum-) "6b0fdeb4ed7ee4c97030abfd78488ebb")
(setf (get llm_msgs::set_force :datatype-) "llm_msgs/set_force")
(setf (get llm_msgs::set_force :request) llm_msgs::set_forceRequest)
(setf (get llm_msgs::set_force :response) llm_msgs::set_forceResponse)

(defmethod llm_msgs::set_forceRequest
  (:response () (instance llm_msgs::set_forceResponse :init)))

(setf (get llm_msgs::set_forceRequest :md5sum-) "6b0fdeb4ed7ee4c97030abfd78488ebb")
(setf (get llm_msgs::set_forceRequest :datatype-) "llm_msgs/set_forceRequest")
(setf (get llm_msgs::set_forceRequest :definition-)
      "float32 force0Ratio
float32 force1Ratio
float32 force2Ratio
float32 force3Ratio
float32 force4Ratio
float32 force5Ratio
---
bool force_accepted

")

(setf (get llm_msgs::set_forceResponse :md5sum-) "6b0fdeb4ed7ee4c97030abfd78488ebb")
(setf (get llm_msgs::set_forceResponse :datatype-) "llm_msgs/set_forceResponse")
(setf (get llm_msgs::set_forceResponse :definition-)
      "float32 force0Ratio
float32 force1Ratio
float32 force2Ratio
float32 force3Ratio
float32 force4Ratio
float32 force5Ratio
---
bool force_accepted

")



(provide :llm_msgs/set_force "6b0fdeb4ed7ee4c97030abfd78488ebb")


