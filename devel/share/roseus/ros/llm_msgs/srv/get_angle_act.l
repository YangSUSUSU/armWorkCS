;; Auto-generated. Do not edit!


(when (boundp 'llm_msgs::get_angle_act)
  (if (not (find-package "LLM_MSGS"))
    (make-package "LLM_MSGS"))
  (shadow 'get_angle_act (find-package "LLM_MSGS")))
(unless (find-package "LLM_MSGS::GET_ANGLE_ACT")
  (make-package "LLM_MSGS::GET_ANGLE_ACT"))
(unless (find-package "LLM_MSGS::GET_ANGLE_ACTREQUEST")
  (make-package "LLM_MSGS::GET_ANGLE_ACTREQUEST"))
(unless (find-package "LLM_MSGS::GET_ANGLE_ACTRESPONSE")
  (make-package "LLM_MSGS::GET_ANGLE_ACTRESPONSE"))

(in-package "ROS")





(defclass llm_msgs::get_angle_actRequest
  :super ros::object
  :slots ())

(defmethod llm_msgs::get_angle_actRequest
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

(defclass llm_msgs::get_angle_actResponse
  :super ros::object
  :slots (_curangleRatio ))

(defmethod llm_msgs::get_angle_actResponse
  (:init
   (&key
    ((:curangleRatio __curangleRatio) (make-array 6 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _curangleRatio __curangleRatio)
   self)
  (:curangleRatio
   (&optional __curangleRatio)
   (if __curangleRatio (setq _curangleRatio __curangleRatio)) _curangleRatio)
  (:serialization-length
   ()
   (+
    ;; float32[6] _curangleRatio
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[6] _curangleRatio
     (dotimes (i 6)
       (sys::poke (elt _curangleRatio i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[6] _curangleRatio
   (dotimes (i (length _curangleRatio))
     (setf (elt _curangleRatio i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(defclass llm_msgs::get_angle_act
  :super ros::object
  :slots ())

(setf (get llm_msgs::get_angle_act :md5sum-) "5a40d1cc41b6533f906a6a90344676b6")
(setf (get llm_msgs::get_angle_act :datatype-) "llm_msgs/get_angle_act")
(setf (get llm_msgs::get_angle_act :request) llm_msgs::get_angle_actRequest)
(setf (get llm_msgs::get_angle_act :response) llm_msgs::get_angle_actResponse)

(defmethod llm_msgs::get_angle_actRequest
  (:response () (instance llm_msgs::get_angle_actResponse :init)))

(setf (get llm_msgs::get_angle_actRequest :md5sum-) "5a40d1cc41b6533f906a6a90344676b6")
(setf (get llm_msgs::get_angle_actRequest :datatype-) "llm_msgs/get_angle_actRequest")
(setf (get llm_msgs::get_angle_actRequest :definition-)
      "---
float32[6] curangleRatio

")

(setf (get llm_msgs::get_angle_actResponse :md5sum-) "5a40d1cc41b6533f906a6a90344676b6")
(setf (get llm_msgs::get_angle_actResponse :datatype-) "llm_msgs/get_angle_actResponse")
(setf (get llm_msgs::get_angle_actResponse :definition-)
      "---
float32[6] curangleRatio

")



(provide :llm_msgs/get_angle_act "5a40d1cc41b6533f906a6a90344676b6")


