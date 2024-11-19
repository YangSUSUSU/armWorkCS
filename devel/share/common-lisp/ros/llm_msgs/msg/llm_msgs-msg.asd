
(cl:in-package :asdf)

(defsystem "llm_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "hand_pose_req" :depends-on ("_package_hand_pose_req"))
    (:file "_package_hand_pose_req" :depends-on ("_package"))
    (:file "pose_action_status" :depends-on ("_package_pose_action_status"))
    (:file "_package_pose_action_status" :depends-on ("_package"))
    (:file "robot_state" :depends-on ("_package_robot_state"))
    (:file "_package_robot_state" :depends-on ("_package"))
  ))