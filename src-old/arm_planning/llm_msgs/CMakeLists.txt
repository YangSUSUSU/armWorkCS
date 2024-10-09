cmake_minimum_required(VERSION 2.8.3)
project(llm_msgs)

find_package(catkin REQUIRED 
    message_generation
    geometry_msgs
    std_msgs
    rospy 
    roscpp
)

add_message_files(
  FILES
  hand_pose_req.msg
  pose_action_status.msg
  robot_state.msg
)

add_service_files(
  FILES
  get_angle_act.srv
  get_status.srv
  set_angle.srv
  set_force.srv
  set_speed.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)