# Install script for directory: /home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/WorkSpace/armWorkCS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs/msg" TYPE FILE FILES
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/msg/hand_pose_req.msg"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/msg/pose_action_status.msg"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/msg/robot_state.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs/srv" TYPE FILE FILES
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/srv/get_angle_act.srv"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/srv/get_status.srv"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/srv/set_angle.srv"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/srv/set_force.srv"
    "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/srv/set_speed.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs/cmake" TYPE FILE FILES "/home/ubuntu/WorkSpace/armWorkCS/build/arm_planning/llm_msgs/catkin_generated/installspace/llm_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ubuntu/WorkSpace/armWorkCS/devel/include/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ubuntu/WorkSpace/armWorkCS/devel/share/roseus/ros/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ubuntu/WorkSpace/armWorkCS/devel/share/common-lisp/ros/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ubuntu/WorkSpace/armWorkCS/devel/share/gennodejs/ros/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ubuntu/WorkSpace/armWorkCS/devel/lib/python3/dist-packages/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ubuntu/WorkSpace/armWorkCS/devel/lib/python3/dist-packages/llm_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ubuntu/WorkSpace/armWorkCS/build/arm_planning/llm_msgs/catkin_generated/installspace/llm_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs/cmake" TYPE FILE FILES "/home/ubuntu/WorkSpace/armWorkCS/build/arm_planning/llm_msgs/catkin_generated/installspace/llm_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs/cmake" TYPE FILE FILES
    "/home/ubuntu/WorkSpace/armWorkCS/build/arm_planning/llm_msgs/catkin_generated/installspace/llm_msgsConfig.cmake"
    "/home/ubuntu/WorkSpace/armWorkCS/build/arm_planning/llm_msgs/catkin_generated/installspace/llm_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/llm_msgs" TYPE FILE FILES "/home/ubuntu/WorkSpace/armWorkCS/src/arm_planning/llm_msgs/package.xml")
endif()

