# Install script for directory: /home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nikoo/workWS/armWorkCS/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nikoo/workWS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_r/catkin_generated/installspace/aubo_arm_r.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r/cmake" TYPE FILE FILES
    "/home/nikoo/workWS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_r/catkin_generated/installspace/aubo_arm_rConfig.cmake"
    "/home/nikoo/workWS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_r/catkin_generated/installspace/aubo_arm_rConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r" TYPE FILE FILES "/home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r/config" TYPE DIRECTORY FILES "/home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r/launch" TYPE DIRECTORY FILES "/home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r/meshes" TYPE DIRECTORY FILES "/home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_r/urdf" TYPE DIRECTORY FILES "/home/nikoo/workWS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_r/urdf/")
endif()
