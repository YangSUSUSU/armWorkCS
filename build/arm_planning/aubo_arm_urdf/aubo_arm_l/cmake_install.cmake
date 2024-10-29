# Install script for directory: /home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dell/cat-WS/armWorkCS/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dell/cat-WS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_l/catkin_generated/installspace/aubo_arm_l.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l/cmake" TYPE FILE FILES
    "/home/dell/cat-WS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_l/catkin_generated/installspace/aubo_arm_lConfig.cmake"
    "/home/dell/cat-WS/armWorkCS/build/arm_planning/aubo_arm_urdf/aubo_arm_l/catkin_generated/installspace/aubo_arm_lConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l" TYPE FILE FILES "/home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l/config" TYPE DIRECTORY FILES "/home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l/launch" TYPE DIRECTORY FILES "/home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l/meshes" TYPE DIRECTORY FILES "/home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aubo_arm_l/urdf" TYPE DIRECTORY FILES "/home/dell/cat-WS/armWorkCS/src/arm_planning/aubo_arm_urdf/aubo_arm_l/urdf/")
endif()

