cmake_minimum_required(VERSION 3.0.2)
project(wbc)


set(MUJOCO_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/mujoco)
set(MUJOCO_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libmujoco.so)

find_package(glfw3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            sensor_msgs
            geometry_msgs
)



# 查找 Eigen 库
find_package(Eigen3 REQUIRED)

# 查找 Pinocchio 库
find_package(pinocchio REQUIRED)
# 生成 catkin 包
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ${MUJOCO_INCLUDE_DIRS}
    ${MUJOCO_LIBRARIES}
    osqp
    OsqpEigen
    CATKIN_DEPENDS roscpp
    )
link_directories(
    ${catkin_LIB_DIRS} lib
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)
# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  # ${OSQPEIGEN_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
  ${CMAKE_CURRENT_SOURCE_DIR}/mjsim
  ${CMAKE_CURRENT_SOURCE_DIR}/math
  ${CMAKE_CURRENT_SOURCE_DIR}/models
  ${CMAKE_CURRENT_SOURCE_DIR}/robotData
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 收集所有的源文件
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/math/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/mjsim/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/models/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/robotData/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp
)
# add_executable(testMain ${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp)
add_executable(testMain ${SOURCES})
add_executable(test1 ${SOURCES})


# add_library(${PROJECT_NAME} SHARED ${SOURCES})  # 创建一个共享库

# 链接库
target_link_libraries(testMain
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  ${MUJOCO_LIBRARIES}
  glfw
  osqp
  OsqpEigen
  ${GLFW3_LIBRARY}
)
target_link_libraries(test1
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  ${OSQPEIGEN_LIBRARIES}
  ${MUJOCO_LIBRARIES}
  glfw
  osqp
  OsqpEigen
  ${GLFW3_LIBRARY}
)

