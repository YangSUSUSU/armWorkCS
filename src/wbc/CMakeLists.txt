cmake_minimum_required(VERSION 3.0.2)
project(wbc)

# 使用 catkin

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
    OsqpEigen
    CATKIN_DEPENDS roscpp
    )
link_directories(
    ${catkin_LIB_DIRS} lib
)
# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${OSQPEIGEN_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
  ${CMAKE_CURRENT_SOURCE_DIR}/math
  ${CMAKE_CURRENT_SOURCE_DIR}/models
  ${CMAKE_CURRENT_SOURCE_DIR}/robotData
)

# 收集所有的源文件
file(GLOB_RECURSE SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/math/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/models/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/robotData/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp
)
# add_executable(testMain ${CMAKE_CURRENT_SOURCE_DIR}/test/testMain.cpp)
add_executable(testMain ${SOURCES})



# 链接库
target_link_libraries(testMain
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  ${OSQPEIGEN_LIBRARIES}
)


