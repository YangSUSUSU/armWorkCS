cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(test_planning)
# find_package(Boost 1.77.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    llm_msgs
    sensor_msgs
    geometry_msgs
    arm_kinematics_solver
)
# 找到 Orocos KDL 库
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}   osqp OsqpEigen
    CATKIN_DEPENDS roscpp arm_kinematics_solver
    )
find_package(iDynTree REQUIRED)
    # 查找 Pinocchio
find_package(pinocchio REQUIRED)
# find_package(inih REQUIRED)
find_package(YAML-CPP REQUIRED)
# 查找 urdfdom（如 Pinocchio 使用 URDF）
find_package(urdfdom REQUIRED)
link_directories(
    ${catkin_LIB_DIRS} lib
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS} 
    ${urdfdom_INCLUDE_DIRS}
)
add_executable(test_gazebo src/test_gazebo.cpp)

add_executable(test_planning src/test_arm.cpp)

add_executable(test_planning_force_calibration src/test_load_calibration.cpp)

# 力数据生成器--sim used
add_executable(test_planning_force_generator src/test_force_generator.cpp)
# 力数据滤波器
add_executable(test_planning_force_filter src/test_force_filter.cpp)
# 导纳控制器
add_executable(test_planning_ad src/test_admmittance.cpp)

target_link_libraries(test_planning ${catkin_LIBRARIES}
                        osqp
                        OsqpEigen)
target_link_libraries(test_planning_ad ${catkin_LIBRARIES})
target_link_libraries(test_planning_force_filter ${catkin_LIBRARIES})
target_link_libraries(test_planning_force_generator ${catkin_LIBRARIES})
target_link_libraries(test_planning_force_calibration 
                            ${catkin_LIBRARIES}
                            ${orocos_kdl_LIBRARIES}
                            )

target_link_libraries(test_gazebo
    ${catkin_LIBRARIES}
    # inih
    ${YAML_CPP_LIBRARIES}
    iDynTree::idyntree-high-level iDynTree::idyntree-estimation
    ${PINOCCHIO_LIBRARIES} ${urdfdom_LIBRARIES}
)
