cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(test_planning)
# 设置 MuJoCo 路径

# MuJoCo 3.13 路径，替换成您的 MuJoCo 安装目录
set(MUJOCO_DIR "/home/nikoo/mujoco-3.1.3")
set(MUJOCO_LIB "/home/nikoo/mujoco-3.1.3/lib/libmujoco.so")
# set(MUJOCO_PATH "/home/nikoo/mujoco-3.1.3")
# 设置 MuJoCo 头文件路径
include_directories(${MUJOCO_PATH}/include)
# 设置 MuJoCo 库文件路径
link_directories(${MUJOCO_PATH}/lib)
find_package(glfw3 REQUIRED)

include_directories(${GLFW3_INCLUDE_DIR})
####################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
)
# 找到 Orocos KDL 库

find_package(Eigen3 REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
    osqp OsqpEigen
    CATKIN_DEPENDS roscpp
    )
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
    ${PINOCCHIO_INCLUDE_DIRS} 
    ${urdfdom_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(test_gazebo src/test_gazebo.cpp)

# add_executable(test_planning src/test_arm.cpp)

# add_executable(test_planning_force_calibration src/test_load_calibration.cpp)

# # 力数据生成器--sim used
# add_executable(test_planning_force_generator src/test_force_generator.cpp)
# # 力数据滤波器
# add_executable(test_planning_force_filter src/test_force_filter.cpp)
# # 导纳控制器
# add_executable(test_planning_ad src/test_admmittance.cpp)
add_executable(mujocoSim src/mujocoSim.cpp
                         src/sim_interface/MJ_interface.cpp
                         src/sim_interface/GLFW_callbacks.cpp)
target_link_libraries(mujocoSim ${catkin_LIBRARIES}
                    ${MUJOCO_LIB}
                    glfw
                    ${GLFW3_LIBRARY})

# target_link_libraries(test_planning ${catkin_LIBRARIES}
#                         osqp
#                         OsqpEigen)
# target_link_libraries(test_planning_ad ${catkin_LIBRARIES})
# target_link_libraries(test_planning_force_filter ${catkin_LIBRARIES})
# target_link_libraries(test_planning_force_generator ${catkin_LIBRARIES})
# target_link_libraries(test_planning_force_calibration 
#                             ${catkin_LIBRARIES}
#                             ${orocos_kdl_LIBRARIES}
#                             )
# target_include_directories(test_gazebo PRIVATE ${MUJOCO_PATH}/include)
target_link_libraries(test_gazebo
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${PINOCCHIO_LIBRARIES} ${urdfdom_LIBRARIES}
    osqp
    OsqpEigen
    ${MUJOCO_LIB}
    ${GLFW3_LIBRARY}

)
# target_link_libraries(dualArmMpc
#     osqp
#     OsqpEigen
#     ${EIGEN3_LIBRARIES}   # 确保链接 Eigen3
#     ${PINOCCHIO_LIBRARIES} 
# )