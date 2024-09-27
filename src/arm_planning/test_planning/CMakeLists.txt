cmake_minimum_required(VERSION 2.8.3)
project(test_planning)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    llm_msgs
    geometry_msgs
    arm_kinematics_solver
)
catkin_package(
    LIBRARIES ${PROJECT_NAME}  
    CATKIN_DEPENDS roscpp arm_kinematics_solver
    )

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)
add_executable(test_planning src/test_arm.cpp)

# 力数据生成器--sim used
add_executable(test_planning_force_generator src/test_force_generator.cpp)
# 力数据滤波器
add_executable(test_planning_force_filter src/test_force_filter.cpp)
# 导纳控制器
add_executable(test_planning_ad src/test_admmittance.cpp)

target_link_libraries(test_planning ${catkin_LIBRARIES})
target_link_libraries(test_planning_ad ${catkin_LIBRARIES})
target_link_libraries(test_planning_force_filter ${catkin_LIBRARIES})
target_link_libraries(test_planning_force_generator ${catkin_LIBRARIES})

