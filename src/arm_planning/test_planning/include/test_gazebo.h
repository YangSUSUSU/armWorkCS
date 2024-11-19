#ifndef ROBOT_CONTROLLER_H  
#define ROBOT_CONTROLLER_H  

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>
#include <iostream>

// Pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <array>
#include <vector>
#include <ImpedanceControl.h>

struct JointTrajectoryData 
{
    std::array<Eigen::VectorXd, 4> positions;  // 记录四次位置，每个位置包含7个关节的值
    std::array<Eigen::VectorXd, 4> velocities; // 存储对应的速度
    std::array<Eigen::VectorXd, 4> accelerations; // 存储对应的加速度
    double dt = 1.0 / 400; // 固定的时间间隔

    JointTrajectoryData() 
    {
        // 初始化为7个关节的向量
        for (int i = 0; i < 4; ++i) {
            positions[i] = Eigen::VectorXd::Zero(7);
            velocities[i] = Eigen::VectorXd::Zero(7);
            accelerations[i] = Eigen::VectorXd::Zero(7);
        }
    }

    void addPosition(const Eigen::VectorXd& new_position) 
    {
        if (new_position.size() != 7) return;

        // 更新历史位置
        for (int i = 3; i > 0; --i) {
            positions[i] = positions[i - 1];
        }
        positions[0] = new_position;

        // 更新速度和加速度
        updateVelocities();
        updateAccelerations();
    }

    void updateVelocities() 
    {
        // 移动历史速度
        for (int i = 3; i > 0; --i) {
            velocities[i] = velocities[i - 1];
        }

        // 计算最新速度
        for (size_t i = 0; i < 7; ++i) 
        {
            velocities[0][i] = (positions[0][i] - positions[1][i]) / dt;
        }
    }

    void updateAccelerations() 
    {
        // 移动历史加速度
        for (int i = 3; i > 0; --i) {
            accelerations[i] = accelerations[i - 1];
        }

        // 计算最新加速度
        for (size_t i = 0; i < 7; ++i) 
        {
            accelerations[0][i] = (velocities[0][i] - velocities[1][i]) / dt;
        }
    }
    Eigen::VectorXd getAccelerations()
    {
        return accelerations[0];
    }
     Eigen::VectorXd getVelocities()
    {
        return velocities[0];
    }
};
struct JointState {
    double position = 0.0; // 当前关节的位置
    double velocity = 0.0; // 当前关节的速度

    // 更新关节状态的方法
    void update(double new_position, double new_velocity) {
        position = new_position;
        velocity = new_velocity;
    }
};
class ControlSystem {
public:
    // 构造函数和析构函数
    ControlSystem(const std::string& urdf_filename);
    ~ControlSystem();

    // 关节状态回调函数
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // 设置关节力矩
    void setJointTorque(const std::string& joint_name, double effort);

    // 使用滑模控制计算关节力矩
    Eigen::VectorXd computeTorqueWithSlidingMode(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                 const Eigen::VectorXd& q_desired, const Eigen::VectorXd& v_desired,
                                                 const Eigen::VectorXd& a_desired);

    // 获取前 7 个关节的质量矩阵、科里奥利矩阵和重力向量
    void getMCGForFirst7Joints(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    std::map<std::string, ros::Publisher> torque_publishers_;
    ros::Publisher jointErrorPub;
    pinocchio::Model model_;
    pinocchio::Data data_;

    // 期望轨迹的结构体
    JointTrajectoryData traj_data; 
    // 关节状态历史记录
    std::vector<JointTrajectoryData> desired_histories;
    std::vector<JointState> joint_state;
    double test_lambda=0.0;
    double test_eta=0.0;

    std::unique_ptr<ImpedanceControl> impedance_;
private:
    int control_mode;
};

#endif // TEST_GAZEBO_H