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
#include <sensor_msgs/JointState.h>  

// Pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <array>
#include <vector>


// 定义常量

// EKF类
class EKF {
public:
    int STATE_DIM = 14;  // 状态维度：7个关节的速度和加速度
    int MEAS_DIM = 7;    // 测量维度：7个关节的加速度
    Eigen::VectorXd state;           // 状态向量：速度和加速度
    Eigen::MatrixXd P;               // 状态协方差矩阵
    Eigen::MatrixXd Q;               // 过程噪声协方差
    Eigen::MatrixXd R;               // 测量噪声协方差
    Eigen::MatrixXd F;               // 状态转移矩阵
    Eigen::MatrixXd H;               // 测量矩阵
    const double noise_p = 0.000001;   // 过程噪声
    const double noise_m = 0.000001;   // 测量噪声
    Eigen::VectorXd prev_velocity;     // 上一次测得的速度（7维）

    EKF() 
    {
        state = Eigen::VectorXd::Zero(14);  // 初始化状态为0
        P = Eigen::MatrixXd::Identity(14, 14);  // 初始协方差矩阵
        Q = noise_p * Eigen::MatrixXd::Identity(14, 14);  // 过程噪声
        R = noise_m * Eigen::MatrixXd::Identity(7, 7);  // 测量噪声
        F = Eigen::MatrixXd::Zero(14, 14);  // 初始化状态转移矩阵
        H = Eigen::MatrixXd::Identity(7, 14);  // 测量矩阵
        prev_velocity = Eigen::VectorXd::Zero(7);  // 初始化上一速度（7维）
    }

// 预测步骤
        void predict(double dt) {
        F.setZero();  // 将 F 矩阵初始化为零

        for (int i = 0; i < 7; ++i) {
            F(i, i) = 1;        // dq(k+1) = dq(k) + ddq(k) * dt
            F(i, 7 + i) = dt;   // dq(k+1) = dq(k) + ddq(k) * dt
            F(7 + i, 7 + i) = 1; // ddq(k+1) = ddq(k)
        }

    // 状态预测
    state = F * state;

    // 打印预测后的状态
    std::cout << "Predicted state: " << state.transpose() << std::endl;

    // 协方差预测
    P = F * P * F.transpose() + Q;

    // 打印协方差矩阵
    std::cout << "Predicted covariance P: " << P << std::endl;

    // 检查状态中是否存在NaN
    for (int i = 0; i < STATE_DIM; ++i) {
        if (std::isnan(state(i))) {
            std::cout << "NaN detected in state at index " << i << std::endl;
            state(i) = 0;  // 强制设置为0
        }
    }
}

// 更新步骤
void update(const Eigen::VectorXd& velocity, double dt) {

    std::cout << "====dt====" << dt<<std::endl;
    if (dt <= 0.0) {
        std::cerr << "Warning: dt is non-positive! Skipping update." << std::endl;
        return;
    }
    // 计算加速度（通过速度差分）
    Eigen::VectorXd measured_accel(7);
    for (int i = 0; i < 7; ++i) {
        measured_accel(i) = (velocity(i) - state(i)) / dt;
    }

    // 打印计算的加速度
    std::cout << "Measured accelerations: " << measured_accel.transpose() << std::endl;

    // 卡尔曼增益
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // 打印 S 矩阵，检查其是否正定
    std::cout << "S matrix: " << S << std::endl;

    Eigen::MatrixXd K = P * H.transpose() * S.ldlt().solve(Eigen::MatrixXd::Identity(MEAS_DIM, MEAS_DIM));

    // 状态更新
    Eigen::VectorXd y = measured_accel - H * state;
    state = state + K * y;

    // 打印更新后的状态
    std::cout << "Updated state after correction: " << state.transpose() << std::endl;

    // 协方差更新
    P = (Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) - K * H) * P;

    // 打印更新后的协方差矩阵
    std::cout << "Updated covariance P: " << P << std::endl;

    // 更新速度（用加速度进行修正）
    for (int i = 0; i < 7; ++i) {
        state(i) = velocity(i);  // 使用传入的 velocity 来更新速度
    }

    // 更新加速度（通过加速度差分修正）
    for (int i = 7; i < STATE_DIM; ++i) {
        state(i) = measured_accel(i - 7);  // 更新加速度部分
    }

    // 检查状态中是否存在NaN
    for (int i = 0; i < STATE_DIM; ++i) {
        if (std::isnan(state(i))) {
            std::cout << "NaN detected in state after update at index " << i << std::endl;
            state(i) = 0;  // 强制设置为0
        }
    }
}

    // 获取估计的加速度
    Eigen::VectorXd getQdd() const 
    {
        return state.tail(7);  // 返回加速度部分
    }
};



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
    ros::Publisher joint_pub;
    pinocchio::Model model_;
    pinocchio::Data data_;

    // 期望轨迹的结构体
    JointTrajectoryData traj_data; 
    // 关节状态历史记录
    std::vector<JointTrajectoryData> desired_histories;
    std::vector<JointState> joint_state;
    double test_lambda=0.0;
    double test_eta=0.0;
    double test_k = 0.0;
    double test_c = 0.0;
    int sim = 0 ;
    EKF leftArmEKF;
private:

};

#endif // TEST_GAZEBO_H