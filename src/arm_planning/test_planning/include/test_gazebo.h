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
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
// #include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>  // 用于时间记录
// Pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <array>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
using json = nlohmann::json;

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
            positions[i] = Eigen::VectorXd::Zero(15);
            velocities[i] = Eigen::VectorXd::Zero(15);
            accelerations[i] = Eigen::VectorXd::Zero(15);
        }
    }

    void addPosition(const Eigen::VectorXd& new_position) 
    {
        if (new_position.size() != 15) return;

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
        for (size_t i = 0; i < 15; ++i) 
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
        for (size_t i = 0; i < 15; ++i) 
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
    double position = 0.0;
    double velocity = 0.0; 
    void update(double new_position, double new_velocity) {
        position = new_position;
        velocity = new_velocity;
    }
};
class ControlSystem {
public:

     ControlSystem(const std::string& urdf_filename);
    ~ControlSystem();
    void messageCallback(const std_msgs::String::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void setJointTorque(const std::string& joint_name, double effort);
    Eigen::VectorXd computeTorqueWithSlidingMode(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                 const Eigen::VectorXd& q_desired, const Eigen::VectorXd& v_desired,
                                                 const Eigen::VectorXd& a_desired);
    Eigen::VectorXd cartesianImpedance(Eigen::VectorXd& carErr,Eigen::MatrixXd& joacb,Eigen::VectorXd& dynCompensated,Eigen::VectorXd& qd);
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &input);
    Eigen::VectorXd optimizeNullSpaceVelocity( const Eigen::MatrixXd& J,                   // 雅可比矩阵
                                            const Eigen::VectorXd& q_main,              // 主任务关节角速度增量
                                            double alpha,                              // 雅可比条件数最小化的权重
                                            double beta);   
    //4
    Eigen::VectorXd WQP(const Eigen::MatrixXd& Jl,
                                const Eigen::MatrixXd& Jr,
                                const Eigen::VectorXd& car_err,
                                const Eigen::VectorXd& nowQ,
                                const Eigen::VectorXd& Qr);
    void upDataCollision(Eigen::VectorXd& nowq);
    Eigen::VectorXd upDataBoundGradient(const Eigen::VectorXd& nowq);
    Eigen::MatrixXd upDataBoundH(const Eigen::VectorXd& nowq);
    //3                           
    Eigen::VectorXd interfaceWQP(Eigen::VectorXd& nowq,Eigen::VectorXd& nowqv,Eigen::VectorXd& Rcar);
    //2
    Eigen::VectorXd tra(Eigen::VectorXd& nowq,Eigen::VectorXd& nowqv);
    //1
    Eigen::VectorXd dualArmSys(Eigen::VectorXd& nowX,
                               std::vector<Eigen::VectorXd>& l_waypoints,
                               std::vector<Eigen::VectorXd>& r_waypoints);
    Eigen::VectorXd lr_Xr;
    Eigen::VectorXd nowX;
    bool left_done = false;
    bool right_done = false;
    bool allarm_done = true;
    Eigen::VectorXd carE; 
    double tolerance_last =  0.01;
    double tolerance_first = 0.02;
    // 存储左右臂的轨迹点
    std::vector<Eigen::VectorXd> l_waypoints;  // 左臂轨迹
    std::vector<Eigen::VectorXd> r_waypoints;  // 右臂轨迹
    ros::Publisher lnowX_pub;  // 发布器
    ros::Publisher rnowX_pub;  // 发布器


    std::unordered_map<int, std::chrono::steady_clock::time_point> waypoint_times_l;
    std::unordered_map<int, std::chrono::steady_clock::time_point> waypoint_times_r;
    int current_waypoint_index_l = 0;
    int current_waypoint_index_r = 0;


    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber car_json_sub;

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
    const int joint_num = 14;
    Eigen::VectorXd impedance_joint_M;
    Eigen::VectorXd impedance_joint_B;
    Eigen::VectorXd impedance_joint_K;

    Eigen::MatrixXd impedance_Cartesian_M;
    Eigen::MatrixXd impedance_Cartesian_B;
    Eigen::MatrixXd impedance_Cartesian_K;
    Eigen::VectorXd last_tau;
    Eigen::VectorXd collision_r;
    double safe_d = 0.08;
    Eigen::VectorXd m_testlp;
    Eigen::MatrixXd m_test_conA;
    Eigen::VectorXd joint_max;
    Eigen::VectorXd joint_min;
    double joint_bound_eta = 0.005;
    Eigen::VectorXd granCollision;

private:

};

#endif // TEST_GAZEBO_H