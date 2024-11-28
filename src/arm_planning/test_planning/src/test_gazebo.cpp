#include "test_gazebo.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp> // 确保包含这个文件
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <iostream>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <cmath>
#include "ini.h"  // 需要安装 inih 库
#include <yaml-cpp/yaml.h>
ControlSystem::ControlSystem(const std::string& urdf_filename) {
    // ROS 订阅器和发布器初始化
    joint_state_sub_ = nh_.subscribe("/arm_controllers/joint_states", 10, &ControlSystem::jointStateCallback, this);
    jointErrorPub= nh_.advertise<sensor_msgs::JointState>("joint_error", 10);
    // 初始化关节力矩发布器
    torque_publishers_["shoulder_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint1_effort_controller/command", 10);
    torque_publishers_["shoulder_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint2_effort_controller/command", 10);
    torque_publishers_["shoulder_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint3_effort_controller/command", 10);
    torque_publishers_["elbow_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint4_effort_controller/command", 10);
    torque_publishers_["elbow_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint5_effort_controller/command", 10);
    torque_publishers_["wrist_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint6_effort_controller/command", 10);
    torque_publishers_["wrist_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint7_effort_controller/command", 10);
    joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    desired_histories.resize(7);
    joint_state.resize(7);
    // 加载动力学模型
    pinocchio::urdf::buildModel(urdf_filename, model_);
// 获取关节数量
    int njoints = model_.njoints;
    
    // 存储关节名称和ID的容器
    std::vector<std::string> joint_names(njoints);
    std::vector<int> joint_ids(njoints);
    
    // 遍历所有关节，获取名称和ID
    for (int i = 0; i < njoints; ++i) {
        // 获取关节的索引（在Pinocchio中，关节索引通常从1开始，但数组索引从0开始）
        pinocchio::JointIndex joint_idx = i + 1;
        
        // 获取关节名称
        joint_names[i] = model_.names[joint_idx];
        
        // 关节ID实际上就是它的索引（从1开始）
        joint_ids[i] = joint_idx;
        
        // 输出关节名称和ID
        std::cout << "Joint " << joint_idx << ": " << joint_names[i] << std::endl;
    }
    

    data_ = pinocchio::Data(model_);

    
    // 获取double类型的值 
    YAML::Node config = YAML::LoadFile("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/config/controller.yaml");
    // std::cout << "Reading YAML file from: " << filepath << std::endl;
    // YAML::Node config = YAML::LoadFile(filepath);

    // 检查 contr   oller 是否存在
    if (config["controller"]) {
        // 获取 lambda_gain 和 eta_gain
        test_lambda = config["controller"]["lambda_gain"].as<double>();
        test_eta = config["controller"]["eta_gain"].as<double>();
        test_k = config["controller"]["k"].as<double>();
        test_c = config["controller"]["c"].as<double>();
        
        std::cout << "Lambda Gain: " << test_lambda << std::endl;
        // std::cout << "Eta Gain: " << eta_gain << std::endl;
    } else {
        std::cerr << "Controller section not found!" << std::endl;
    }

        // 检查 contr   oller 是否存在
    if (config["impedance_controller"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        // impedance_M = config["impedance_controller"]["impedance_M"].as<double>();
        // impedance_B = config["impedance_controller"]["impedance_B"].as<double>();
        // impedance_K = config["impedance_controller"]["impedance_K"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }
    
}

ControlSystem::~ControlSystem() {}

void ControlSystem::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
   
    // 更新关节历史状态
    std::vector<std::string> joint_names = {
        "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint"};

    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto pos_it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (pos_it != msg->name.end()) 
        {
            size_t index = std::distance(msg->name.begin(), pos_it);
            joint_state[i].update(msg->position[index], msg->velocity[index]);
        }
    }
}

void ControlSystem::setJointTorque(const std::string& joint_name, double effort) 
{
    auto it = torque_publishers_.find(joint_name);
    if (it != torque_publishers_.end()) {
        std_msgs::Float64 effort_msg;
        effort_msg.data = effort;
        it->second.publish(effort_msg);
    } else {
        ROS_WARN("Torque publisher for joint %s not found.", joint_name.c_str());
    }
}

Eigen::VectorXd ControlSystem::computeTorqueWithSlidingMode(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                            const Eigen::VectorXd& q_desired, const Eigen::VectorXd& v_desired,
                                                            const Eigen::VectorXd& a_desired) 
{


    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(14); 
    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(14);
    Eigen::VectorXd a_desired_full = Eigen::VectorXd::Zero(14);



    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(7);
    joint_state_msg.position.resize(7);
    joint_state_msg.velocity.resize(7);
    joint_state_msg.name = 
    {
        "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint"
    };
    joint_state_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 7; i++)
    {
        joint_state_msg.position[i]=q(i);
    }
    joint_pub.publish(joint_state_msg);

    q_full.head(7) = q;  // 前7个关节位置
    v_full.head(7) = v;  // 前7个关节速度
    a_desired_full.head(7) = a_desired;  // 前7个关节加速度

    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(7, 7) * test_lambda; 
    double eta = test_eta;  // 趋近率增益
    double k = 0.0015;   // 滑模控制增益

    Eigen::VectorXd e = -(q.head(7) - q_desired);
    //=====================================================
        sensor_msgs::JointState joint_state;
        joint_state.name.resize(7); 
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"1","2","3","4","5","6","7"};// 替换为你的关节名称
        for (int i = 0; i < 7; ++i) 
        { 
            joint_state.position[i] = e(i);
        }
        jointErrorPub.publish(joint_state);
    //=====================================================



    Eigen::VectorXd e_dot = -(v.head(7) - v_desired);
    Eigen::VectorXd s = (e_dot + lambda * e);

    // 滑模面导数项
    Eigen::VectorXd s_dot = a_desired + lambda * e_dot;
    pinocchio::rnea(model_, data_, q_full,v_full,a_desired_full);  // 使用14维的q_full
    pinocchio::crba(model_,data_,q_full);
    pinocchio::computeCoriolisMatrix(model_, data_, q_full, v_full);  // 使用14维的q_full和v_full
    Eigen::MatrixXd C_full = data_.C;  // 计算完整的科里奥利矩阵
    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q_full);  // 计算完整的重力向量
    Eigen::MatrixXd M_full = data_.M;  // 计算完整的质量矩阵
    Eigen::VectorXd tanhS = Eigen::VectorXd::Zero(7);
    for (int i = 0; i < s.size(); ++i) {
        tanhS(i) = std::tanh(s(i)/test_eta); // 逐元素计算 tanh
    }
    s(1)=2*s(1);
    tanhS(1) = 2*tanhS(1);
    Eigen::VectorXd tau = 
                        // M_full.topLeftCorner(7,7) * a_desired
                         C_full.topLeftCorner(7, 7) * v_desired
                        // + G_full.head(7)+ test_k * s + test_c *tanhS; 
                        + test_lambda*e + test_eta* e_dot
                        + G_full.head(7); 

    
    // 计算所有框架的雅可比矩阵
    pinocchio::forwardKinematics(model_,data_,q_full);
    pinocchio::computeJointJacobians(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);

    // 获取框架 ID
    pinocchio::FrameIndex frame_id = model_.getFrameId("left_flange");
    std::cout << "frame_id: " <<frame_id << std::endl;

    Eigen::MatrixXd jacobian = pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jaco_left;
    jaco_left = jacobian.block<6,7>(0,0);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jaco_left);
    Eigen::MatrixXd jaco_pseudo_inv = cod.pseudoInverse();
    Eigen::MatrixXd null_space_projection = Eigen::MatrixXd::Identity(7, 7) - jaco_pseudo_inv * jaco_left;
    Eigen::VectorXd null = Eigen::VectorXd::Zero(7);
    if (sim< 2500)
    {

    }
    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Vector3d pos = data_.oMf[22].translation();
    Eigen::Quaterniond quaternion(data_.oMf[22].rotation());
    std::cout << "Position: " << data_.oMf[22].translation().transpose() << std::endl;
    std::cout << "Rotation (Quaternion): " << quaternion.coeffs().transpose() << std::endl;
    double time =ros::Time::now().toSec(); // 当前时间;
    for (int  i = 0; i < 7; i++)
    {
        /* code */
        null(i)= 300*sin(8*time);
    }
        // 0.3427  0.55786 0.688766
    null= null_space_projection*null;
    std::cout<<(tau+null).transpose()<<std::endl;
    return tau;
}

void ControlSystem::getMCGForFirst7Joints(const Eigen::VectorXd& q, const Eigen::VectorXd& v) 
{
    // 惯性矩阵 M
    pinocchio::crba(model_, data_, q);
    Eigen::MatrixXd M = data_.M.topLeftCorner(7, 7);
    pinocchio::computeCoriolisMatrix(model_, data_, q, v);
    Eigen::MatrixXd C = data_.C.topLeftCorner(7, 7);
    Eigen::VectorXd G = pinocchio::computeGeneralizedGravity(model_, data_, q).head(7);

    std::cout << "Mass matrix (M):\n" << M << "\nCoriolis matrix (C):\n" << C << "\nGravity vector (G):\n" << G.transpose() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_control_node");

    ControlSystem control_system("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf");

    Eigen::VectorXd q = Eigen::VectorXd::Zero(14);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(14);
    Eigen::VectorXd a_desired = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd v_desired = Eigen::VectorXd::Zero(7);

    ros::Rate loop_rate(400);

    while (ros::ok()) 
    {
        double time = ros::Time::now().toSec();

        Eigen::VectorXd desired_position = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd now_q(7);
        Eigen::VectorXd now_qd(7);

        Eigen::VectorXd max(7);
        Eigen::VectorXd min(7);
        max<<2.96,3.4,2.96,0,2.96,1.04,1.48;
        min<<-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;

        Eigen::VectorXd mid;
        mid = (max+min)/2;
        
        for (int i = 0; i < 7; ++i) 
        {   
            // desired_position(3) = -3.1415926535/2; // 设置每个关节的期望位置为 0.5 * sin(time)
            desired_position(i)= mid(i)+0.5*sin(1.5*time);

            now_q(i)=control_system.joint_state[i].position;
            now_qd(i)=control_system.joint_state[i].velocity;

        }
        // desired_position(3)= -mid(3)+0.5*sin(0.5*time);
        // 更新期望轨迹中的位置
        control_system.traj_data.addPosition(desired_position);

        // 获取期望的速度和加速度，返回值为 Eigen::VectorXd 类型
        Eigen::VectorXd desired_velocity = control_system.traj_data.getVelocities();
        Eigen::VectorXd desired_acceleration = control_system.traj_data.getAccelerations();
        
        // control_system.getMCGForFirst7Joints(q, v);
        Eigen::VectorXd tau = control_system.computeTorqueWithSlidingMode(now_q, now_qd, desired_position, desired_velocity, desired_acceleration);
        tau=tau;
        control_system.setJointTorque("shoulder_pitch_l_joint", tau(0));
        control_system.setJointTorque("shoulder_roll_l_joint", tau(1));
        control_system.setJointTorque("shoulder_yaw_l_joint", tau(2));
        control_system.setJointTorque("elbow_pitch_l_joint", tau(3));
        control_system.setJointTorque("elbow_yaw_l_joint", tau(4));
        control_system.setJointTorque("wrist_pitch_l_joint", tau(5));
        control_system.setJointTorque("wrist_roll_l_joint", tau(6));
        ros::spinOnce();
        loop_rate.sleep();
    }
    // [ 2.96, 3.4,  2.96, 0,     2.96,1.04,  1.48]
    // [-2.96,-0.26,-2.96, -2.96,-2.96,-1.04,-1.66]
    return 0;
}
