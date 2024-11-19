#include "test_gazebo.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp> // 确保包含这个文件
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <iostream>
// #include "ini.h"  // 需要安装 inih 库
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

    desired_histories.resize(7);
    joint_state.resize(7);
    // 加载动力学模型
    pinocchio::urdf::buildModel(urdf_filename, model_);
    // 获取关节数量
    int njoints = model_.njoints;
    
    // 存储关节名称和ID的容器
    std::vector<std::string> joint_names(njoints);
    std::vector<int> joint_ids(njoints);
    // std::cout << "model_.njoints :" << model_.njoints;
    
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
    YAML::Node config = YAML::LoadFile("/home/ubuntu/work/armWorkCS/src/arm_planning/test_planning/config/controller.yaml");
    // std::cout << "Reading YAML file from: " << filepath << std::endl;
    // YAML::Node config = YAML::LoadFile(filepath);

    // 检查 contr   oller 是否存在
    if (config["controller"]) {
        // 获取 lambda_gain 和 eta_gain
        test_lambda = config["controller"]["lambda_gain"].as<double>();
        test_eta = config["controller"]["eta_gain"].as<double>();
        
        std::cout << "Lambda Gain: " << test_lambda << std::endl;
        // std::cout << "Eta Gain: " << eta_gain << std::endl;
    } else {
        std::cerr << "Controller section not found!" << std::endl;
    }

    // get the impedance param
    if (config["impedance"]) {
        // 获取 lambda_gain 和 eta_gain
        double M_ = config["impedance"]["Mass"].as<double>();
        double B_ = config["impedance"]["Damping"].as<double>();
        double K_ = config["impedance"]["Stiff"].as<double>();
        bool sensor_ = config["impedance"]["sensor_used"].as<bool>();
        
        // std::cout << "M: " << M_ << "B: " << B_ <<  std::endl;
        impedance_ = std::make_unique<ImpedanceControl>(M_, B_, K_, 7, sensor_);
        // std::cout << "Eta Gain: " << eta_gain << std::endl;
    } else {
        std::cerr << "Controller section not found!" << std::endl;
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

    q_full.head(7) = q_desired;  // 前7个关节位置
    v_full.head(7) = v_desired;  // 前7个关节速度
    // a_desired_full.head(7) = a_desired;  // 前7个关节加速度

    // 滑模控制参数配置
    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(7, 7) * test_lambda; // 滑模增益
    double eta = test_eta;  // 趋近率增益
    double k = 0.0015;   // 滑模控制增益

    // 计算误差和滑模面
    Eigen::VectorXd e = -(q.head(7) - q_desired);
    // std::cout<<e.transpose()<<std::endl;
    //test Error used;
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
        // 发布消息
        jointErrorPub.publish(joint_state);



    Eigen::VectorXd e_dot = -(v.head(7) - v_desired);
    Eigen::VectorXd s = (e_dot + lambda * e);

    // 滑模面导数项
    Eigen::VectorXd s_dot = a_desired - lambda * e_dot;
    pinocchio::crba(model_, data_, q_full);  // 使用14维的q_full
    Eigen::MatrixXd M_full = data_.M;  // 计算完整的质量矩阵

    pinocchio::computeCoriolisMatrix(model_, data_, q_full, v_full);  // 使用14维的q_full和v_full
    Eigen::MatrixXd C_full = data_.C;  // 计算完整的科里奥利矩阵

    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q_full);  // 计算完整的重力向量

    // 控制律：加入滑模控制，提取前7个关节
    Eigen::VectorXd tau = M_full.topLeftCorner(7,7) * (a_desired + lambda * e_dot + eta * s.array().sign().matrix()) 
                        + C_full.topLeftCorner(7, 7) * v_full.head(7)
                        + G_full.head(7)+test_lambda*e + test_eta* e_dot; 

    ImpedanceControlState state_;
    // state_->q_d = q_desired;
    // state_->q_v = v_desired;


    // 控制律：加入滑模控制，提取前7个关节
    // Eigen::VectorXd tau = 3*(C_full.topLeftCorner(7, 7) * v_full.head(7)
    //                     + G_full.head(7))+test_lambda*e + test_eta* e_dot ;
    // Eigen::VectorXd tau =  test_lambda*e + test_eta* e_dot;                      
    // Eigen::VectorXd tau =Eigen::VectorXd::Zero(7);
    // tau(2)=10;
    // std::cout<<tau.transpose()<<std::endl;
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

    // std::cout << "Mass matrix (M):\n" << M << "\nCoriolis matrix (C):\n" << C << "\nGravity vector (G):\n" << G.transpose() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_control_node");

    ControlSystem control_system("/home/ubuntu/work/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf");

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
            // desired_position(3) = -0.5; // 设置每个关节的期望位置为 0.5 * sin(time)
            desired_position(i)= mid(i)+0.5*sin(0.5*time);

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