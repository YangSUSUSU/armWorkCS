#include "test_gazebo.h"
#include <iostream>

// ROS Interface 类实现
RosInterface::RosInterface() {
    joint_state_sub_ = nh_.subscribe("/arm_controllers/joint_states", 10, &RosInterface::jointStateCallback, this);

    // 初始化关节力矩发布器
    torque_publishers_["shoulder_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint1_effort_controller/command", 10);
    torque_publishers_["shoulder_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint2_effort_controller/command", 10);
    torque_publishers_["shoulder_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint3_effort_controller/command", 10);
    torque_publishers_["elbow_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint4_effort_controller/command", 10);
    torque_publishers_["elbow_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint5_effort_controller/command", 10);
    torque_publishers_["wrist_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint6_effort_controller/command", 10);
    torque_publishers_["wrist_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint7_effort_controller/command", 10);
}

RosInterface::~RosInterface() {}

void RosInterface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_positions_[msg->name[i]] = msg->position[i];
        joint_velocities_[msg->name[i]] = msg->velocity[i];
        joint_efforts_[msg->name[i]] = msg->effort[i];
    }
}

void RosInterface::printJointStates() const {
    std::cout << "Current Joint States:\n";
    for (const auto& joint : joint_positions_) {
        const std::string& joint_name = joint.first;
        double position = joint.second;
        double velocity = joint_velocities_.at(joint_name);
        double effort = joint_efforts_.at(joint_name);

        std::cout << "Joint: " << joint_name << "\n";
        std::cout << "  Position: " << position << "\n";
        std::cout << "  Velocity: " << velocity << "\n";
        std::cout << "  Effort: " << effort << "\n";
    }
    std::cout << "--------------------------\n";
}

void RosInterface::setJointTorque(const std::string& joint_name, double effort) {
    auto it = torque_publishers_.find(joint_name);
    if (it != torque_publishers_.end()) {
        std_msgs::Float64 effort_msg;
        effort_msg.data = effort;
        // effort_msg.header.stamp = ros::Time::now();
        it->second.publish(effort_msg);
    } else {
        ROS_WARN("Torque publisher for joint %s not found.", joint_name.c_str());
    }
}

// DynamicsCalculator 类实现
DynamicsCalculator::DynamicsCalculator(const std::string& urdf_filename)
{
    pinocchio::urdf::buildModel(urdf_filename, model_);
    pinocchio::Data data(model_);
    data_ = data;
    // data_(model_);
}

DynamicsCalculator::~DynamicsCalculator() {}

void DynamicsCalculator::forwardKinematics(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateGlobalPlacements(model_, data_);
}

Eigen::VectorXd DynamicsCalculator::computeJointTorques(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a) {
    return pinocchio::rnea(model_, data_, q, v, a);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_control_node");

    // 创建 ROS 接口
    RosInterface ros_interface;

    // 创建动力学计算对象
    DynamicsCalculator dynamics_calculator("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf");
    
    // 设置关节的位置、速度和加速度为零
    Eigen::VectorXd q = Eigen::VectorXd::Zero(14);  // 假设机器人有7个关节
    Eigen::VectorXd v = Eigen::VectorXd::Zero(14);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(14);

    ros::Rate loop_rate(400);  // 10 Hz

    while (ros::ok()) {
        ros::spinOnce();

        // 进行正向运动学计算
        dynamics_calculator.forwardKinematics(q);
        // s = e+Ced
        // 计算逆动力学，得到保持在零位置的关节力矩
        double time = ros::Time::now().toSec(); 
        q(3) = 3*sin(time);
        Eigen::VectorXd tau = dynamics_calculator.computeJointTorques(q, v, a);

        // 打印关节力矩
        std::cout << "Joint torques to keep joints at zero position:" << std::endl;
        std::cout << tau.head(7).transpose() << std::endl;

        // 示例：下发力矩指令
        ros_interface.setJointTorque("shoulder_pitch_l_joint", tau[0]);
        ros_interface.setJointTorque("shoulder_roll_l_joint", tau[1]);
        ros_interface.setJointTorque("shoulder_yaw_l_joint", tau[2]);
        ros_interface.setJointTorque("elbow_pitch_l_joint", tau[3]);
        ros_interface.setJointTorque("elbow_yaw_l_joint", tau[4]);
        ros_interface.setJointTorque("wrist_pitch_l_joint", tau[5]);        
        ros_interface.setJointTorque("wrist_roll_l_joint", tau[6]);


        loop_rate.sleep();
    }

    return 0;
}
