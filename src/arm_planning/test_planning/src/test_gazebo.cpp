#include "test_gazebo.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp> // 确保包含这个文件
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/centroidal.hpp>
#include <iostream>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <cmath>
#include "ini.h"  // 需要安装 inih 库
#include <yaml-cpp/yaml.h>
#include <random>
 #include <interactive_markers/interactive_marker_server.h>
ControlSystem::ControlSystem(const std::string& urdf_filename) 
{
    pinocchio::urdf::buildModel(urdf_filename, model_);
    int njoints = model_.njoints;
    std::vector<std::string> joint_names(njoints);
    std::vector<int> joint_ids(njoints);

    for (int i = 0; i < njoints; ++i) 
    {
        pinocchio::JointIndex joint_idx = i + 1;
        joint_names[i] = model_.names[joint_idx];
        joint_ids[i] = joint_idx;
        std::cout << "Joint " << joint_idx << ": " << joint_names[i] << std::endl;
    }
    data_ = pinocchio::Data(model_);
    lnowX_pub = nh_.advertise<geometry_msgs::PoseStamped>("/wbc_current_pose_l", 10);
    rnowX_pub = nh_.advertise<geometry_msgs::PoseStamped>("/wbc_current_pose_r", 10);
    car_json_sub = nh_.subscribe("/wbc_absolute_motion", 10, &ControlSystem::messageCallback, this);
    // joint_sub = nh_.subscribe("/wbc_joint_motion", 10, &ControlSystem::jointmessageCallback, this);

    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ControlSystem::jointStateCallback, this);

    jointErrorPub= nh_.advertise<sensor_msgs::JointState>("/joint_error", 10);
    torque_publishers_["waist_yaw_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint0_effort_controller/command", 10);
    torque_publishers_["shoulder_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint1_effort_controller/command", 10);
    torque_publishers_["shoulder_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint2_effort_controller/command", 10);
    torque_publishers_["shoulder_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint3_effort_controller/command", 10);
    torque_publishers_["elbow_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint4_effort_controller/command", 10);
    torque_publishers_["elbow_yaw_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint5_effort_controller/command", 10);
    torque_publishers_["wrist_pitch_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint6_effort_controller/command", 10);
    torque_publishers_["wrist_roll_l_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint7_effort_controller/command", 10);
    torque_publishers_["shoulder_pitch_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint8_effort_controller/command", 10);
    torque_publishers_["shoulder_roll_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint9_effort_controller/command", 10);
    torque_publishers_["shoulder_yaw_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint10_effort_controller/command", 10);
    torque_publishers_["elbow_pitch_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint11_effort_controller/command", 10);
    torque_publishers_["elbow_yaw_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint12_effort_controller/command", 10);
    torque_publishers_["wrist_pitch_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint13_effort_controller/command", 10);
    torque_publishers_["wrist_roll_r_joint"] = nh_.advertise<std_msgs::Float64>("/arm_controllers/joint14_effort_controller/command", 10);

    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================

    joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

    //避障相关
    collision_r = Eigen::VectorXd::Zero(16);
    collision_r<< 0.15,0.15,0.15,0.15,
                0.045,0.045,0.045,0.045,0.045,0.1,
                0.045,0.045,0.045,0.045,0.045,0.1;
    m_testlp = Eigen::VectorXd::Zero(9);
    m_test_conA = Eigen::MatrixXd::Zero(9, 17);

    desired_histories.resize(18);
    joint_state.resize(17);
    lr_Xr = Eigen::VectorXd::Zero(14);
    nowX = Eigen::VectorXd::Zero(14);
    carE = Eigen::VectorXd::Zero(12);
    joint_max= Eigen::VectorXd::Zero(17);
    joint_min= Eigen::VectorXd::Zero(17);
    granCollision =Eigen::VectorXd::Zero(17);


    joint_max<< 0,0.15,0.25,
                0.79,2.87,2.27,0,2.27,0.8,0.8,
                3.14,0.17,2.27,2.27,2.27,0.8,0.8;
    joint_min<<-0.1,-0.15,-0.25,
                -3.14,-0.17,-2.27,-2.27,-2.27,-0.8,-0.8,
                -0.79,-2.87,-2.27,0,-2.27,-0.8,-0.8;
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    //===================================================================================================================================
    impedance_joint_M = Eigen::VectorXd::Zero(8);
    impedance_joint_B = Eigen::VectorXd::Zero(8);
    impedance_joint_K = Eigen::VectorXd::Zero(8);
    impedance_Cartesian_M =  Eigen::MatrixXd::Identity(6, 6);
    impedance_Cartesian_B =  Eigen::MatrixXd::Identity(6, 6);
    impedance_Cartesian_K =  Eigen::MatrixXd::Identity(6, 6);
    last_tau = Eigen::VectorXd::Zero(15);


    YAML::Node config = YAML::LoadFile("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/config/controller.yaml");

    if (config["controller"]) 
    {
        test_lambda = config["controller"]["lambda_gain"].as<double>();
        test_eta = config["controller"]["eta_gain"].as<double>();
        test_k = config["controller"]["k"].as<double>();
        test_c = config["controller"]["c"].as<double>();

    } 
    else 
    {
        std::cerr << "Controller section not found!" << std::endl;
    }

        // 检查 contr   oller 是否存在
    if (config["impedance_M"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_joint_M(0) = config["impedance_M"]["j1"].as<double>();
        impedance_joint_M(1) = config["impedance_M"]["j2"].as<double>();
        impedance_joint_M(2) = config["impedance_M"]["j3"].as<double>();
        impedance_joint_M(3) = config["impedance_M"]["j4"].as<double>();
        impedance_joint_M(4) = config["impedance_M"]["j5"].as<double>();
        impedance_joint_M(5) = config["impedance_M"]["j6"].as<double>();
        impedance_joint_M(6) = config["impedance_M"]["j7"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }
    
    if (config["impedance_B"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_joint_B(0) = config["impedance_B"]["j1"].as<double>();
        impedance_joint_B(1) = config["impedance_B"]["j2"].as<double>();
        impedance_joint_B(2) = config["impedance_B"]["j3"].as<double>();
        impedance_joint_B(3) = config["impedance_B"]["j4"].as<double>();
        impedance_joint_B(4) = config["impedance_B"]["j5"].as<double>();
        impedance_joint_B(5) = config["impedance_B"]["j6"].as<double>();
        impedance_joint_B(6) = config["impedance_B"]["j7"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }

    if (config["impedance_K"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_joint_K(0) = config["impedance_K"]["j1"].as<double>();
        impedance_joint_K(1) = config["impedance_K"]["j2"].as<double>();
        impedance_joint_K(2) = config["impedance_K"]["j3"].as<double>();
        impedance_joint_K(3) = config["impedance_K"]["j4"].as<double>();
        impedance_joint_K(4) = config["impedance_K"]["j5"].as<double>();
        impedance_joint_K(5) = config["impedance_K"]["j6"].as<double>();
        impedance_joint_K(6) = config["impedance_K"]["j7"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }

    if (config["impedance_Cartesian_M"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_Cartesian_M(0,0) = config["impedance_Cartesian_M"]["x"].as<double>();
        impedance_Cartesian_M(1,1) = config["impedance_Cartesian_M"]["y"].as<double>();
        impedance_Cartesian_M(2,2) = config["impedance_Cartesian_M"]["z"].as<double>();
        impedance_Cartesian_M(3,3) = config["impedance_Cartesian_M"]["rx"].as<double>();
        impedance_Cartesian_M(4,4) = config["impedance_Cartesian_M"]["ry"].as<double>();
        impedance_Cartesian_M(5,5) = config["impedance_Cartesian_M"]["rz"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }
    
    if (config["impedance_Cartesian_B"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_Cartesian_B(0,0) = config["impedance_Cartesian_B"]["x"].as<double>();
        impedance_Cartesian_B(1,1) = config["impedance_Cartesian_B"]["y"].as<double>();
        impedance_Cartesian_B(2,2) = config["impedance_Cartesian_B"]["x"].as<double>();
        impedance_Cartesian_B(3,3) = config["impedance_Cartesian_B"]["rx"].as<double>();
        impedance_Cartesian_B(4,4) = config["impedance_Cartesian_B"]["ry"].as<double>();
        impedance_Cartesian_B(5,5) = config["impedance_Cartesian_B"]["rz"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }

    if (config["impedance_Cartesian_K"]) 
    {
        // 获取 lambda_gain 和 eta_gain
        impedance_Cartesian_K(0,0) = config["impedance_Cartesian_K"]["x"].as<double>();
        impedance_Cartesian_K(1,1) = config["impedance_Cartesian_K"]["y"].as<double>();
        impedance_Cartesian_K(2,2) = config["impedance_Cartesian_K"]["z"].as<double>();
        impedance_Cartesian_K(3,3) = config["impedance_Cartesian_K"]["rx"].as<double>();
        impedance_Cartesian_K(4,4) = config["impedance_Cartesian_K"]["ry"].as<double>();
        impedance_Cartesian_K(5,5) = config["impedance_Cartesian_K"]["rz"].as<double>();

    } else 
    {
        std::cerr << "impedance_controller section not found!" << std::endl;
    }
}

ControlSystem::~ControlSystem() {}

void ControlSystem::messageCallback(const std_msgs::String::ConstPtr& msg)
{
    // if (!allarm_done)
    // {
    //    return;
    // }
    
    allarm_done = false;
    left_done = false;
    right_done = false;
    carE.setConstant(100);
    try {
            // 获取消息中的字符串
            std::string json_string = msg->data;

            // 解析 JSON 字符串
            json j = json::parse(json_string);

            // 清空之前的轨迹点数据
            l_waypoints.clear();
            r_waypoints.clear();

            // 解析 left_waypoints 和 right_waypoints
            if (j.contains("left_waypoints")) {
                std::vector<std::vector<double>> left_waypoints_data = j["left_waypoints"];
                std::cout << "Left Waypoints:" << std::endl;
                for (const auto& point : left_waypoints_data) {
                    Eigen::VectorXd point_vec(point.size());
                    for (size_t i = 0; i < point.size(); ++i) {
                        point_vec(i) = point[i];
                    }
                    l_waypoints.push_back(point_vec);  // 存储左臂轨迹点
                    // 输出调试信息
                    for (const auto& value : point) {
                        std::cout << value << " ";
                    }
                    std::cout << std::endl;
                }
            }

            if (j.contains("right_waypoints")) {
                std::vector<std::vector<double>> right_waypoints_data = j["right_waypoints"];
                std::cout << "Right Waypoints:" << std::endl;
                if (right_waypoints_data.empty()) {
                    std::cout << "No waypoints available." << std::endl;
                    right_done =true;
                } else {
                    for (const auto& point : right_waypoints_data) {
                        Eigen::VectorXd point_vec(point.size());
                        for (size_t i = 0; i < point.size(); ++i) {
                        point_vec(i) = point[i];
                    }
                    r_waypoints.push_back(point_vec);  // 存储右臂轨迹点
                    // 输出调试信息
                    for (const auto& value : point) {
                        std::cout << value << " ";
                    }
                    std::cout << std::endl;
                }
            }
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse JSON: %s", e.what());
    }

}

void ControlSystem::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
   
    // 更新关节历史状态
    std::vector<std::string> joint_names = {
            "waist_Z_joint",
            "waist_roll_joint",
            // "waist_pitch_joint",
            "waist_yaw_joint",
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6",
            "left_joint7",
            "right_joint1",
            "right_joint2",
            "right_joint3",
            "right_joint4",
            "right_joint5",
            "right_joint6",
            "right_joint7"
            // "shoulder_pitch_l_joint",
            // "shoulder_roll_l_joint",
            // "shoulder_yaw_l_joint",
            // "elbow_pitch_l_joint",
            // "elbow_yaw_l_joint",
            // "wrist_pitch_l_joint",
            // "wrist_roll_l_joint",
            // "shoulder_pitch_r_joint",
            // "shoulder_roll_r_joint",
            // "shoulder_yaw_r_joint",
            // "elbow_pitch_r_joint",
            // "elbow_yaw_r_joint",
            // "wrist_pitch_r_joint",
            // "wrist_roll_r_joint"};
            };

    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto pos_it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (pos_it != msg->name.end()) 
        {
            size_t index = std::distance(msg->name.begin(), pos_it);
            joint_state[i].update(msg->position[index], msg->velocity[index]);
        }
    }

    Eigen::VectorXd nowq = Eigen::VectorXd::Zero(17);
    for (int i = 0; i < 17; i++)
    {
        nowq(i) =   joint_state[i].position;
    }

    pinocchio::forwardKinematics(model_, data_, nowq);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::FrameIndex frame_id_left = model_.getFrameId("L_hand_base_link");
    pinocchio::FrameIndex frame_id_right = model_.getFrameId("R_hand_base_link");
    Eigen::Vector3d pos_l = data_.oMf[frame_id_left].translation();
    Eigen::Vector3d pos_r = data_.oMf[frame_id_right].translation();
    pos_l(2) = pos_l(2) - 0.5;
    pos_r(2) = pos_r(2) - 0.5;

    Eigen::Quaterniond quat_l(data_.oMf[frame_id_left].rotation());
    Eigen::Quaterniond quat_r(data_.oMf[frame_id_right].rotation());
    nowX.head(3) = pos_l;
    nowX(3+0)=quat_l.x();
    nowX(3+1)=quat_l.y();
    nowX(3+2)=quat_l.z();
    nowX(3+3)=quat_l.w();
    nowX.segment(7,3) = pos_r;
    nowX(10+0)=quat_r.x();
    nowX(10+1)=quat_r.y();
    nowX(10+2)=quat_r.z();
    nowX(10+3)=quat_r.w();




    geometry_msgs::PoseStamped pose_msg_l;
    geometry_msgs::PoseStamped pose_msg_r;

    pose_msg_l.header.stamp = ros::Time::now();
    pose_msg_l.header.frame_id = "base_link";

    pose_msg_r.header.stamp = ros::Time::now();
    pose_msg_r.header.frame_id = "base_link";

    pose_msg_l.pose.position.x = nowX(0);
    pose_msg_l.pose.position.y = nowX(1);
    pose_msg_l.pose.position.z = nowX(2);
    pose_msg_l.pose.orientation.x = nowX(3);
    pose_msg_l.pose.orientation.y = nowX(4);
    pose_msg_l.pose.orientation.z = nowX(5);
    pose_msg_l.pose.orientation.w = nowX(6);
    
    pose_msg_r.pose.position.x = nowX(0+7);
    pose_msg_r.pose.position.y = nowX(1+7);
    pose_msg_r.pose.position.z = nowX(2+7);
    pose_msg_r.pose.orientation.x = nowX(3+7);
    pose_msg_r.pose.orientation.y = nowX(4+7);
    pose_msg_r.pose.orientation.z = nowX(5+7);
    pose_msg_r.pose.orientation.w = nowX(6+7);

    lnowX_pub.publish(pose_msg_l);
    rnowX_pub.publish(pose_msg_r);    // return pose_msg;
    // std::cout<<nowX.transpose()<<std::endl;

}
Eigen::VectorXd ControlSystem::upDataBoundGradient(const Eigen::VectorXd& nowq)
{
    Eigen::VectorXd res_grad = Eigen::VectorXd::Zero(17);
    for (int i = 0; i < 17; i++)
    {
        if(nowq(i)<joint_min(i) + joint_bound_eta)
        {
            res_grad(i) =  1/( nowq(i) - joint_min(i) +0.000001);
        }
        else if (nowq(i)>joint_max(i) - joint_bound_eta)
        {
            res_grad(i) =  1/( nowq(i) - joint_max(i)  +0.000001);
        }
        else
        {
            res_grad(i) = 0.0;
        }
    }
    return res_grad;
}
Eigen::MatrixXd ControlSystem::upDataBoundH(const Eigen::VectorXd& nowq)
{
    Eigen::VectorXd res_grad = Eigen::VectorXd::Zero(17);
    Eigen::MatrixXd res_m = Eigen::MatrixXd::Identity(17, 17) ; 
    for (int i = 0; i < 17; i++)
    {
        if(nowq(i)<joint_min(i) + joint_bound_eta)
        {
            res_m(i,i) =  1000 * joint_bound_eta * abs( nowq(i) - joint_min(i) );
        }
        else if (nowq(i)>joint_max(i) - joint_bound_eta)
        {
            res_m(i,i) =  1000 * joint_bound_eta * abs( nowq(i) - joint_max(i) );
        }
        else
        {
            res_m(i,i) = 0.0;
        }
    }
     
    return res_m;
}

void ControlSystem::upDataCollision(Eigen::VectorXd& nowq) 
{
    pinocchio::forwardKinematics(model_, data_, nowq);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::FrameIndex b1 = model_.getFrameId("base_collision_1");
    pinocchio::FrameIndex b2 = model_.getFrameId("base_collision_2");
    pinocchio::FrameIndex b3 = model_.getFrameId("base_collision_3");
    pinocchio::FrameIndex b4 = model_.getFrameId("base_collision_4");

    // pinocchio::FrameIndex l1 = model_.getFrameId("left_collision_11");
    pinocchio::FrameIndex l21 = model_.getFrameId("left_collision_21");
    pinocchio::FrameIndex l22 = model_.getFrameId("left_collision_22");
    pinocchio::FrameIndex l31 = model_.getFrameId("left_collision_31");
    pinocchio::FrameIndex l41 = model_.getFrameId("left_collision_41");
    pinocchio::FrameIndex l51 = model_.getFrameId("left_collision_51");
    pinocchio::FrameIndex l61 = model_.getFrameId("left_collision_61");

    // pinocchio::FrameIndex r1 = model_.getFrameId("right_collision_11");
    pinocchio::FrameIndex r21 = model_.getFrameId("right_collision_21");
    pinocchio::FrameIndex r22 = model_.getFrameId("right_collision_22");
    pinocchio::FrameIndex r31 = model_.getFrameId("right_collision_31");
    pinocchio::FrameIndex r41 = model_.getFrameId("right_collision_41");
    pinocchio::FrameIndex r51 = model_.getFrameId("right_collision_51");
    pinocchio::FrameIndex r61 = model_.getFrameId("right_collision_61");
    pinocchio::computeJointJacobians(model_, data_, nowq);

    Eigen::Vector3d pos_b1 = data_.oMf[b1].translation();
    Eigen::Vector3d pos_b2 = data_.oMf[b2].translation();
    Eigen::Vector3d pos_b3 = data_.oMf[b3].translation();
    Eigen::Vector3d pos_b4 = data_.oMf[b4].translation();

    Eigen::Vector3d pos_l21 = data_.oMf[l21].translation();
    Eigen::Vector3d pos_l22 = data_.oMf[l22].translation();
    Eigen::Vector3d pos_l31 = data_.oMf[l31].translation();
    Eigen::Vector3d pos_l41 = data_.oMf[l41].translation();
    Eigen::Vector3d pos_l51 = data_.oMf[l51].translation();
    Eigen::Vector3d pos_l61 = data_.oMf[l61].translation();

    Eigen::Vector3d pos_r21 = data_.oMf[r21].translation();
    Eigen::Vector3d pos_r22 = data_.oMf[r22].translation();
    Eigen::Vector3d pos_r31 = data_.oMf[r31].translation();
    Eigen::Vector3d pos_r41 = data_.oMf[r41].translation();
    Eigen::Vector3d pos_r51 = data_.oMf[r51].translation();
    Eigen::Vector3d pos_r61 = data_.oMf[r61].translation();

    // Eigen::MatrixXd jacobian_b1 = pinocchio::getFrameJacobian(model_, data_, b1, pinocchio::LOCAL_WORLD_ALIGNED);
    // Eigen::MatrixXd jacobian_b2 = pinocchio::getFrameJacobian(model_, data_, b2, pinocchio::LOCAL_WORLD_ALIGNED);
    // Eigen::MatrixXd jacobian_b3 = pinocchio::getFrameJacobian(model_, data_, b3, pinocchio::LOCAL_WORLD_ALIGNED);
    // Eigen::MatrixXd jacobian_b4 = pinocchio::getFrameJacobian(model_, data_, b4, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::MatrixXd jacobian_l21 = pinocchio::getFrameJacobian(model_, data_, l21, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_l22 = pinocchio::getFrameJacobian(model_, data_, l22, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_l31 = pinocchio::getFrameJacobian(model_, data_, l31, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_l41 = pinocchio::getFrameJacobian(model_, data_, l41, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_l51 = pinocchio::getFrameJacobian(model_, data_, l51, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_l61 = pinocchio::getFrameJacobian(model_, data_, l61, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::MatrixXd jacobian_r21 = pinocchio::getFrameJacobian(model_, data_, r21, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r22 = pinocchio::getFrameJacobian(model_, data_, r22, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r31 = pinocchio::getFrameJacobian(model_, data_, r31, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r41 = pinocchio::getFrameJacobian(model_, data_, r41, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r51 = pinocchio::getFrameJacobian(model_, data_, r51, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r61 = pinocchio::getFrameJacobian(model_, data_, r61, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::MatrixXd jac_l21 = jacobian_l21.block<3,17>(0,0);
    Eigen::MatrixXd jac_l22 = jacobian_l22.block<3,17>(0,0);
    Eigen::MatrixXd jac_l31 = jacobian_l31.block<3,17>(0,0);
    Eigen::MatrixXd jac_l41 = jacobian_l41.block<3,17>(0,0);
    Eigen::MatrixXd jac_l51 = jacobian_l51.block<3,17>(0,0);
    Eigen::MatrixXd jac_l61 = jacobian_l61.block<3,17>(0,0);

    Eigen::MatrixXd jac_r21 = jacobian_r21.block<3,17>(0,0);
    Eigen::MatrixXd jac_r22 = jacobian_r22.block<3,17>(0,0);
    Eigen::MatrixXd jac_r31 = jacobian_r31.block<3,17>(0,0);
    Eigen::MatrixXd jac_r41 = jacobian_r41.block<3,17>(0,0);
    Eigen::MatrixXd jac_r51 = jacobian_r51.block<3,17>(0,0);
    Eigen::MatrixXd jac_r61 = jacobian_r61.block<3,17>(0,0);
    
    (-(pos_b1 - pos_l61)/(pos_b1 - pos_l61).norm()).transpose()*jac_l61;
    (-(pos_b2 - pos_l61)/(pos_b2 - pos_l61).norm()).transpose()*jac_l61;
    (-(pos_b3 - pos_l61)/(pos_b3 - pos_l61).norm()).transpose()*jac_l61;
    (-(pos_b4 - pos_l61)/(pos_b4 - pos_l61).norm()).transpose()*jac_l61;

    (-(pos_b1 - pos_r61)/(pos_b1 - pos_r61).norm()).transpose()*jac_r61;
    (-(pos_b2 - pos_r61)/(pos_b2 - pos_r61).norm()).transpose()*jac_r61;
    (-(pos_b3 - pos_r61)/(pos_b3 - pos_r61).norm()).transpose()*jac_r61;
    (-(pos_b4 - pos_r61)/(pos_b4 - pos_r61).norm()).transpose()*jac_r61;


    (-(pos_l61 - pos_r61)/(pos_l61 - pos_r61).norm()).transpose()*jac_r61;
    (-(pos_r61 - pos_l61)/(pos_r61 - pos_l61).norm()).transpose()*jac_l61;


    Eigen::VectorXd lp = Eigen::VectorXd::Zero(9);
    lp(0) = ((pos_b1 - pos_l61).norm() -(collision_r(0)+collision_r(9)+ safe_d));
    lp(1) = ((pos_b2 - pos_l61).norm() -(collision_r(1)+collision_r(9)+ safe_d));
    lp(2) = ((pos_b3 - pos_l61).norm() -(collision_r(2)+collision_r(9)+ safe_d));
    lp(3) = ((pos_b4 - pos_l61).norm() -(collision_r(3)+collision_r(9)+ safe_d));



    lp(4) = ((pos_b1 - pos_r61).norm() -(collision_r(0)+collision_r(15)+ safe_d));
    lp(5) = ((pos_b2 - pos_r61).norm() -(collision_r(1)+collision_r(15)+ safe_d));
    lp(6) = ((pos_b3 - pos_r61).norm() -(collision_r(2)+collision_r(15)+ safe_d));
    lp(7) = ((pos_b4 - pos_r61).norm() -(collision_r(3)+collision_r(15)+ safe_d));

    lp(8) = ((pos_l61 - pos_r61).norm() -(collision_r(15)+collision_r(9)+ safe_d));
    // lp(9) = ((pos_r61 - pos_l61).norm() -(collision_r(15)+collision_r(9)+ safe_d));



    Eigen::MatrixXd temp_conA = Eigen::MatrixXd::Zero(9,17);
    temp_conA.block<1,17>(0,0) = (-(pos_b1 - pos_l61)/(pos_b1 - pos_l61).norm()).transpose()*jac_l61;
    temp_conA.block<1,17>(1,0) = (-(pos_b2 - pos_l61)/(pos_b2 - pos_l61).norm()).transpose()*jac_l61;
    temp_conA.block<1,17>(2,0) = (-(pos_b3 - pos_l61)/(pos_b3 - pos_l61).norm()).transpose()*jac_l61;
    temp_conA.block<1,17>(3,0) = (-(pos_b4 - pos_l61)/(pos_b4 - pos_l61).norm()).transpose()*jac_l61;

    temp_conA.block<1,17>(4,0) = (-(pos_b1 - pos_r61)/(pos_b1 - pos_r61).norm()).transpose()*jac_r61;
    temp_conA.block<1,17>(5,0) = (-(pos_b2 - pos_r61)/(pos_b2 - pos_r61).norm()).transpose()*jac_r61;
    temp_conA.block<1,17>(6,0) = (-(pos_b3 - pos_r61)/(pos_b3 - pos_r61).norm()).transpose()*jac_r61;
    temp_conA.block<1,17>(7,0) = (-(pos_b4 - pos_r61)/(pos_b4 - pos_r61).norm()).transpose()*jac_r61;


    temp_conA.block<1,17>(8,0) = (-(pos_l61 - pos_r61)/(pos_l61 - pos_r61).norm()).transpose()*jac_r61;
    // temp_conA.block<1,17>(9,0) = 0.5*(-(pos_r61 - pos_l61)/(pos_r61 - pos_l61).norm()).transpose()*jac_l61;

    #include <cmath>
    // Eigen::VectorXd granCollision =Eigen::VectorXd::Zero(17);
    Eigen::VectorXd weight =Eigen::VectorXd::Zero(9);
    for (int i = 0; i < 9; i++)
    {
        if (abs(lp(i)) < 1.2 * safe_d)
        {
            weight(i) = exp(0.00002*lp(i) + 0.000005*safe_d);
            granCollision = granCollision - weight(i) * (temp_conA.row(i)).transpose() ;
        }

    }
    if((lp.array().abs() > 1.2 * safe_d).all())
    {
        granCollision =Eigen::VectorXd::Zero(17);
    }
    std::cout<<granCollision.transpose()<<std::endl;
    //====先试试末端TODO
    // m_testlp = lp + 0.1*(lp-m_testlp);
    m_testlp = lp;

    m_test_conA = temp_conA;
    // std::cout<<"===dis====="<<(pos_b2 - pos_l61).norm() -collision_r(2) -collision_r(9)<<std::endl;
    // std::cout<<"====="<< temp_conA.block<1,17>(0,0) * nowq - lp(0)<<";"<<
    //             temp_conA.block<1,17>(1,0) * nowq - lp(1)<<";"<<
    //            temp_conA.block<1,17>(2,0) * nowq - lp(2)<<";"<< 
    //            temp_conA.block<1,17>(3,0) * nowq - lp(3)<<";"<<std::endl;

    // std::cout<<"====="<<pos_l61.transpose()<<std::endl;

    // std::cout<<"==========jacobian_l21============="<<std::endl;
    // std::cout<<jacobian_l21<<std::endl;
    // std::cout<<"==========jacobian_l22============="<<std::endl;
    // std::cout<<jacobian_l22<<std::endl;
    // std::cout<<"==========jacobian_l31============="<<std::endl;
    // std::cout<<jacobian_l31<<std::endl;
    // std::cout<<"==========jacobian_l41============="<<std::endl;
    // std::cout<<jacobian_l41<<std::endl;
    // std::cout<<"==========jacobian_l51============="<<std::endl;
    // std::cout<<jacobian_l51<<std::endl;
    // std::cout<<"==========jacobian_l61============="<<std::endl;
    // std::cout<<jacobian_l61<<std::endl;
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
Eigen::VectorXd ControlSystem::cartesianImpedance(Eigen::VectorXd& carErr,Eigen::MatrixXd& joacb,Eigen::VectorXd& dynCompensated,Eigen::VectorXd& qd)
{
    return  dynCompensated + joacb.transpose() * (impedance_Cartesian_K * carErr - impedance_Cartesian_B*joacb*qd);
}

Eigen::VectorXd ControlSystem::computeTorqueWithSlidingMode(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                            const Eigen::VectorXd& q_desired, const Eigen::VectorXd& v_desired,
                                                            const Eigen::VectorXd& a_desired) 
{

    // std::cout<<"====debug  1====="<<std::endl;
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(15); 
    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(15);
    Eigen::VectorXd a_desired_full = Eigen::VectorXd::Zero(15);

    // std::cout<<"====debug  2====="<<std::endl;


    // sensor_msgs::JointState joint_state_msg;
    // joint_state_msg.name.resize(15);
    // joint_state_msg.position.resize(15);
    // joint_state_msg.velocity.resize(15);
    // joint_state_msg.name = 
    // {
    //    "waist_yaw_joint", "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    //     "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
    //     "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
    //     "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint"
    // };
    // joint_state_msg.header.stamp = ros::Time::now();
    // for (int i = 0; i < 15; i++)
    // {
    //     joint_state_msg.position[i]=q(i);
    // }
    // joint_pub.publish(joint_state_msg);
    // std::cout<<"====debug  3====="<<std::endl;

    q_full.head(15) = q;  // 前7个关节位置
    v_full.head(15) = v;  // 前7个关节速度
    // a_desired_full.head(7) = a_desired;  // 前7个关节加速度

    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(15, 15) * test_lambda; 
    double eta = test_eta;  // 趋近率增益
    double k = 0.0015;   // 滑模控制增益

    Eigen::VectorXd e = -(q - q_desired);
    //=====================================================
        sensor_msgs::JointState joint_state;
        joint_state.name.resize(15); 
        joint_state.position.resize(15);
        joint_state.velocity.resize(15);
        joint_state.effort.resize(15);
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14"};// 替换为你的关节名称
        for (int i = 0; i < 8; ++i) 
        { 
            joint_state.position[i] = e(i);
        }
        jointErrorPub.publish(joint_state);
    //=====================================================


    // std::cout<<"====debug  4====="<<std::endl;

    Eigen::VectorXd e_dot = -(v - v_desired);
    Eigen::VectorXd s = (e_dot + lambda * e);
    // std::cout<<"====debug  4.1====="<<std::endl;

    // 滑模面导数项
    Eigen::VectorXd s_dot = a_desired + lambda * e_dot;
    // std::cout<<"====debug  4.2====="<<std::endl;

    auto dynTor = pinocchio::rnea(model_, data_, q_full,v_full,a_desired_full);  // 使用14维的q_full
    // std::cout<<"====debug  4.3====="<<std::endl;

    pinocchio::crba(model_,data_,q_full);
    // std::cout<<"====debug  4.4====="<<std::endl;

    pinocchio::computeCoriolisMatrix(model_, data_, q_full, v_full);  // 使用14维的q_full和v_full
    // std::cout<<"====debug  4.5====="<<std::endl;

    Eigen::MatrixXd C_full = data_.C;  // 计算完整的科里奥利矩阵
    // std::cout<<"====debug  4.6====="<<std::endl;

    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q_full);  // 计算完整的重力向量
    // std::cout<<"====debug  4.7====="<<std::endl;

    Eigen::MatrixXd M_full = data_.M;  // 计算完整的质量矩阵
    // std::cout<<"====debug  4.8====="<<std::endl;

    // Eigen::VectorXd tanhS = Eigen::VectorXd::Zero(8);
    // for (int i = 0; i < s.size(); ++i) {
    //     tanhS(i) = std::tanh(s(i)/test_eta); // 逐元素计算 tanh
    // }
    // s(1)=2*s(1);
    // tanhS(1) = 2*tanhS(1);
    Eigen::VectorXd tau = 
                        M_full * a_desired +
                         C_full * v_desired
                        // + G_full.head(7)+ test_k * s + test_c *tanhS; 
                        + 15*e + 0.5* e_dot
                        + G_full; 
    // test_lambda = config["controller"]["lambda_gain"].as<double>();
    // test_eta = config["controller"]["eta_gain"].as<double>();
    // std::cout<<"====debug  5====="<<std::endl;

    // 计算所有框架的雅可比矩阵
    pinocchio::forwardKinematics(model_,data_,q_full);
    pinocchio::computeJointJacobians(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
    // std::cout<<"====debug  5.1====="<<std::endl;

    // 获取框架 ID
    pinocchio::FrameIndex frame_id_left = model_.getFrameId("left_flange");
    pinocchio::FrameIndex frame_id_right = model_.getFrameId("right_flange");

    pinocchio::FrameIndex frame_id_base = model_.getFrameId("base_link");


    Eigen::MatrixXd jacobian_l = pinocchio::getFrameJacobian(model_, data_, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r = pinocchio::getFrameJacobian(model_, data_, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED);
    // std::cout << "jacobian_r: " << std::endl;
    // std::cout << jacobian_r << std::endl;

    Eigen::MatrixXd jaco_left;
    jaco_left = jacobian_l.block<6,8>(0,0);

    Eigen::MatrixXd jaco_right;
    jaco_right = jacobian_r.block<6,7>(0,8);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jaco_left.transpose());
    const Eigen::MatrixXd temp_jacoLeftT = jaco_left.transpose();
    Eigen::MatrixXd jaco_pseudo_inv = cod.pseudoInverse();
    // Eigen::MatrixXd jaco_pseudo_inv = pseudoInverse(temp_jacoLeftT);
    Eigen::MatrixXd null_space_projection = Eigen::MatrixXd::Identity(8, 8) - jaco_left.transpose()*jaco_pseudo_inv;
    Eigen::VectorXd null = Eigen::VectorXd::Zero(8);
    // std::cout<<"====debug  5.2====="<<std::endl;

    pinocchio::forwardKinematics(model_, data_, q_full);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Vector3d pos_l = data_.oMf[22].translation();
    Eigen::Vector3d pos_r = data_.oMf[74].translation();
    Eigen::Quaterniond quaternion(data_.oMf[22].rotation());
    // std::cout << "Position: " << data_.oMf[22].translation().transpose() << std::endl;
    // std::cout << "Rotation (Quaternion): " << quaternion.coeffs().transpose() << std::endl;
    double time =ros::Time::now().toSec(); // 当前时间;
    // std::cout<<"====debug  5.3====="<<std::endl;

    Eigen::Quaterniond desired_quaternion;
    desired_quaternion.x()=0.0937198;
    desired_quaternion.y()=-0.0698212;
    desired_quaternion.z()=0.687292;
    desired_quaternion.w()=0.716918;

    Eigen::Quaterniond rot_now; 
    rot_now = quaternion;
    Eigen::Quaterniond e_quaternion; 
    e_quaternion = rot_now.inverse() * desired_quaternion;
    Eigen::Matrix3d r_now = rot_now.toRotationMatrix(); 
    Eigen::Matrix3d r_e = e_quaternion.toRotationMatrix();  
 
    Eigen::AngleAxisd angle_axis(r_e);
    Eigen::Vector3d temp_voir;
    temp_voir = angle_axis.angle() * angle_axis.axis();
        // Eigen::Matrix3d 
    Eigen::Vector3d temp_Rsin;

    temp_Rsin(0)=temp_voir(0);
    temp_Rsin(1)=temp_voir(1);
    temp_Rsin(2)=temp_voir(2)+1*sin(0.5*time);
    temp_Rsin = r_now*temp_Rsin;
    // std::cout<<"====debug  5.4====="<<std::endl;

    Eigen::VectorXd max(8);
    Eigen::VectorXd min(8);
    max<<0.5,2.96,3.4,2.96,0,2.96,1.04,1.48;
    min<<-0.5,-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;

    Eigen::VectorXd mid = Eigen::VectorXd::Zero(8);
    mid(0)= 0 ;
    mid = (max+min)/2;
    for (int i = 1; i < 8; i++)
    {
        mid(i)=mid(i)+1.2*sin(time);
    }
    // std::cout<<"====debug  5.5====="<<std::endl;
    // 0.322600,-0.25,0.05
    Eigen::VectorXd carErr_right = Eigen::VectorXd::Zero(6); 
    carErr_right(0)=0.322600-pos_r(0);
    carErr_right(1)=-0.35-pos_r(1);
    carErr_right(2)=0.05-pos_r(2);
    carErr_right(3)=0;
    carErr_right(4)=0;
    carErr_right(5)=0;

    Eigen::VectorXd carErr_left = Eigen::VectorXd::Zero(6); 
    carErr_left(0)=0.341688-pos_l(0);
    carErr_left(1)=0.399841-pos_l(1)+0.1*sin(time);
    carErr_left(2)=0.057347-pos_l(2)+0.1*cos(time);
    carErr_left(3)=temp_Rsin(0);
    carErr_left(4)=temp_Rsin(1);
    carErr_left(5)=temp_Rsin(2);
    // std::cout<<"====debug  5.6====="<<std::endl;
    const Eigen::MatrixXd ocJac = jaco_left;
    // std::cout<<"====debug  5.61====="<<std::endl;
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_a(ocJac);
    // std::cout<<"====debug  5.62====="<<std::endl;
    Eigen::MatrixXd jaco_pseudo_T_inv = cod_a.pseudoInverse();
    const Eigen::VectorXd qmain = jaco_pseudo_T_inv * carErr_left;
    // std::cout<<"====debug  5.7====="<<std::endl;
    auto result = optimizeNullSpaceVelocity(ocJac, qmain,1.0,1.0);
    // std::cout<<result.transpose()<<std::endl;
    Eigen::VectorXd left_tau = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd right_tau = Eigen::VectorXd::Zero(7);

    if (sim>2500)
    {
      null = test_lambda*(result-q.head(8)) + 0.1*(v_desired.head(8)-v.head(8));
      Eigen::VectorXd temp_l = dynTor.head(8);
      Eigen::VectorXd tempv_l = v.head(8);
      Eigen::VectorXd temp_r = dynTor.tail(7);
      Eigen::VectorXd tempv_r = v.tail(7);
      left_tau = cartesianImpedance(carErr_left,jaco_left,temp_l,tempv_l);
      right_tau = cartesianImpedance(carErr_right,jaco_right,temp_r,tempv_r);
      //null= null_space_projection*null;
      left_tau=null+left_tau;
      tau.head(8)=left_tau;
      tau.tail(7)=right_tau;
    }
    
    sim++;
    last_tau = tau;
    // std::cout<<tau.transpose()<<std::endl;
    return 0.5*tau+last_tau*0.5;
}
Eigen::VectorXd ControlSystem::tra(Eigen::VectorXd& nowq,Eigen::VectorXd& nowqv)
{
    
    // if (allarm_done)
    // {
    //    return Eigen::VectorXd::Zero(17);
    // }
    

//    lr_Xr = dualArmSys(nowX,l_waypoints,r_waypoints);
   auto wqpResult = interfaceWQP(nowq,nowqv,lr_Xr);
   return wqpResult;


}

Eigen::VectorXd ControlSystem::dualArmSys(Eigen::VectorXd& nowX,
                               std::vector<Eigen::VectorXd>& l_waypoints,
                               std::vector<Eigen::VectorXd>& r_waypoints)
 {
    Eigen::VectorXd l_current_position = nowX.head(7);  // 左臂当前位姿
    Eigen::VectorXd r_current_position = nowX.tail(7);  // 右臂当前位姿
    
    Eigen::VectorXd desired_position(14);  // 14维期望位姿（7维左臂，7维右臂）
    double temp_tolerance = 0.01;  // 误差容忍度

    auto current_time = std::chrono::steady_clock::now();  // 当前时间
    // std::cout<<carE.transpose()<<std::endl;

    // 处理左臂的目标
    if (!l_waypoints.empty() && !left_done) {
        Eigen::VectorXd l_target = l_waypoints.front();
        double l_error = (l_current_position - l_target).norm();

        if (l_waypoints.size() == 1) {
            temp_tolerance = tolerance_last;
        } else {
            temp_tolerance = tolerance_first;
        }

        // 记录左臂目标点的时间
        if (waypoint_times_l.find(current_waypoint_index_l) == waypoint_times_l.end()) {
            waypoint_times_l[current_waypoint_index_l] = current_time;  // 第一个目标点的起始时间
        }

        // 计算目标点与当前时间的差值
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - waypoint_times_l[current_waypoint_index_l]).count();
        
        if (elapsed_time > 7) {
            std::cout << "Left arm waypoint " << current_waypoint_index_l + 1 << " took too long (" << elapsed_time << " seconds), skipping to next." << std::endl;
            
            if (l_waypoints.size() > 1) {
                l_waypoints.erase(l_waypoints.begin());  // 删除当前目标点
                current_waypoint_index_l++;  // 更新目标点索引
                waypoint_times_l[current_waypoint_index_l] = current_time;  // 记录下一个目标点的起始时间
            } else {
                std::cout << "Unable to reach the last left arm waypoint within time limit." << std::endl;
                left_done = true;
            }
        }

        // 判断左臂是否到达目标位置
        if (carE.head(6).cwiseAbs().maxCoeff() < temp_tolerance) {
            std::cout << "Left arm waypoint completed with error: " << carE.head(6).cwiseAbs().maxCoeff() << std::endl;
            
            if (!l_waypoints.empty()) {
                l_waypoints.erase(l_waypoints.begin());  // 删除完成的目标点
                current_waypoint_index_l++;  // 更新目标点索引
                waypoint_times_l[current_waypoint_index_l] = current_time;  // 记录下一个目标点的起始时间
            }
        }

        if (!l_waypoints.empty()) {
            desired_position.head(7) = l_waypoints.front();  // 更新左臂期望位置
        } else {
            desired_position.head(7) = l_current_position;  // 如果左臂的目标已完成所有点，直接将当前位姿作为期望位姿
            left_done = true;  // 左臂完成所有目标点
        }
    } else {
        desired_position.head(7) = l_current_position;  // 如果左臂目标点已经完成所有，直接将当前位姿赋给期望位姿
        left_done = true;
    }

    // 处理右臂的目标
    if (!r_waypoints.empty() && !right_done) {
        Eigen::VectorXd r_target = r_waypoints.front();
        // double r_error = (r_current_position - r_target).norm();

        if (r_waypoints.size() == 1) {
            temp_tolerance = tolerance_last;
        } else {
            temp_tolerance = tolerance_first;
        }

        // 记录右臂目标点的时间
        if (waypoint_times_r.find(current_waypoint_index_r) == waypoint_times_r.end()) {
            waypoint_times_r[current_waypoint_index_r] = current_time;  // 第一个目标点的起始时间
        }

        // 计算目标点与当前时间的差值
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - waypoint_times_r[current_waypoint_index_r]).count();
        
        if (elapsed_time > 7) {
            std::cout << "Right arm waypoint " << current_waypoint_index_r + 1 << " took too long (" << elapsed_time << " seconds), skipping to next." << std::endl;
            
            if (r_waypoints.size() > 1) {
                r_waypoints.erase(r_waypoints.begin());  // 删除当前目标点
                current_waypoint_index_r++;  // 更新目标点索引
                waypoint_times_r[current_waypoint_index_r] = current_time;  // 记录下一个目标点的起始时间
            } else {
                std::cout << "Unable to reach the last right arm waypoint within time limit." << std::endl;
                right_done = true;
            }
        }

        // 判断右臂是否到达目标位置
        if (carE.tail(6).cwiseAbs().maxCoeff() < temp_tolerance) {
            std::cout << "Right arm waypoint completed with error: " << carE.tail(6).cwiseAbs().maxCoeff() << std::endl;
            
            if (!r_waypoints.empty()) {
                r_waypoints.erase(r_waypoints.begin());  // 删除完成的目标点
                current_waypoint_index_r++;  // 更新目标点索引
                waypoint_times_r[current_waypoint_index_r] = current_time;  // 记录下一个目标点的起始时间
            }
        }

        if (!r_waypoints.empty()) {
            desired_position.tail(7) = r_waypoints.front();  // 更新右臂期望位置
        } else {
            desired_position.tail(7) = r_current_position;  // 如果右臂的目标已完成所有点，直接将当前位姿作为期望位姿
            std::cout<<"==right?=="<<desired_position.tail(6).transpose()<<std::endl;
            right_done = true;  // 右臂完成所有目标点
        }
    }

    // 如果左右臂之一完成所有目标点，返回期望位姿
    if (left_done && right_done) {
        allarm_done = true;
        std::cout << "Dual arm has completed all waypoints." << std::endl;
        waypoint_times_l.clear();  // 清空左臂目标点时间记录
        waypoint_times_r.clear();  // 清空右臂目标点时间记录
    }

    return desired_position;  // 返回更新后的期望位姿
}                              

Eigen::VectorXd ControlSystem::interfaceWQP(Eigen::VectorXd& nowq,Eigen::VectorXd& nowqv,Eigen::VectorXd& Rcar)
{
    // if (allarm_done)
    // {
    //     Eigen::VectorXd tempRes = Eigen::VectorXd::Zero(17);
    //     return tempRes;
    // }
    
    // std::cout<<"=====now_q======"<<nowq.transpose()<<std::endl;

    double time =ros::Time::now().toSec(); // 当前时间;
    pinocchio::computeJointJacobians(model_, data_, nowq);

    pinocchio::forwardKinematics(model_, data_, nowq);
    pinocchio::updateFramePlacements(model_, data_);
    // step1 获取lr 雅可比
    // pinocchio::FrameIndex frame_id_left = model_.getFrameId("left_flange");
    // pinocchio::FrameIndex frame_id_right = model_.getFrameId("right_flange");
    pinocchio::FrameIndex frame_id_left = model_.getFrameId("L_hand_base_link");
    pinocchio::FrameIndex frame_id_right = model_.getFrameId("R_hand_base_link");
    // std::cout<<"FrameIndex:"<<frame_id_left<<";"<<frame_id_right<<std::endl;
    Eigen::MatrixXd jacobian_l = pinocchio::getFrameJacobian(model_, data_, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r = pinocchio::getFrameJacobian(model_, data_, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED);
    // auto j_c = data_.Jcom;
    // auto c_p = data_.com[0];
    // std::cout<<"==c_p=="<<c_p.transpose()<<std::endl;
    // auto c_p = j_c * 
    // step2 获取笛卡尔err

    Eigen::Vector3d pos_l = data_.oMf[frame_id_left].translation();
    Eigen::Vector3d pos_r = data_.oMf[frame_id_right].translation();
    Eigen::Quaterniond quat_l(data_.oMf[frame_id_left].rotation());
    Eigen::Quaterniond quat_r(data_.oMf[frame_id_right].rotation());
    pos_l(2) = pos_l(2) - 0.5;//base offset
    pos_r(2) = pos_r(2) - 0.5;
// 6D-left:0.445156;0.287187;0.215173;-0.0345472;-0.232099;0.673005;0.701428
// 6D-right:0.422576;-0.303942;0.171599;0.701961;-0.684411;0.196978;0.00561809
    // std::cout<<"6D-left:"<<pos_l(0)<<";"
    //          <<pos_l(1)<<";"
    //          <<pos_l(2)<<";"
    //          <<quat_l.x()<<";"
    //          <<quat_l.y()<<";"
    //          <<quat_l.z()<<";"
    //          <<quat_l.w()<<std::endl;

    // std::cout<<"6D-right:"<<pos_r(0)<<";"
    //         <<pos_r(1)<<";"
    //         <<pos_r(2)<<";"
    //         <<quat_r.x()<<";"
    //         <<quat_r.y()<<";"
    //         <<quat_r.z()<<";"
    //         <<quat_r.w()<<std::endl;


    // Eigen::Vector3d pos_l_d(0.445156,0.287187,0.215173);
    // Eigen::Quaterniond quat_l_d;
    // quat_l_d.x()=-0.0345472;
    // quat_l_d.y()=-0.232099;
    // quat_l_d.z()=0.673005;
    // quat_l_d.w()=0.701428;
    // // car sin sign test 
    // // double a = -0.303942+0.08*sin(0.5*time);
    // // double b = 0.171599+0.08*sin(0.5*time);
    
    // Eigen::Vector3d pos_r_d(0.422576,-0.303942,0.171599);
    // Eigen::Quaterniond quat_r_d;
    // quat_r_d.x()=0.701961;
    // quat_r_d.y()=-0.684411;
    // quat_r_d.z()=0.196978;
    // quat_r_d.w()=0.00561809;
    // ========================================================================Rcar
    Eigen::Vector3d pos_l_d;
    pos_l_d(0)=Rcar(0);
    pos_l_d(1)=Rcar(1);
    pos_l_d(2)=Rcar(2);

    Eigen::Quaterniond quat_l_d;
    quat_l_d.x()=Rcar(3);
    quat_l_d.y()=Rcar(4);
    quat_l_d.z()=Rcar(5);
    quat_l_d.w()=Rcar(6);

    Eigen::Vector3d pos_r_d;
    pos_r_d(0)=Rcar(0+7);
    pos_r_d(1)=Rcar(1+7);
    pos_r_d(2)=Rcar(2+7);

    Eigen::Quaterniond quat_r_d;
    quat_r_d.x()=Rcar(3+7);
    quat_r_d.y()=Rcar(4+7);
    quat_r_d.z()=Rcar(5+7);
    quat_r_d.w()=Rcar(6+7);


    // Eigen::Vector3d pos_l_d(0.378,0.250,0.38);
    // Eigen::Quaterniond quat_l_d;
    // quat_l_d.x()=0.481355;
    // quat_l_d.y()=-0.287633;
    // quat_l_d.z()=0.164375;
    // quat_l_d.w()=0.811508;

    pos_r_d<<0.378,-0.188,0.38;
    quat_r_d.x()=0.80853;
    quat_r_d.y()=-0.162;
    quat_r_d.z()=0.29277;
    quat_r_d.w()=0.48402;

    
    // // car sin sign test 
    // // double a = -0.303942+0.08*sin(0.5*time);
    // // double b = 0.171599+0.08*sin(0.5*time);

    // if (sim>8000)
    // {
    //     pos_l_d(0) = 0.8;
    //     pos_l_d(1) = 0.42;
    //     pos_l_d(2) = pos_l_d(2)+0.22;

    //     pos_r_d(2) = pos_r_d(2)+0.22;
    //     Eigen::Quaterniond quat_test;
    //     quat_test.x()=0;
    //     quat_test.y()=1.682;
    //     quat_test.z()=-0.75;
    //     quat_test.w()=0.732;
    //     quat_r_d = quat_r_d*quat_test;
    //     /* code */
    // }
    
    
    Eigen::Quaterniond e_quat_l; 
    e_quat_l = quat_l.inverse() * quat_l_d;
    Eigen::Matrix3d mat_l_now = quat_l.toRotationMatrix(); 
    Eigen::Matrix3d mat_l_e = e_quat_l.toRotationMatrix();  
    Eigen::AngleAxisd angle_axis_l(mat_l_e);
    Eigen::Vector3d temp_rotError_l;
    temp_rotError_l = angle_axis_l.angle() * angle_axis_l.axis();
    temp_rotError_l = mat_l_now * temp_rotError_l;

    Eigen::Quaterniond e_quat_r; 
    e_quat_r = quat_r.inverse() * quat_r_d;
    Eigen::Matrix3d mat_r_now = quat_r.toRotationMatrix(); 
    Eigen::Matrix3d mat_r_e = e_quat_r.toRotationMatrix();  
    Eigen::AngleAxisd angle_axis_r(mat_r_e);
    Eigen::Vector3d temp_rotError_r;
    temp_rotError_r = angle_axis_r.angle() * angle_axis_r.axis();
    temp_rotError_r = mat_r_now * temp_rotError_r;
    Eigen::VectorXd carErr_r = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd carErr_l = Eigen::VectorXd::Zero(6);
    carErr_l.head(3) = (pos_l_d - pos_l).normalized()*10.0;
    carErr_l.head(3) = pos_l_d - pos_l;
    carErr_l.tail(3) = temp_rotError_l;
    carErr_r.head(3) = pos_r_d - pos_r;
    carErr_r.tail(3) = temp_rotError_r;

    // carErr_l(1)+=0.3*sin(2.3*time);
    // carErr_l(2)+=0.3*cos(2.3*time);
    // // carErr_l(4)+=0.15*cos(2*time);

    // carErr_r(1)+=0.3*sin(2.6*time);
    // carErr_r(2)+=0.3*cos(2.6*time);
    // carErr_r(4)+=0.15*cos(2*time);
    carE<<carErr_l,carErr_r;

    Eigen::VectorXd tempZero = Eigen::VectorXd::Zero(6);
    // carE.tail(6) = tempZero;
    // if (left_done)
    // {
    //     carE.head(6) = tempZero;
    // }
    // if (right_done)
    // {
    //    carE.tail(6) = tempZero;
    // }
    
    

    Eigen::VectorXd Qr = Eigen::VectorXd::Zero(17);
    // nowq.head(3) = Eigen::VectorXd::Zero(3);
    // Qr(6-1)= (20-30*sin(3*time))*3.14/180 - nowq(6-1);
    // Qr(13-1)= (20+30*sin(3*time))*3.14/180 - nowq(13-1);
    // Qr(12) = 45 *3.14/180 - nowq(12);
    // Qr(5) = -45 *3.14/180 - nowq(6-1);
                //     desired_position(3) =  -1.95;
            //     desired_position(4) =  0.61;
            //     desired_position(5) =  0.63;
            //     desired_position(6) =  -1.57;
            //     desired_position(7) =  0.8;
            //     desired_position(8) =  -0.33;
            //     desired_position(9) =  0;
    // Qr(3) =  -1.95 - nowq(3);   
    // Qr(4) =  0.61- nowq(4);  
    // Qr(5) =  0.63- nowq(5);  
    // Qr(6) =  -1.57- nowq(6);  
    // Qr(7) =  0.8 - nowq(7);  
    // Qr(8) =  - 0.33- nowq(8);  
    // Qr(9) =  0 - nowq(9);  



    
    // std::cout<<"=======wqp1.1========="<<std::endl;
    jacobian_l.block<6,3>(0,0)=Eigen::MatrixXd::Zero(6,3);
    jacobian_r.block<6,3>(0,0)=Eigen::MatrixXd::Zero(6,3);
    upDataCollision(nowq);
    auto result = WQP(jacobian_l,
                        jacobian_r,
                        carE,
                        nowq,
                        Qr);

                        // T*T
    // std::cout<<result.transpose()<<std::endl;
    // result.head(3) = Eigen::VectorXd::Zero(3);
    return result;
}


Eigen::VectorXd ControlSystem::optimizeNullSpaceVelocity(const Eigen::MatrixXd& J,      // 雅可比矩阵
                                            const Eigen::VectorXd& q_main,              // 主任务关节角速度增量
                                            double alpha,                               // 雅可比条件数最小化的权重
                                            double beta)                                // 关节角速度增量和最小化的权重

{
    int n = J.rows();                       
    int m = n;   
    Eigen::MatrixXd JTJ = J.transpose() * J;
    double determinant = JTJ.determinant();

    // std::cout << "The determinant of J^T * J is: " << determinant << std::endl;
                        
    Eigen::MatrixXd H = 0.001 * J.transpose() * J + 1 * Eigen::MatrixXd::Identity(8, 8);
    Eigen::VectorXd c = 1 * q_main;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(14,8);
    A.block<6,8>(0,0) = J;
    Eigen::MatrixXd b = Eigen::MatrixXd::Identity(8,8);
    A.block<8,8>(6,0) = b;
    Eigen::VectorXd max = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd min = Eigen::VectorXd::Zero(8);

    Eigen::VectorXd up = Eigen::VectorXd::Zero(14);
    Eigen::VectorXd lp = Eigen::VectorXd::Zero(14);
    max<<0.5,2.96,3.4,2.96,0,2.96,1.04,1.48;
    min<<-0.5,-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;
    // max = max *2/400;
    // min = min *2/400;
    up.segment(6, 8) = max;
    lp.segment(6, 8) = min;
    OsqpEigen::Solver solver;
    // solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    // solver.settings()->setAbsoluteTolerance(1e-6);
    // solver.settings()->setRelativeTolerance(1e-6);
    solver.data()->setNumberOfVariables(8);
    solver.data()->setNumberOfConstraints(14);
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    if (!solver.data()->setHessianMatrix(H_sparse)) {
        std::cerr << "Error setting Hessian matrix" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }

    if (!solver.data()->setGradient(c)) {
        std::cerr << "Error setting gradient vector" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        std::cerr << "Error setting constraint matrix" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }

    if (!solver.data()->setLowerBound(lp)) {
        std::cerr << "Error setting lower bounds" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }

    if (!solver.data()->setUpperBound(up)) {
        std::cerr << "Error setting upper bounds" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }

    if (!solver.initSolver()) {
        std::cerr << "Solver initialization failed" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }

    auto result = solver.solve();
    // if (result != OsqpEigen::ErrorExitFlag::NoError) {
    //     std::cerr << "Solver failed to find a solution. Error: " << result << std::endl;
    //     return Eigen::VectorXd::Zero(n);
    // }

    // 获取优化后的零空间角速度增量
    return solver.getSolution();
}
Eigen::VectorXd ControlSystem::WQP(const Eigen::MatrixXd& Jl,
                                const Eigen::MatrixXd& Jr,
                                const Eigen::VectorXd& car_err,
                                const Eigen::VectorXd& nowQ,
                                const Eigen::VectorXd& Qr)
    {

            // nowQ.head(3) = Eigen::VectorXd::Zero(3);
            for (int i = 0; i < 17; i++)
            {
                if (nowQ(i)<joint_min(i)||joint_max(i)<nowQ(i))
                {
                    // Eigen::VectorXd temp_q_max = 
                    std::cout<<"===out of q bound==  "<<i<<nowQ(i)<<std::endl;

                    return Eigen::VectorXd::Zero(17);
                        /* code */
                }
                
            }
            


            const int num = 17;
            // if (car0joint)
            // {
            //     double eta_car = 30000;
            //     double eta_qpos= 0.01;
            //     Qr.setConstant(0.0);
            // }
            // else
            // {
            //     double eta_car = 0;
            //     double eta_qpos= 30000;
            //     car_err.setConstant(0.0);
            // }
            
            double eta_car = 300000;
            double eta_qpos= 50;
            // double eta_car = 0;
            // double eta_qpos= 30000;
            Eigen::MatrixXd JointWeight = Eigen::MatrixXd::Identity(17, 17);
            JointWeight(0,0) = 300000;
            JointWeight(1,1) = 300000;
            JointWeight(2,2) = 300000;
            // auto boundH = upDataBoundH(nowQ);
            auto boundGrad = upDataBoundGradient(nowQ);

            Eigen::VectorXd leftError = car_err.head(6);
            leftError(3) = leftError(3) * 0.9;
            leftError(4) = leftError(4) * 0.9;
            leftError(5) = leftError(5) * 0.9;
            Eigen::VectorXd rightError = car_err.tail(6);
            rightError(3) = rightError(3) * 0.9;
            rightError(4) = rightError(4) * 0.9;
            rightError(5) = rightError(5) * 0.9;


            Eigen::MatrixXd H = eta_car * Jl.transpose()*Jl 
                            + eta_car * Jr.transpose()*Jr 
                            + eta_qpos * Eigen::MatrixXd::Identity(17, 17) + 0.0001*JointWeight;


            Eigen::VectorXd c =     -1*eta_car*Jl.transpose()*leftError
                                    -1*eta_car*Jr.transpose()*rightError
                                    -1*eta_qpos*Qr ;
                                    -10*boundGrad;
                                     10*granCollision;

            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(55,17);
            A.block<6,17>(0,0) = Jl;
            A.block<6,17>(6,0) = Jr;
            A.block<17,17>(12,0) = Eigen::MatrixXd::Identity(17, 17);
            A.block<17,17>(29,0) = Eigen::MatrixXd::Identity(17, 17);
            A.block<9,17>(46,0) = m_test_conA;
            // std::cout<<"=======wqp1.2========="<<std::endl;

            //==========约束1 笛卡尔线速度角速度
            Eigen::VectorXd Car_max = Eigen::VectorXd::Zero(12);
            Car_max.setConstant(0.1);

            Eigen::VectorXd Car_min = Eigen::VectorXd::Zero(12);
            Car_min.setConstant(-0.1);
            //==========约束2 关节速度

            Eigen::VectorXd qv_max = Eigen::VectorXd::Zero(17);
            qv_max.setConstant(3.14);
            qv_max(0) = 0.002*0.5 ;
            qv_max(1) = 0.002*0.5 ;
            qv_max(2) = 0.002*0.5 ;
            // qv_max(3) = 0.02*5 ;
            Eigen::VectorXd qv_min = Eigen::VectorXd::Zero(17);
            qv_min .setConstant(-3.14);
            qv_min(0) = -0.002*0.5 ;
            qv_min(1) = -0.002*0.5 ;
            qv_min(2) = -0.002*0.5 ;

            // △q ＜ qmax -qnow
            Eigen::VectorXd q_max = Eigen::VectorXd::Zero(17);
            Eigen::VectorXd q_min = Eigen::VectorXd::Zero(17);

            q_max<<0.0,0.15,0.25,
            0.79,2.27,2.27,0,2.27,0.8,0.8,
            3.14,0.17,2.27,2.27,2.27,0.8,0.8;
            q_min<<-0.1,-0.15,-0.25,
            -3.14,-0.17,-2.27,-2.27,-2.27,-0.8,-0.8,
            -0.79,-1.75,-2.27,0,-2.27,-0.8,-0.8;
            Eigen::VectorXd up = Eigen::VectorXd::Zero(55);
            Eigen::VectorXd lp = Eigen::VectorXd::Zero(55);


            Eigen::VectorXd car_temp_ = Eigen::VectorXd::Zero(6);
            car_temp_<< 2.05,2.05,2.05,
                        2.05,2.05,2.05;
            up.head(6) = 5*car_temp_ ;
            up.segment(6, 6) = 5*car_temp_;
            lp.head(6) = -5*car_temp_ ;
            lp.segment(6, 6) = -5*car_temp_;

            up.segment(12, 17) = qv_max;
            lp.segment(12, 17) = qv_min;

            up.segment(29, 17) = 0.0025*(q_max-nowQ);
            lp.segment(29, 17) = 0.0025*(q_min-nowQ);

            // up.segment(46, 17) = shoulderU;
            // lp.segment(46, 17) = shoulderL;
            Eigen::VectorXd epsilon = Eigen::VectorXd::Zero(9);
            epsilon.setConstant(10000);

            up.tail(9) =  epsilon;
            lp.tail(9) = -epsilon;

            lp.tail(9) = -m_testlp/2;


            OsqpEigen::Solver solver;

            int n = 17;
            solver.settings()->setWarmStart(false);
            solver.settings()->setVerbosity(false);
            solver.data()->setNumberOfVariables(17);
            solver.data()->setNumberOfConstraints(55);
            Eigen::SparseMatrix<double> H_sparse = H.sparseView();
            if (!solver.data()->setHessianMatrix(H_sparse)) 
            {
                std::cerr << "Error setting Hessian matrix" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            if (!solver.data()->setGradient(c)) {
                std::cerr << "Error setting gradient vector" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            Eigen::SparseMatrix<double> A_sparse = A.sparseView();
            if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
                std::cerr << "Error setting constraint matrix" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            if (!solver.data()->setLowerBound(lp)) {
                std::cerr << "Error setting lower bounds" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            if (!solver.data()->setUpperBound(up)) {
                std::cerr << "Error setting upper bounds" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            if (!solver.initSolver()) {
                std::cerr << "Solver initialization failed" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            auto result = solver.solve();
            if (!solver.solve())
            {
                std::cout<<"========="<<std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            auto tempRes = solver.getSolution();
            solver.clearSolver();

            return tempRes;
    }
void ControlSystem::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );
}

Eigen::VectorXd m_lr = Eigen::VectorXd::Zero(14);
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
     m_lr(0) = feedback->pose.position.x;
     m_lr(1) = feedback->pose.position.y;
     m_lr(2) = feedback->pose.position.z;
     m_lr(3) = feedback->pose.orientation.x ;
     m_lr(4) = feedback->pose.orientation.y ;
     m_lr(5) = feedback->pose.orientation.z ;
     m_lr(6) = feedback->pose.orientation.w ;

}
Eigen::MatrixXd ControlSystem:: pseudoInverse(const Eigen::MatrixXd &input)
{
  double lambda_ = 0.5;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = input; // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  return svd.matrixV() * S_.transpose() * svd.matrixU().transpose();
}
void ControlSystem::initInteractiveMarker()
{
    interactive_markers::InteractiveMarkerServer server("simple_marker");  
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "pelvis_link";
    int_marker.name = "my_marker";
    int_marker.description = "Simple 1-DOF Control";
    int_marker.scale = 0.4;

    int_marker.pose.position.x = 0.378; 
    int_marker.pose.position.y = 0.250;
    int_marker.pose.position.z = 0.38;

    int_marker.pose.orientation.x = 0.481355;
    int_marker.pose.orientation.y = -0.287633;
    int_marker.pose.orientation.z = 0.164375;
    int_marker.pose.orientation.w =0.811508;
  // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 0.3;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    //   control.scale = 0.5;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);
    server.applyChanges();

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_control_node");

    ControlSystem control_system("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model2/urdf/dual_arm_and_hand_description.urdf");

    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd a_desired = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd v_desired = Eigen::VectorXd::Zero(18);

    ros::Rate loop_rate(400);
    interactive_markers::InteractiveMarkerServer server("simple_marker");  
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "pelvis_link";
    int_marker.name = "my_marker";
    int_marker.description = "Simple 1-DOF Control";
    int_marker.scale = 0.4;

    int_marker.pose.position.x = 0.378; 
    int_marker.pose.position.y = 0.250;
    int_marker.pose.position.z = 0.38;

    int_marker.pose.orientation.x = 0.481355;
    int_marker.pose.orientation.y = -0.287633;
    int_marker.pose.orientation.z = 0.164375;
    int_marker.pose.orientation.w =0.811508;
  // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 0.3;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    //   control.scale = 0.5;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    // add the control to the interactive marker
    int_marker.controls.push_back(box_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker,&processFeedback);
//   server.insert(int_marker,&processFeedback, this, _1));

  // 'commit' changes and send to all clients
    server.applyChanges();
    control_system.initInteractiveMarker();


    while (ros::ok()) 
    {
        double time = ros::Time::now().toSec();

        Eigen::VectorXd desired_position = Eigen::VectorXd::Zero(17);
        Eigen::VectorXd now_q(17);
        Eigen::VectorXd now_qd(17);

        Eigen::VectorXd max(7);
        Eigen::VectorXd min(7);
        max<<2.96,3.4,2.96,0,2.96,1.04,1.48;
        min<<-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;

        Eigen::VectorXd mid;
        mid = (max+min)/2;

                desired_position(3) =  -111*3.14/180;
                desired_position(4) =  35* 3.14/180;;
                desired_position(5) =  36* 3.14/180;;
                desired_position(6) =  -90* 3.14/180;;
                desired_position(7) =  45* 3.14/180;;
                desired_position(8) =  -19* 3.14/180;;
                desired_position(9) =  0;

                desired_position(3+7) =  1.34;
                desired_position(4+7) =  0;
                desired_position(5+7) =  1.92;
                desired_position(6+7) =  1.23;
                desired_position(7+7) =  -1.75;
                desired_position(8+7) =  0;
                desired_position(9+7) =  0;
        // desired_position(3) =  0.0;
        // desired_position(4) =  145*3.1415926535/180;
        // desired_position(5) =  0;
        // desired_position(6) =  - 120*3.1415926535/180;
        // desired_position(7) =  0;
        // desired_position(8) =  - 40*3.1415926535/180;
        // desired_position(9) =  0;

        // desired_position(10) = 0.0;
        // desired_position(11) = -145*3.1415926535/180;
        // desired_position(12) = 0;
        // desired_position(13) = 120*3.1415926535/180;
        // desired_position(14) = 0;
        // desired_position(15) = 40*3.1415926535/180;
        // desired_position(16) = 0;



        // desired_position(3+11-1) = -3.1415926535/2; // 设置每个关节的期望位置为 0.5 * sin(time)
        for (int i = 0; i < 17; ++i) 
        {   


            now_q(i)=control_system.joint_state[i].position;
            now_qd(i)=control_system.joint_state[i].velocity;

        }
        // control_system.upDataCollision(now_q);

        Eigen::VectorXd xr = Eigen::VectorXd::Zero(14);
        
        control_system.lr_Xr = m_lr;
        auto wqpQ = control_system.tra(now_q,now_qd);
        wqpQ = wqpQ.normalized();
        // desired_position(3)= -mid(3)+0.5*sin(0.5*time);
        // 更新期望轨迹中的位置
        control_system.traj_data.addPosition(desired_position);

        // 获取期望的速度和加速度，返回值为 Eigen::VectorXd 类型
        Eigen::VectorXd desired_velocity = control_system.traj_data.getVelocities();
        Eigen::VectorXd desired_acceleration = control_system.traj_data.getAccelerations();
        sensor_msgs::JointState joint_state;
        joint_state.name.resize(17); 
        joint_state.position.resize(17);
        joint_state.velocity.resize(17);
        joint_state.effort.resize(17);
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = 
        {
            "waist_Z_joint",
            "waist_roll_joint",
            // "waist_pitch_joint",
            "waist_yaw_joint",
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6",
            "left_joint7",
            "right_joint1",
            "right_joint2",
            "right_joint3",
            "right_joint4",
            "right_joint5",
            "right_joint6",
            "right_joint7"
            // "shoulder_pitch_l_joint",
            // "shoulder_roll_l_joint",
            // "shoulder_yaw_l_joint",
            // "elbow_pitch_l_joint",
            // "elbow_yaw_l_joint",
            // "wrist_pitch_l_joint",
            // "wrist_roll_l_joint",
            // "shoulder_pitch_r_joint",
            // "shoulder_roll_r_joint",
            // "shoulder_yaw_r_joint",
            // "elbow_pitch_r_joint",
            // "elbow_yaw_r_joint",
            // "wrist_pitch_r_joint",
            // "wrist_roll_r_joint"};
        };
        
        // Eigen::Vector4d noiseWQP(wqpQ(0),wqpQ(1),wqpQ(2),wqpQ(3));
        // // 创建随机数生成器
        // std::random_device rd;  // 随机种子
        // std::mt19937 gen(rd()); // 随机数生成器
        // std::uniform_real_distribution<double> dist(-0.0015, 0.0015); // 噪声范围为 30%

        // // 为每个分量添加 10% 噪声
        // for (int i = 0; i < noiseWQP.size(); ++i) {
        //     double noise = dist(gen) * wqpQ(i); // 生成基于分量大小的噪声
        //     noiseWQP(i) += noise;
        // }
        // wqpQ.head(4) = noiseWQP;
        wqpQ(0)  = 0.0;
        wqpQ(1)  = 0.0;
        wqpQ(2)  = 0.0;

        // wqpQ.head(3) = Eigen::VectorXd::Zero(3);
        for (int i = 0; i < 17; ++i) 
        { 

            if(control_system.sim>2500&&control_system.sim<4500){
                // joint_state.position[i] = 0.0025*wqpQ(i)+now_q(i);
                desired_position(3) =  -111*3.14/180;
                desired_position(4) =  45* 3.14/180;
                desired_position(5) =  46* 3.14/180;
                desired_position(6) =  -90* 3.14/180;
                desired_position(7) =  45* 3.14/180;
                desired_position(8) =  -19* 3.14/180;
                desired_position(9) =  0;
                
                desired_position(3+7) =  111*3.14/180;
                desired_position(4+7) =  -35* 3.14/180;;
                desired_position(5+7) =  -36* 3.14/180;;
                desired_position(6+7) =  90* 3.14/180;;
                desired_position(7+7) =  -45* 3.14/180;;
                desired_position(8+7) =  19* 3.14/180;;
                desired_position(9+7) =  0;
                // desired_position(3+7) =  1.34;
                // desired_position(4+7) =  0;
                // desired_position(5+7) =  1.92;
                // desired_position(6+7) =  1.23;
                // desired_position(7+7) =  -1.75;
                // desired_position(8+7) =  0;
                // desired_position(9+7) =  0;
                joint_state.position[i] = now_q(i) + 0.0025*(desired_position(i) -now_q(i));

            }
            else if (control_system.sim<=2500)
            {
                desired_position(3) =  -111*3.14/180;
                desired_position(4) =  35* 3.14/180;;
                desired_position(5) =  36* 3.14/180;;
                desired_position(6) =  -90* 3.14/180;;
                desired_position(7) =  45* 3.14/180;;
                desired_position(8) =  -19* 3.14/180;;
                desired_position(9) =  0;

                desired_position(3+7) =  111*3.14/180;
                desired_position(4+7) =  -35* 3.14/180;;
                desired_position(5+7) =  -36* 3.14/180;;
                desired_position(6+7) =  90* 3.14/180;;
                desired_position(7+7) =  -45* 3.14/180;;
                desired_position(8+7) =  19* 3.14/180;;
                desired_position(9+7) =  0;
    
               joint_state.position[i] = now_q(i) + 0.0025*(desired_position(i) -now_q(i));

            }
            // if (control_system.sim<=2500)
            // {
            //     desired_position(0+3) = 0.0 ;
            //     desired_position(1+3) = 145.0 * 3.14/180 ;
            //     desired_position(2+3) = 0.0 ;
            //     desired_position(3+3) = -120.0 * 3.14/180 ;
            //     desired_position(4+3) = 0.0 ;
            //     desired_position(5+3) = -40.0 * 3.14/180 ; ;
            //     desired_position(6+3) = 0.0 ;
            //     joint_state.position[i] = now_q(i) + 0.0025*(desired_position(i) -now_q(i));

            // }
            // else
            // {
            //     joint_state.position[i] = 0.0025*wqpQ(i)+now_q(i);

            // }

            // else if (control_system.sim>=4500&&control_system.sim<6500)
            // {   
            //     desired_position(3) =  -111*3.14/180;
            //     desired_position(4) =  35* 3.14/180;;
            //     desired_position(5) =  36* 3.14/180;;
            //     desired_position(6) =  -90* 3.14/180;;
            //     desired_position(7) =  45* 3.14/180;;
            //     desired_position(8) =  -19* 3.14/180;;
            //     desired_position(9) =  0;


            //     desired_position(3+7) =  1.34;
            //     desired_position(4+7) =  0;
            //     desired_position(5+7) =  1.92 - 0.75;
            //     desired_position(6+7) =  1.23;
            //     desired_position(7+7) =  -1.75;
            //     desired_position(8+7) =  0;
            //     desired_position(9+7) =  0;
            //     // desired_position(3+7) =  111*3.14/180;
            //     // desired_position(4+7) =  -35* 3.14/180;;
            //     // desired_position(5+7) =  -36* 3.14/180;;
            //     // desired_position(6+7) =  90* 3.14/180;;
            //     // desired_position(7+7) =  -45* 3.14/180;;
            //     // desired_position(8+7) =  19* 3.14/180;;
            //     // desired_position(9+7) =  0;

            //     joint_state.position[i] = now_q(i) + 0.025*(desired_position(i) -now_q(i));
            // }
            // else if (control_system.sim>=6500&&control_system.sim<8500)
            // {   
            //                  desired_position(3) =  -111*3.14/180;
            //     desired_position(4) =  35* 3.14/180;;
            //     desired_position(5) =  36* 3.14/180;;
            //     desired_position(6) =  -90* 3.14/180;;
            //     desired_position(7) =  45* 3.14/180;;
            //     desired_position(8) =  -19* 3.14/180;;
            //     desired_position(9) =  0;


            //     desired_position(3+7) =  1.34;
            //     desired_position(4+7) =  0;
            //     desired_position(5+7) =  1.92 + 0.75;
            //     desired_position(6+7) =  1.23;
            //     desired_position(7+7) =  -1.75;
            //     desired_position(8+7) =  0;
            //     desired_position(9+7) =  0;
            //     // desired_position(3+7) =  111*3.14/180;
            //     // desired_position(4+7) =  -35* 3.14/180;;
            //     // desired_position(5+7) =  -36* 3.14/180;;
            //     // desired_position(6+7) =  90* 3.14/180;;
            //     // desired_position(7+7) =  -45* 3.14/180;;
            //     // desired_position(8+7) =  19* 3.14/180;;
            //     // desired_position(9+7) =  0;

            //     joint_state.position[i] = now_q(i) + 0.025*(desired_position(i) -now_q(i));
            // }
            else
            {
                joint_state.position[i] = 0.0025*wqpQ(i)+now_q(i);
                // joint_state.position[i] = now_q(i) + 0.0025*(desired_position(i) -now_q(i));


            }

        }
        control_system.sim+=1;
        control_system.joint_pub.publish(joint_state);
        // control_system.getMCGForFirst7Joints(q, v);
        // Eigen::VectorXd tau = control_system.computeTorqueWithSlidingMode(now_q, now_qd, desired_position, desired_velocity, desired_acceleration);
        // tau=tau;
        // control_system.setJointTorque("waist_yaw_joint", tau(0));
        // control_system.setJointTorque("shoulder_pitch_l_joint", tau(1));
        // control_system.setJointTorque("shoulder_roll_l_joint", tau(2));
        // control_system.setJointTorque("shoulder_yaw_l_joint", tau(3));
        // control_system.setJointTorque("elbow_pitch_l_joint", tau(4));
        // control_system.setJointTorque("elbow_yaw_l_joint", tau(5));
        // control_system.setJointTorque("wrist_pitch_l_joint", tau(6));
        // control_system.setJointTorque("wrist_roll_l_joint", tau(7));
        // control_system.setJointTorque("shoulder_pitch_r_joint", tau(8));
        // control_system.setJointTorque("shoulder_roll_r_joint", tau(9));
        // control_system.setJointTorque("shoulder_yaw_r_joint", tau(10));
        // control_system.setJointTorque("elbow_pitch_r_joint", tau(11));
        // control_system.setJointTorque("elbow_yaw_r_joint", tau(12));
        // control_system.setJointTorque("wrist_pitch_r_joint", tau(13));
        // control_system.setJointTorque("wrist_roll_r_joint", tau(14));
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
