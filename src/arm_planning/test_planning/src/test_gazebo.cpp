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



ControlSystem::ControlSystem(const std::string& urdf_filename) 
{
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
    joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    desired_histories.resize(18);
    joint_state.resize(18);
    impedance_joint_M = Eigen::VectorXd::Zero(8);
    impedance_joint_B = Eigen::VectorXd::Zero(8);
    impedance_joint_K = Eigen::VectorXd::Zero(8);
    impedance_Cartesian_M =  Eigen::MatrixXd::Identity(6, 6);
    impedance_Cartesian_B =  Eigen::MatrixXd::Identity(6, 6);
    impedance_Cartesian_K =  Eigen::MatrixXd::Identity(6, 6);
    last_tau = Eigen::VectorXd::Zero(15);
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

void ControlSystem::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
   
    // 更新关节历史状态
    std::vector<std::string> joint_names = {
        "waist_Z_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "waist_yaw_joint","shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
        "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
        "shoulder_pitch_r_joint", "shoulder_roll_r_joint", "shoulder_yaw_r_joint",
        "elbow_pitch_r_joint", "elbow_yaw_r_joint", "wrist_pitch_r_joint", "wrist_roll_r_joint"};

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

Eigen::VectorXd ControlSystem::testWqp(Eigen::VectorXd& nowq,Eigen::VectorXd& nowqv)
{
    // std::cout<<"=====now_q======"<<nowq.transpose()<<std::endl;

    double time =ros::Time::now().toSec(); // 当前时间;
    pinocchio::computeJointJacobians(model_, data_, nowq);

    pinocchio::forwardKinematics(model_, data_, nowq);
    pinocchio::updateFramePlacements(model_, data_);
    // step1 获取lr 雅可比
    pinocchio::FrameIndex frame_id_left = model_.getFrameId("left_flange");
    pinocchio::FrameIndex frame_id_right = model_.getFrameId("right_flange");
    // std::cout<<"FrameIndex:"<<frame_id_left<<";"<<frame_id_right<<std::endl;
    Eigen::MatrixXd jacobian_l = pinocchio::getFrameJacobian(model_, data_, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);
    Eigen::MatrixXd jacobian_r = pinocchio::getFrameJacobian(model_, data_, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED);

    // step2 获取笛卡尔err

    Eigen::Vector3d pos_l = data_.oMf[27].translation();
    Eigen::Vector3d pos_r = data_.oMf[79].translation();
    Eigen::Quaterniond quat_l(data_.oMf[27].rotation());
    Eigen::Quaterniond quat_r(data_.oMf[79].rotation());

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


    Eigen::Vector3d pos_l_d(0.445156,0.287187,0.215173);
    Eigen::Quaterniond quat_l_d;
    quat_l_d.x()=-0.0345472;
    quat_l_d.y()=-0.232099;
    quat_l_d.z()=0.673005;
    quat_l_d.w()=0.701428;
    // car sin sign test 
    // double a = -0.303942+0.08*sin(0.5*time);
    // double b = 0.171599+0.08*sin(0.5*time);
    Eigen::Vector3d pos_r_d(0.422576,-0.303942,0.171599);
    Eigen::Quaterniond quat_r_d;
    quat_r_d.x()=0.701961;
    quat_r_d.y()=-0.684411;
    quat_r_d.z()=0.196978;
    quat_r_d.w()=0.00561809;
    
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
    carErr_l.head(3) = pos_l_d - pos_l;
    // carErr_l.tail(3) = temp_rotError_l;
    carErr_r.head(3) = pos_r_d - pos_r;
    // carErr_r.tail(3) = temp_rotError_r;

    carErr_l(0)+=0.05*sin(0.3*time);
    carErr_l(2)+=0.05*cos(0.3*time);

    carErr_r(1)+=0.1*sin(-0.6*time);
    carErr_r(2)+=0.1*cos(-0.6*time);
    Eigen::VectorXd carE = Eigen::VectorXd::Zero(12); 
    carE<<carErr_l,carErr_r;

    Eigen::VectorXd Qr = Eigen::VectorXd::Zero(18);
    // Qr = nowq;
    Qr(6)= -20*sin(time)*3.14/180 - nowq(6);
    Qr(13)= 20*sin(time)*3.14/180 - nowq(13);

    // std::cout<<"=======wqp1.1========="<<std::endl;

    auto result = WQP(jacobian_l,
                    jacobian_r,
                    carE,
                    nowq,
                    Qr);
    // std::cout<<result.transpose()<<std::endl;
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

            std::cout<<"===shoulderError=="<<nowQ(6)*180/3.14<<";"<<nowQ(13)*180/3.14<<std::endl;

            double eta_car = 10000;
            double eta_qpos= 500;
            Eigen::MatrixXd JointWeight = Eigen::MatrixXd::Identity(18, 18);
            JointWeight(0,0) = 1;
            JointWeight(1,1) = 0.01;
            JointWeight(2,2) = 0.01;
            JointWeight(3,3) = 0.01;

            Eigen::MatrixXd H = eta_car * Jl.transpose()*Jl 
                            + eta_car * Jr.transpose()*Jr 
                            + eta_qpos * Eigen::MatrixXd::Identity(18, 18) + JointWeight;
            Eigen::VectorXd c =   -eta_car*Jl.transpose()*car_err.head(6) 
                                  -eta_car*Jr.transpose()*car_err.tail(6)
                                  - eta_qpos*Qr;
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(66,18);
            A.block<6,18>(0,0) = Jl;
            A.block<6,18>(6,0) = Jr;
            A.block<18,18>(12,0) = Eigen::MatrixXd::Identity(18, 18);
            A.block<18,18>(30,0) = Eigen::MatrixXd::Identity(18, 18);

            Eigen::MatrixXd shoulderLim = Eigen::MatrixXd::Zero(18, 18);
            shoulderLim(6,6)= 1;
            // shoulderLim(8,8)= 1;

            shoulderLim(13,13)= 1;
            // shoulderLim(15,15)= 1;
            Eigen::VectorXd shoulderU = Eigen::VectorXd::Zero(18);
            Eigen::VectorXd shoulderL = Eigen::VectorXd::Zero(18);
            shoulderU.setConstant(1);
            shoulderL.setConstant(-1);
            shoulderU(6) = Qr(6) + 100*3.1415926535/180;
            // shoulderU(8) = Qr(8) + 5*3.1415926535/180;
            shoulderU(13) = Qr(13) + 100*3.1415926535/180;
            // shoulderU(15) = Qr(15) + 5*3.1415926535/180;

            shoulderL(6) = Qr(6)- 100*3.1415926535/180;
            // shoulderL(8) = Qr(8)- 5*3.1415926535/180;
            shoulderL(13) = Qr(13) - 100*3.1415926535/180;
            // shoulderL(15) = Qr(15) - 5*3.1415926535/180;

            //  5*3.1415926535/180;
            A.block<18,18>(48,0) = shoulderLim;
            // std::cout<<"=======wqp1.2========="<<std::endl;

            //==========约束1 笛卡尔线速度角速度
            Eigen::VectorXd Car_max = Eigen::VectorXd::Zero(12);
            Car_max.setConstant(1);

            Eigen::VectorXd Car_min = Eigen::VectorXd::Zero(12);
            Car_min.setConstant(-1);
            //==========约束2 关节速度

            Eigen::VectorXd qv_max = Eigen::VectorXd::Zero(18);
            qv_max.setConstant(1);
            qv_max(0) = 0.01 ;
            qv_max(1) = 0.02 ;
            qv_max(2) = 0.02 ;
            qv_max(3) = 0.002 ;
            Eigen::VectorXd qv_min = Eigen::VectorXd::Zero(18);
            qv_min .setConstant(-1);
            qv_min(0) = -0.01 ;
            qv_min(1) = -0.02 ;
            qv_min(2) = -0.02 ;
            qv_min(3) = -0.02 ;
            //==========约束3 关节位置
            // △q ＜ qmax -qnow
            Eigen::VectorXd q_max = Eigen::VectorXd::Zero(18);
            Eigen::VectorXd q_min = Eigen::VectorXd::Zero(18);
            q_max<<0,0.25,0.25,0.25, //腰部关节
            2.96,3.4,2.96,0,2.96,1.04,1.48,//l
            2.96,3.4,2.96,0,2.96,1.04,1.48;//r
            q_min<<-0.15,-0.25,-0.25,-0.25,
            -2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66,
            -2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;
            //
            Eigen::VectorXd up = Eigen::VectorXd::Zero(66);
            Eigen::VectorXd lp = Eigen::VectorXd::Zero(66);


            // std::cout<<"=======wqp1.3========="<<std::endl;

            Eigen::VectorXd car_temp_ = Eigen::VectorXd::Zero(6);
            car_temp_<< 1,1,1,
                        1,1,1;
            up.head(6) = car_temp_ ;
            up.segment(6, 6) = car_temp_;
            lp.head(6) = -car_temp_ ;
            lp.segment(6, 6) = -car_temp_;

            up.segment(12, 18) = qv_max;
            lp.segment(12, 18) = qv_min;

            up.segment(30, 18) = q_max-nowQ;
            lp.segment(30, 18) = q_min-nowQ;
            up.tail(18) = shoulderU;
            lp.tail(18) = shoulderL;

            OsqpEigen::Solver solver;


            int n = 18;
            solver.settings()->setWarmStart(false);
            solver.settings()->setVerbosity(false);
            solver.data()->setNumberOfVariables(18);
            solver.data()->setNumberOfConstraints(66);
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
            return solver.getSolution();
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
int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_control_node");

    ControlSystem control_system("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf");
    // ros::Publisher joint_pub;
    // control_system->joint_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd a_desired = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd q_desired = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd v_desired = Eigen::VectorXd::Zero(18);

    ros::Rate loop_rate(400);

    while (ros::ok()) 
    {
        double time = ros::Time::now().toSec();

        Eigen::VectorXd desired_position = Eigen::VectorXd::Zero(18);
        Eigen::VectorXd now_q(18);
        Eigen::VectorXd now_qd(18);

        Eigen::VectorXd max(7);
        Eigen::VectorXd min(7);
        max<<2.96,3.4,2.96,0,2.96,1.04,1.48;
        min<<-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;

        Eigen::VectorXd mid;
        mid = (max+min)/2;
        
        for (int i = 0; i < 18; ++i) 
        {   
            desired_position(3+4) = -3.1415926535/2; // 设置每个关节的期望位置为 0.5 * sin(time)
            desired_position(3+11) = -3.1415926535/2; // 设置每个关节的期望位置为 0.5 * sin(time)
            // desired_position(i)= mid(i)+0.5*sin(1.5*time);

            now_q(i)=control_system.joint_state[i].position;
            now_qd(i)=control_system.joint_state[i].velocity;

        }
        auto wqpQ = control_system.testWqp(now_q,now_qd);
        // desired_position(3)= -mid(3)+0.5*sin(0.5*time);
        // 更新期望轨迹中的位置
        control_system.traj_data.addPosition(desired_position);

        // 获取期望的速度和加速度，返回值为 Eigen::VectorXd 类型
        Eigen::VectorXd desired_velocity = control_system.traj_data.getVelocities();
        Eigen::VectorXd desired_acceleration = control_system.traj_data.getAccelerations();
        sensor_msgs::JointState joint_state;
        joint_state.name.resize(18); 
        joint_state.position.resize(18);
        joint_state.velocity.resize(18);
        joint_state.effort.resize(18);
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = 
        {
            "waist_Z_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "waist_yaw_joint",
            "shoulder_pitch_l_joint",
            "shoulder_roll_l_joint",
            "shoulder_yaw_l_joint",
            "elbow_pitch_l_joint",
            "elbow_yaw_l_joint",
            "wrist_pitch_l_joint",
            "wrist_roll_l_joint",
            "shoulder_pitch_r_joint",
            "shoulder_roll_r_joint",
            "shoulder_yaw_r_joint",
            "elbow_pitch_r_joint",
            "elbow_yaw_r_joint",
            "wrist_pitch_r_joint",
            "wrist_roll_r_joint"};
        for (int i = 0; i < 18; ++i) 
        { 
            if(control_system.sim>2500){
                joint_state.position[i] = 0.01*wqpQ(i)+now_q(i);
            }
            else
            {
                joint_state.position[i] = desired_position(i);
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
