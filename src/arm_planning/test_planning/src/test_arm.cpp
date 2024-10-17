
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <llm_msgs/hand_pose_req.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Eigen>
#include <arm_kinematics_solver/arm_kinematics_solver.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <llm_msgs/hand_pose_req.h>
#include <Eigen/Dense>
#include <array>
#include <vector>
// #define SIM


class SmoothingFilter {
public:
    SmoothingFilter(size_t windowSize) : windowSize(windowSize), index(0) {
        values.resize(windowSize, {0.0, 0.0, 0.0}); // 初始化数组
    }

    std::array<double, 3> filter(const std::array<double, 3>& input_force) {
        values[index] = input_force;

        std::array<double, 3> smoothed_value = {0.0, 0.0, 0.0};
        for (const auto& val : values) {
            smoothed_value[0] += val[0];
            smoothed_value[1] += val[1];
            smoothed_value[2] += val[2];
        }

        smoothed_value[0] /= windowSize;
        smoothed_value[1] /= windowSize;
        smoothed_value[2] /= windowSize;

        index = (index + 1) % windowSize; // 更新索引
        return smoothed_value;
    }

private:
    size_t windowSize; // 窗口大小
    std::vector<std::array<double, 3>> values; // 存储最近的值
    int index; // 当前索引
};

class ArmController {
public:
    ArmController(ros::NodeHandle& nh) 
        : filter_left(3),filter_right(3) {  // 初始化平滑滤波器，窗口大小为5
        left_arm_pub_ = nh.advertise<llm_msgs::hand_pose_req>("/left_arm_pose_req", 10);
        right_arm_pub_ = nh.advertise<llm_msgs::hand_pose_req>("/right_arm_pose_req", 10);

        left_sub = nh.subscribe("desired_pose_left", 10, &ArmController::positionCallback_left, this);
        right_sub = nh.subscribe("desired_pose_right", 10, &ArmController::positionCallback_right, this);
        
        #ifdef SIM
            joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);
            // joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);

        #else
            joint_state_sub_left_teleoperation = nh.subscribe("/human_arm_cmd_left" , 10, &ArmController::jointStateCallback_left_teleoperation, this);
            joint_state_sub_right_teleoperation = nh.subscribe("/human_arm_cmd_right" , 10, &ArmController::jointStateCallback_right_teleoperation, this);

            joint_state_sub_left = nh.subscribe("/human_arm_state_left" , 10, &ArmController::jointStateCallback_left, this);
            joint_state_sub_right = nh.subscribe("/human_arm_state_right" , 10, &ArmController::jointStateCallback_right, this);
            

        #endif
        // joint_state_sub = nh.subscribe("/human_arm_state_left", 10, &ArmController::jointStateCallback, this);

        first();
        ros::Rate r(198);
        /////////////////////////
        while (ros::ok()) {
            command();
            publishGraspPoses();
            ros::spinOnce();    
            r.sleep();
        }
    }

    struct ArmParam {
        int n_joints;
        std::string name;
        std::string base_link;
        std::string eef_link;
        std::vector<std::string> joint_names;
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
        std::vector<double> joint_eff;
        std::vector<double> joints_min;
        std::vector<double> joints_max;
        int which_arm; // 0-left, 1-right
        double max_line_vel, max_rot_vel, max_line_acc, max_rot_acc, max_line_jerk, max_rot_jerk;

        void resize() {
            joint_names.resize(n_joints);
            joint_pos.resize(n_joints);
            joint_vel.resize(n_joints);
            joint_eff.resize(n_joints);
            joints_min.resize(n_joints);
            joints_max.resize(n_joints);
        };
    };

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        auto joint_positions_L = msg->position; // 保存 joint position

        std::string error_string_left;

        Eigen::MatrixXd InitPoseT_left;

        arm_kinematics::ArmKinematicsSolver kin_solver_leftSim(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);

        kin_solver_leftSim.getFkSolution(InitPoseT_left, joint_positions_L, error_string_left);
        // L_grasp_pose[0] = InitPoseT_left(0, 3);
        // L_grasp_pose[1] = InitPoseT_left(1, 3);
        // L_grasp_pose[2] = InitPoseT_left(2, 3);


        auto joint_positions_R = msg->position; // 保存 joint position

        std::string error_string_right;

        Eigen::MatrixXd InitPoseT_right;
        arm_kinematics::ArmKinematicsSolver kin_solver_rightSim(urdf_path, arm_param_right.base_link, arm_param_right.eef_link, arm_param_right.joints_min, arm_param_right.joints_max);

        kin_solver_rightSim.getFkSolution(InitPoseT_right, joint_positions_R, error_string_right);
        // R_grasp_pose[0] = InitPoseT_right(0, 3);
        // R_grasp_pose[1] = InitPoseT_right(1, 3);
        // R_grasp_pose[2] = InitPoseT_right(2, 3);
    }
    void jointStateCallback_left(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        // auto joint_positions_R = msg->position; // 保存 joint position
        arm_param_left.name = "left";
        arm_param_left.base_link = "base_link";
        arm_param_left.eef_link = "left_flange";
        arm_param_left.joints_min ={-3.1415926, 0, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
        arm_param_left.joints_max ={0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};

        arm_param_right.name = "right";
        arm_param_right.base_link = "base_link";
        arm_param_right.eef_link = "right_flange";
        arm_param_right.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
        arm_param_right.joints_max ={3.1415926  ,0         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};
        // std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        // urdf_path += "/models/AUBO_HRM.urdf";
        auto joint_positions_L = msg->position; // 保存 joint position

        std::string error_string_left;

        Eigen::MatrixXd InitPoseT_left;

        arm_kinematics::ArmKinematicsSolver kin_solver_left(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);

        kin_solver_left.getFkSolution(InitPoseT_left, joint_positions_L, error_string_left);
        // if (1)
        // {
        //             // L_grasp_pose[0]=InitPoseT_left(0, 3);
        //             // L_grasp_pose[1]=InitPoseT_left(1, 3);
        //             // L_grasp_pose[2]=InitPoseT_left(2, 3);
        //             Eigen::Quaterniond quaternion(InitPoseT_left.block<3,3>(0,0)); 
        //             L_grasp_pose[3]=quaternion.x();
        //             L_grasp_pose[4]=quaternion.y();
        //             L_grasp_pose[5]=quaternion.z();
        //             L_grasp_pose[6]=quaternion.w();
        //             // first_L_=false;
        // }
        

    }
    void jointStateCallback_right_teleoperation(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        auto joint_positions_R = msg->position; // 保存 joint position

        std::string error_string_right;

        Eigen::MatrixXd InitPoseT_right;
        arm_kinematics::ArmKinematicsSolver kin_solver_right(urdf_path, arm_param_right.base_link, arm_param_right.eef_link, arm_param_right.joints_min, arm_param_right.joints_max);

        kin_solver_right.getFkSolution(InitPoseT_right, joint_positions_R, error_string_right);

        double temp_R_grasp_pose0=InitPoseT_right(0, 3);
        double temp_R_grasp_pose1=InitPoseT_right(1, 3);
        double temp_R_grasp_pose2=InitPoseT_right(2, 3);
        // Eigen::Quaterniond quaternion(InitPoseT_right.block<3,3>(0,0)); 
        std::cout<<"test==R="<<temp_R_grasp_pose0<<";"<<temp_R_grasp_pose1<<"；"<<temp_R_grasp_pose2<<std::endl;
        // R_grasp_pose[3]=quaternion.x();
        // R_grasp_pose[4]=quaternion.y();
        // R_grasp_pose[5]=quaternion.z();
        // R_grasp_pose[6]=quaternion.w();

    }
    void jointStateCallback_left_teleoperation(const sensor_msgs::JointState::ConstPtr& msg) 
    {

        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        auto joint_positions_L = msg->position; // 保存 joint position

        std::string error_string_left;

        Eigen::MatrixXd InitPoseT_left;

        arm_kinematics::ArmKinematicsSolver kin_solver_left(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);

        kin_solver_left.getFkSolution(InitPoseT_left, joint_positions_L, error_string_left);

        //  L_grasp_pose[0]=InitPoseT_left(0, 3);
        //  L_grasp_pose[1]=InitPoseT_left(1, 3);
        //  L_grasp_pose[2]=InitPoseT_left(2, 3);
        // double temp_R_grasp_pose0=InitPoseT_left(0, 3);
        // double temp_R_grasp_pose1=InitPoseT_left(1, 3);
        // double temp_R_grasp_pose2=InitPoseT_left(2, 3);
        // Eigen::Quaterniond quaternion(InitPoseT_right.block<3,3>(0,0)); 
        // std::cout<<"test==L="<<temp_R_grasp_pose0<<";"<<temp_R_grasp_pose1<<"；"<<temp_R_grasp_pose2<<std::endl;
        // Eigen::Quaterniond quaternion(InitPoseT_left.block<3,3>(0,0)); 
        // L_grasp_pose[3]=quaternion.x();
        // L_grasp_pose[4]=quaternion.y();
        // L_grasp_pose[5]=quaternion.z();
        // L_grasp_pose[6]=quaternion.w();
    }
    void jointStateCallback_right(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        auto joint_positions_R = msg->position; // 保存 joint position
        arm_param_left.name = "left";
        arm_param_left.base_link = "base_link";
        arm_param_left.eef_link = "left_flange";
        arm_param_left.joints_min ={-3.1415926, 0, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
        arm_param_left.joints_max ={0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};

        arm_param_right.name = "right";
        arm_param_right.base_link = "base_link";
        arm_param_right.eef_link = "right_flange";
        arm_param_right.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
        arm_param_right.joints_max ={3.1415926  ,0         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};
        std::string error_string_right;

        Eigen::MatrixXd InitPoseT_right;
        arm_kinematics::ArmKinematicsSolver kin_solver_right(urdf_path, arm_param_right.base_link, arm_param_right.eef_link, arm_param_right.joints_min, arm_param_right.joints_max);

        kin_solver_right.getFkSolution(InitPoseT_right, joint_positions_R, error_string_right);
        // if (first_R_)
        // {
        //             R_grasp_pose[0]=InitPoseT_right(0, 3);
        //             R_grasp_pose[1]=InitPoseT_right(1, 3);
        //             R_grasp_pose[2]=InitPoseT_right(2, 3);
        //             Eigen::Quaterniond quaternion(InitPoseT_right.block<3,3>(0,0)); 
        //             R_grasp_pose[3]=quaternion.x();
        //             R_grasp_pose[4]=quaternion.y();
        //             R_grasp_pose[5]=quaternion.z();
        //             R_grasp_pose[6]=quaternion.w();
        //             first_R_=false;
        // }
        


    }

    void positionCallback_left(const geometry_msgs::Pose::ConstPtr& msg) {
        // 将接收到的位置数据传递给滤波器
        std::array<double, 3> input_position = {msg->position.x, msg->position.y, msg->position.z};
        auto filtered_position = filter_left.filter(input_position);

        // 更新 xx, yy, zz 为滤波后的值
        xx_left = filtered_position[0];
        yy_left = filtered_position[1];
        zz_left = filtered_position[2];
        // xx_left = msg->position.x;
        // yy_left = msg->position.y;
        // zz_left = msg->position.z;
        qx_left = msg->orientation.x;
        qy_left = msg->orientation.y;
        qz_left = msg->orientation.z;
        qw_left = msg->orientation.w;
    }
    void positionCallback_right(const geometry_msgs::Pose::ConstPtr& msg) {
        // 将接收到的位置数据传递给滤波器
        std::array<double, 3> input_position = {msg->position.x, msg->position.y, msg->position.z};
        auto filtered_position = filter_right.filter(input_position);

        // 更新 xx, yy, zz 为滤波后的值
                //  pose_msg.position.x, 
        //  pose_msg.position.y, 
        //  pose_msg.position.z, 
        xx_right = filtered_position[0];
        yy_right = filtered_position[1];
        zz_right = filtered_position[2];
        // xx_right = msg->position.x;
        // yy_right = msg->position.y;
        // zz_right = msg->position.z;
        qx_right = msg->orientation.x;
        qy_right = msg->orientation.y;
        qz_right = msg->orientation.z;
        qw_right = msg->orientation.w;
    }
    void first() {
        ROS_INFO("=======first========");


        arm_param_left.name = "left";
        arm_param_left.base_link = "base_link";
        arm_param_left.eef_link = "left_flange";
        arm_param_left.joints_min ={-3.1415926, -0.2, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
        arm_param_left.joints_max ={0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};

        arm_param_right.name = "right";
        arm_param_right.base_link = "base_link";
        arm_param_right.eef_link = "right_flange";
        arm_param_right.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
        arm_param_right.joints_max ={3.1415926  ,0.2         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};



        // L_grasp_pose = {0.352496-0.035, 0.162388 + 0.015, 0.746278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
        // R_grasp_pose = {0.354662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};
        // L_grasp_pose = { 0.505169,0.323111,0.249817,0.0872157,0.631405,-0.11521,0.761871 };
        // R_grasp_pose = { 0.50705,-0.317131,0. ,0.702849,0.108244,0.695218,-0.104683};
        // L_grasp_pose = { 0.405169,0.323111,0.249817,0.0872157,0.631405,-0.11521,0.76187};
                                                   
        L_grasp_pose = { 0.505169,0.323111,0.249817,0.303926,0.884509,-0.0545777,0.349706 };
        R_grasp_pose = { 0.50705,-0.317131,0.249316,0.702849,0.108244,0.695218,-0.104683};
        left_grasp_pose.hand_move_enable = 1;
        left_grasp_pose.hand_side = 0;
        left_grasp_pose.hand_reset = 1;
        left_grasp_pose.pose_req.position.x = L_grasp_pose[0];
        left_grasp_pose.pose_req.position.y = L_grasp_pose[1];
        left_grasp_pose.pose_req.position.z = L_grasp_pose[2];
        left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];
        left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4];
        left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5];
        left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6];

        right_grasp_pose.hand_move_enable = 1;
        right_grasp_pose.hand_side = 1;
        right_grasp_pose.hand_reset = 1;        
        right_grasp_pose.pose_req.position.x = R_grasp_pose[0]; 
        right_grasp_pose.pose_req.position.y = R_grasp_pose[1];
        right_grasp_pose.pose_req.position.z = R_grasp_pose[2];
        right_grasp_pose.pose_req.orientation.x = R_grasp_pose[3];
        right_grasp_pose.pose_req.orientation.y = R_grasp_pose[4];
        right_grasp_pose.pose_req.orientation.z = R_grasp_pose[5];
        right_grasp_pose.pose_req.orientation.w = R_grasp_pose[6];
        
        sleep(1);
        left_arm_pub_.publish(left_grasp_pose);

        right_arm_pub_.publish(right_grasp_pose); // 如果需要右手抓握位置，可以取消注释
        sleep(1);

        // 更新左抓握位置
        updateGraspPoses(left_grasp_pose, L_grasp_pose);
        // 更新右抓握位置
        updateGraspPoses(right_grasp_pose, R_grasp_pose);
    }

    void updateGraspPoses(llm_msgs::hand_pose_req& grasp_pose, const std::vector<double>& grasp_pose_vals) 
    {
        grasp_pose.pose_req.position.x = grasp_pose_vals[0];
        grasp_pose.pose_req.position.y = grasp_pose_vals[1];
        grasp_pose.pose_req.position.z = grasp_pose_vals[2];
        grasp_pose.pose_req.orientation.x = grasp_pose_vals[3];
        grasp_pose.pose_req.orientation.y = grasp_pose_vals[4];
        grasp_pose.pose_req.orientation.z = grasp_pose_vals[5];
        grasp_pose.pose_req.orientation.w = grasp_pose_vals[6];
    }

    void command () 
    {
        double radius = 0.08; // 圆的半径
        double angular_velocity = 1.0; // 角速度
        double time = ros::Time::now().toSec(); // 当前时间
        //往复测试
        double line = radius * cos(angular_velocity * time);
        //姿态正弦模拟信号
        double amplitude = M_PI / 12;
        double frequency = 0.2;
        double angle_increment = amplitude * std::sin(frequency * time);
        Eigen::AngleAxisd delta_angle_axis(angle_increment, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond delta_q = Eigen::Quaterniond(delta_angle_axis); 



        //左侧位置导纳增量
        Eigen::Vector3d delta_xyz_left(xx_left, yy_left, zz_left);
        auto nowTcp2b_l = Quat2T_matrix(L_grasp_pose);
        auto result_left = nowTcp2b_l.block<3,3>(0,0) * delta_xyz_left;
        //右侧侧位置导纳增量
        Eigen::Vector3d delta_xyz_right(xx_right, yy_right, zz_right);
        auto nowTcp2b_r = Quat2T_matrix(R_grasp_pose);
        auto result_right = nowTcp2b_r.block<3,3>(0,0) * delta_xyz_right;


        //左侧姿态导纳叠加
        Eigen::Quaterniond now_q_l;
        Eigen::Quaterniond dw_q_l;
        now_q_l.x() = L_grasp_pose[3];
        now_q_l.y() = L_grasp_pose[4];
        now_q_l.z() = L_grasp_pose[5];
        now_q_l.w() = L_grasp_pose[6];
        dw_q_l.x() = qx_left;
        dw_q_l.y() = qy_left;
        dw_q_l.z() = qz_left;
        dw_q_l.w() = qw_left;
        Eigen::Quaterniond update_q_l;
        Eigen::Quaterniond result_l = now_q_l *dw_q_l*delta_q;
        result_l.normalize();

        //右侧姿态导纳叠加
        Eigen::Quaterniond now_q_r;//当前期望姿态
        Eigen::Quaterniond dw_q_r;
        now_q_r.x() = R_grasp_pose[3];
        now_q_r.y() = R_grasp_pose[4];
        now_q_r.z() = R_grasp_pose[5];
        now_q_r.w() = R_grasp_pose[6];
        dw_q_r.x() = qx_right;
        dw_q_r.y() = qy_right;
        dw_q_r.z() = qz_right;
        dw_q_r.w() = qw_right;
        Eigen::Quaterniond update_q_r;
        Eigen::Quaterniond result_r = now_q_r*dw_q_r;
        result_r.normalize();

        left_grasp_pose.hand_move_enable = 1;
        left_grasp_pose.hand_side = 0;
        left_grasp_pose.hand_reset = 1;
        left_grasp_pose.pose_req.position.x = L_grasp_pose[0]+result_left(0);
        left_grasp_pose.pose_req.position.y = L_grasp_pose[1]+result_left(1);
        left_grasp_pose.pose_req.position.z = L_grasp_pose[2]+result_left(2);
        // std::cout<<"====="<<result_left.transpose()<<std::endl;
        // std::cout<<"====="<<L_grasp_pose[0]<<";"<<L_grasp_pose[1]<<";"<<L_grasp_pose[2]<<";"<<L_grasp_pose[3]<<";"<<L_grasp_pose[4]<<";"<<L_grasp_pose[5]<<";"<<L_grasp_pose[6]<<";"<<std::endl;
        // left_grasp_pose.pose_req.position.x = L_grasp_pose[0]+result_left(0);
        // left_grasp_pose.pose_req.position.y = L_grasp_pose[1]+result_left(1);
        // left_grasp_pose.pose_req.position.z = L_grasp_pose[2]+result_left(2);
        // left_grasp_pose.pose_req.position.x = L_grasp_pose[0]+ddddd(0)+result_left(0);
        // left_grasp_pose.pose_req.position.y = L_grasp_pose[1]+ddddd(1)+result_left(1);
        // left_grasp_pose.pose_req.position.z = L_grasp_pose[2]+ddddd(2)+result_left(2);
        left_grasp_pose.pose_req.orientation.x = result_l.x();
        left_grasp_pose.pose_req.orientation.y = result_l.y();
        left_grasp_pose.pose_req.orientation.z = result_l.z();
        left_grasp_pose.pose_req.orientation.w = result_l.w();
        // left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];      
        // left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4]; 
        // left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5]; 
        // left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6]; 
        
        // left_grasp_pose.pose_req.orientation.x = qx_left;
        // left_grasp_pose.pose_req.orientation.y = qy_left;
        // left_grasp_pose.pose_req.orientation.z = qz_left;
        // left_grasp_pose.pose_req.orientation.w = qw_left;

        right_grasp_pose.hand_move_enable = 1;
        right_grasp_pose.hand_side = 1;
        right_grasp_pose.hand_reset = 1;  
        // right_grasp_pose.pose_req.position.x = R_grasp_pose[0]+ddddd(0);
        // right_grasp_pose.pose_req.position.y = R_grasp_pose[1]+ddddd(1);
        // right_grasp_pose.pose_req.position.z = R_grasp_pose[2]+ddddd(2);

        right_grasp_pose.pose_req.position.x = R_grasp_pose[0]+result_right(0);
        right_grasp_pose.pose_req.position.y = R_grasp_pose[1]+result_right(1);
        right_grasp_pose.pose_req.position.z = R_grasp_pose[2]+result_right(2);
        // right_grasp_pose.pose_req.position.x = R_grasp_pose[0];
        // right_grasp_pose.pose_req.position.y = R_grasp_pose[1];
        // right_grasp_pose.pose_req.position.z = R_grasp_pose[2];
        // Eigen::Quaterniond testQ;
        // testQ.x()=qx_right;
        // testQ.y()=qy_right;
        // testQ.z()=qz_right;
        // testQ.w()=qw_right;
        // testQ=testQ*delta_q;
        // testQ.normalize();
        // qx_right=testQ.x();
        // qy_right=testQ.y();
        // qz_right=testQ.z();
        // qw_right=testQ.w();
        right_grasp_pose.pose_req.orientation.x = qx_right;
        right_grasp_pose.pose_req.orientation.y = qy_right;
        right_grasp_pose.pose_req.orientation.z = qz_right;
        right_grasp_pose.pose_req.orientation.w = qw_right;
        // right_grasp_pose.pose_req.orientation.x = result_r.x();
        // right_grasp_pose.pose_req.orientation.y = result_r.y();
        // right_grasp_pose.pose_req.orientation.z = result_r.z();
        // right_grasp_pose.pose_req.orientation.w = result_r.w();
        // std::cout<<"===RR=="<<R_grasp_pose[0]+result_right(0)<<";"<<R_grasp_pose[1]+result_right(1)<<";"<<R_grasp_pose[2]+result_right(2)<<";"<<qx_right<<";"<<qy_right<<";"<<qz_right<<";"<<qw_right<<";"<<std::endl;
        // std::cout<<"===LL=="<<L_grasp_pose[0]+result_left(0)<<";"<<L_grasp_pose[1]+result_left(1)<<";"<<L_grasp_pose[2]+result_left(2)<<";"<<qx_left<<";"<<qy_left<<";"<<qz_left<<";"<<qw_left<<";"<<std::endl;


    }

    void publishGraspPoses() 
    {
        left_arm_pub_.publish(left_grasp_pose);
        right_arm_pub_.publish(right_grasp_pose); // 如果需要右手抓握位置，可以取消注释
    }

    Eigen::Matrix4d Quat2T_matrix(const std::vector<double>& quat) {
        Eigen::Vector3d posValue;
        Eigen::Quaterniond quattt;

        for (int i = 0; i < 3; i++) {
            posValue(i) = quat[i];
        }

        quattt.x() = quat[3];
        quattt.y() = quat[4];
        quattt.z() = quat[5];
        quattt.w() = quat[6];

        Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
        T.block<3, 3>(0, 0) = quattt.toRotationMatrix();
        T.block<3, 1>(0, 3) = posValue;

        return T;
    }

private:
    // ros::Publisher pub_;
    ros::Publisher left_arm_pub_;
    ros::Publisher right_arm_pub_;
    // ros::Publisher test_trajectory_pub_;
    ros::Subscriber left_sub;
    ros::Subscriber right_sub;

    ros::Subscriber joint_state_sub;
    ros::Subscriber joint_state_sub_left;
    ros::Subscriber joint_state_sub_right;

    ros::Subscriber joint_state_sub_left_teleoperation;
    ros::Subscriber joint_state_sub_right_teleoperation;

    llm_msgs::hand_pose_req left_grasp_pose, right_grasp_pose;
    ArmParam arm_param_left;
    ArmParam arm_param_right;
    std::vector<double> L_grasp_pose{7}, R_grasp_pose{7};

    double xx_left;
    double yy_left;
    double zz_left;
    double qx_left;
    double qy_left;
    double qz_left;
    double qw_left;

    double xx_right;
    double yy_right;
    double zz_right;
    double qx_right;
    double qy_right;
    double qz_right;
    double qw_right;
    bool first_R_ =true;
    bool first_L_ =true;

    SmoothingFilter filter_left; // 添加平滑滤波器
    SmoothingFilter filter_right; // 添加平滑滤波器

};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test_arm");
    ros::NodeHandle nh;

    ArmController arm_controller(nh);

    return 0;
}
