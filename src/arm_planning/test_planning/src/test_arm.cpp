
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
// using namespace std;

// class ArmController {
// public:
//     ArmController(ros::NodeHandle& nh) {
//         pub_ = nh.advertise<llm_msgs::hand_pose_req>("/hand_pose_req", 1);
//         sub_ = nh.subscribe("desired_position", 10, &ArmController::positionCallback, this);
//         joint_state_sub = nh.subscribe("/human_arm_state_left", 10, &ArmController::jointStateCallback, this);

//         first();
//         ros::Rate r(200);
//          while (ros::ok()) {
//         // std::cout<<xx<<", "<<yy<<", "<<zz<<std::endl;  // 打印滤波后的数据
            
//             initializeGraspPoses();
//             publishGraspPoses();
//             // sleep(0.5);
//             ros::spinOnce();    
//             r.sleep();
//         }
//     }
//         struct ArmParam
// {
//     int n_joints;
//     std::string name;
//     std::string base_link;
//     std::string eef_link;
//     std::vector<std::string> joint_names;
//     std::vector<double> joint_pos;
//     std::vector<double> joint_vel;
//     std::vector<double> joint_eff;
//     std::vector<double> joints_min;
//     std::vector<double> joints_max;
//     int which_arm;//0-left, 1-right
//     double max_line_vel, max_rot_vel, max_line_acc, max_rot_acc, max_line_jerk, max_rot_jerk;

//     void resize()
//     {
//         joint_names.resize(n_joints);
//         joint_pos.resize(n_joints);
//         joint_vel.resize(n_joints);
//         joint_eff.resize(n_joints);
//         joints_min.resize(n_joints);
//         joints_max.resize(n_joints);
//     };
// };
//     void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//         // 处理接收到的 joint state 数据
//         joint_positions = msg->position; // 保存 joint position
//         joint_velocities = msg->velocity; // 保存 joint velocity
//         joint_efforts = msg->effort; // 保存 joint effort
//         std::string error_string;
//         // ROS_INFO("Current position = %f ,%f ,%f ",joint_positions[0],joint_positions[1],joint_positions[2]);          
//         Eigen::MatrixXd InitPoseT;
//         std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
//         urdf_path += "/models/AUBO_HRM.urdf";
//         arm_kinematics::ArmKinematicsSolver kin_solver(urdf_path, arm_param_.base_link, arm_param_.eef_link, arm_param_.joints_min, arm_param_.joints_max);

//         kin_solver.getFkSolution(InitPoseT, joint_positions, error_string);
//         // std::cout << "InitPoseT: " << std::endl << InitPoseT << std::endl;
//         L_grasp_pose[0]=InitPoseT(0,3);
//         L_grasp_pose[1]=InitPoseT(1,3);
//         L_grasp_pose[2]=InitPoseT(2,3);
//     }
//     void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
//         // 处理接收到的位置数据
//          xx=msg->x;
//          yy=msg->y;
//          zz=msg->z;
//         //  initializeGraspPoses();
//         // ROS_INFO("Received desired position: x: %.8f, y: %.8f, z: %.8f", msg->x, msg->y, msg->z);
//     }
//     ros::Publisher pub_;
//     ros::Subscriber sub_;
//     llm_msgs::hand_pose_req left_grasp_pose, right_grasp_pose;
//     ArmParam arm_param_;
//     ros::Subscriber joint_state_sub;
//     std::vector<double> joint_positions;
//     std::vector<double> joint_velocities;
//     std::vector<double> joint_efforts;
//     std::vector<double> L_grasp_pose{7}, R_grasp_pose{7};

//         // L_grasp_pose = {0.432496 - 0.035, 0.162388 + 0.015, 0.276278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
        
//         // R_grasp_pose = {0.384662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};
//     double xx;
//     double yy;
//     double zz;
//     void first(){

//         ROS_INFO("=======first========");
//         arm_param_.name == "left";
//         arm_param_.base_link="base_link";
//         arm_param_.eef_link="left_flange";
//         arm_param_.joints_min ={-3.1415926,0        ,-2.268928,-1.779,-2.268928,-0.6454,-0.6454};
//         arm_param_.joints_max ={0.7853981 ,1.7453292,2.2689280,0        , 2.268928, 0.6454, 0.6454};

//         // MatrixXd T_L = Quat2T_matrix(L_grasp_pose);
//         // MatrixXd T_change(4, 4);
//         // T_change << 0, -1, 0, 0,
//         //             1, 0, 0, 0,
//         //             0, 0, 1, 0,
//         //             0, 0, 0, 1;

//         // MatrixXd T_result = T_L * T_change;
//         L_grasp_pose = {0.432496 - 0.035, 0.162388 + 0.015, 0.276278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
        
//         R_grasp_pose = {0.384662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};
//         // 更新左抓握位置
//         left_grasp_pose.pose_req.position.x = L_grasp_pose[0];
//         left_grasp_pose.pose_req.position.y = L_grasp_pose[1];
//         left_grasp_pose.pose_req.position.z = L_grasp_pose[2];
//         left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];
//         left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4];
//         left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5];
//         left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6];

//         // 更新右抓握位置
//         right_grasp_pose.pose_req.position.x = R_grasp_pose[0];
//         right_grasp_pose.pose_req.position.y = R_grasp_pose[1];
//         right_grasp_pose.pose_req.position.z = R_grasp_pose[2];
//         right_grasp_pose.pose_req.orientation.x = R_grasp_pose[3];
//         right_grasp_pose.pose_req.orientation.y = R_grasp_pose[4];
//         right_grasp_pose.pose_req.orientation.z = R_grasp_pose[5];
//         right_grasp_pose.pose_req.orientation.w = R_grasp_pose[6];
//     }
//     void initializeGraspPoses() {
//         // L_grasp_pose = {0.432496 - 0.035+xx, 0.162388 + 0.015+yy, 0.276278 - 0.01+zz, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
//         // R_grasp_pose = {0.384662+xx, -0.109894+yy, 0.14731+zz, 0.7415988, -0.01539735, 0.21900877, -0.63390008};
//         double time = ros::Time::now().toSec(); // 获取当前时间

//         Eigen::Vector3d delta_xyz(xx,yy,zz);
//         // Eigen::Vector3d delta_xyz(0.0,0,0);

//         // Eigen:Matrix4d deltaMat;
//         // deltaMat << 0, -1, 0, 0,
//         //               1, 0, 0, 0,
//         //               0, 0, 1, 0,
//         //               0, 0, 0, 1;
        
//         // delta_xyz(0)=xx;
//         // delta_xyz(1)=yy;
//         // delta_xyz(2)=xx;
//         // std::cout<<"===delta_xyz=======:"<<delta_xyz.transpose()<<std::endl;

//         auto nowTcp2b= Quat2T_matrix(L_grasp_pose);
//         auto result = nowTcp2b*delta_xyz.homogeneous();
//         // ROS_INFO("====initializeGraspPoses======", left_grasp_pose.pose_req.position.x);
//         //std::cout<<"===Position=======:"<<result.transpose()<<std::endl;

//         L_grasp_pose[0] = result(0);
//         L_grasp_pose[1] = result(1);
//         L_grasp_pose[2] = result(2);

//         //  std::cout<<result(0)<<std::endl;

//         // L_grasp_pose[2] =result(1);
//         // L_grasp_pose[1] =result(2);
        
//         // std::cout<<"====================================="<<L_grasp_pose[0]<<std::endl;
//         // L_grasp_pose = {0.432496 - 0.035, 0.162388 + 0.015, 0.276278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
//         //R_grasp_pose = {0.384662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};

//         // MatrixXd T_L = Quat2T_matrix(L_grasp_pose);
//         // MatrixXd T_change(4, 4);
//         // T_change << 0, -1, 0, 0,
//         //             1, 0, 0, 0,
//         //             0, 0, 1, 0,
//         //             0, 0, 0, 1;

//         // MatrixXd T_result = T_L * T_change;

//         // 更新左抓握位置
//         left_grasp_pose.hand_move_enable=1;
//         left_grasp_pose.hand_side=0;
//         left_grasp_pose.hand_reset=1;
//         // left_grasp_pose.pose_req.position.x += xx;
//         // left_grasp_pose.pose_req.position.y += yy;
//         // left_grasp_pose.pose_req.position.z += zz;
//         left_grasp_pose.pose_req.position.x = L_grasp_pose[0];
//         left_grasp_pose.pose_req.position.y = L_grasp_pose[1];
//         left_grasp_pose.pose_req.position.z = L_grasp_pose[2];
//         left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];
//         left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4];
//         left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5];
//         left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6];

//         // 更新右抓握位置
//         // right_grasp_pose.pose_req.position.x += xx;
//         // right_grasp_pose.pose_req.position.y += yy;
//         // right_grasp_pose.pose_req.position.z += zz;
//         // right_grasp_pose.pose_req.orientation.x = R_grasp_pose[3];
//         // right_grasp_pose.pose_req.orientation.y = R_grasp_pose[4];
//         // right_grasp_pose.pose_req.orientation.z = R_grasp_pose[5];
//         // right_grasp_pose.pose_req.orientation.w = R_grasp_pose[6];
//     }

//     void publishGraspPoses() 
//     {
//             pub_.publish(left_grasp_pose);
//             // pub_.publish(right_grasp_pose);
       
//     }

//     // 此处需要实现 Quat2T_matrix 函数
//     Eigen::Matrix4d Quat2T_matrix(std::vector<double>& quat) 
//     {
//     Eigen::Vector3d posValue;
//     Eigen::Quaterniond quattt;

//     for (int i=0; i<3; i++)
//     {
//         posValue(i) = quat[i];
//     }

//     quattt.x() = quat[3];
//     quattt.y() = quat[4];
//     quattt.z() = quat[5];
//     quattt.w() = quat[6];

//     Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4);
//     T.block<3,3>(0,0) = quattt.toRotationMatrix();
//     T.block<3,1>(0,3) = posValue;

//     return T;
//     }
// };

// int main(int argc, char* argv[]) {
//     ros::init(argc, argv, "test_arm");
//     ros::NodeHandle nh;

//     ArmController arm_controller(nh);

//     return 0;
// }
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <llm_msgs/hand_pose_req.h>
#include <Eigen/Dense>
#include <array>
#include <vector>

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
        : filter_(5) {  // 初始化平滑滤波器，窗口大小为5
        pub_ = nh.advertise<llm_msgs::hand_pose_req>("/hand_pose_req", 10);
        sub_ = nh.subscribe("desired_pose", 10, &ArmController::positionCallback, this);
        joint_state_sub = nh.subscribe("/human_arm_state_left", 10, &ArmController::jointStateCallback, this);
        // joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);

        test_trajectory_pub_ = nh.advertise<geometry_msgs::Point>("desired_trajectory", 10); // 初始化话题发布者


        first();
        ros::Rate r(199);
        while (ros::ok()) {
            initializeGraspPoses();
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

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        joint_positions = msg->position; // 保存 joint position
        joint_velocities = msg->velocity; // 保存 joint velocity
        joint_efforts = msg->effort; // 保存 joint effort
        std::string error_string;

        Eigen::MatrixXd InitPoseT;
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        arm_kinematics::ArmKinematicsSolver kin_solver(urdf_path, arm_param_.base_link, arm_param_.eef_link, arm_param_.joints_min, arm_param_.joints_max);

        kin_solver.getFkSolution(InitPoseT, joint_positions, error_string);
        L_grasp_pose[0] = InitPoseT(0, 3);
        L_grasp_pose[1] = InitPoseT(1, 3);
        L_grasp_pose[2] = InitPoseT(2, 3);
    }

    void positionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        // 将接收到的位置数据传递给滤波器
        std::array<double, 3> input_position = {msg->position.x, msg->position.y, msg->position.z};
        auto filtered_position = filter_.filter(input_position);

        // 更新 xx, yy, zz 为滤波后的值
        xx = filtered_position[0];
        yy = filtered_position[1];
        zz = filtered_position[2];
        qx = msg->orientation.x;
        qy = msg->orientation.y;
        qz = msg->orientation.z;
        qw = msg->orientation.w;
    }

    void first() {
        ROS_INFO("=======first========");
        arm_param_.name = "left";
        arm_param_.base_link = "base_link";
        arm_param_.eef_link = "left_flange";
        // arm_param_.joints_min = {-3.1415926, 0, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
        arm_param_.joints_min ={-3.1415926,-0.174   ,-2.268928,-2.267,-3.14,-0.6454,-0.6454};
        arm_param_.joints_max ={1.7853981 ,1.7453292,2.2689280,0        , 3.14, 0.6454, 0.6454};

        L_grasp_pose = {0.432496-0.035, 0.162388 + 0.015, 0.276278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
        // L_grasp_pose = {0.505169,0.323111,0.249817,0.0872157,0.631405,-0.11521,0.761871};

        R_grasp_pose = {0.384662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};

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
        
        pub_.publish(left_grasp_pose);
        


        // sleep(2);
        // 更新左抓握位置
        updateGraspPoses(left_grasp_pose, L_grasp_pose);
        // 更新右抓握位置
        updateGraspPoses(right_grasp_pose, R_grasp_pose);
    }

    void updateGraspPoses(llm_msgs::hand_pose_req& grasp_pose, const std::vector<double>& grasp_pose_vals) {
        grasp_pose.pose_req.position.x = grasp_pose_vals[0];
        grasp_pose.pose_req.position.y = grasp_pose_vals[1];
        grasp_pose.pose_req.position.z = grasp_pose_vals[2];
        grasp_pose.pose_req.orientation.x = grasp_pose_vals[3];
        grasp_pose.pose_req.orientation.y = grasp_pose_vals[4];
        grasp_pose.pose_req.orientation.z = grasp_pose_vals[5];
        grasp_pose.pose_req.orientation.w = grasp_pose_vals[6];
    }

    void initializeGraspPoses() 
    {
        double radius = 0.11; // 圆的半径
        double angular_velocity = 0.15; // 角速度
        double time = ros::Time::now().toSec(); // 当前时间
        Eigen::Vector3d ddddd;
        ddddd(0) = radius * cos(angular_velocity * time);
        ddddd(1) = radius * sin(angular_velocity * time);
        ddddd(2) = 0;
        Eigen::Vector3d delta_xyz(xx, yy, zz);
        auto nowTcp2b = Quat2T_matrix(L_grasp_pose);
        // auto result = nowTcp2b * delta_xyz.homogeneous();
        auto result = nowTcp2b.block<3,3>(0,0) * delta_xyz;
        ddddd=nowTcp2b.block<3,3>(0,0) * delta_xyz;

        // L_grasp_pose[0] = result(0);
        // L_grasp_pose[1] = result(1);
        // L_grasp_pose[2] = result(2);

        left_grasp_pose.hand_move_enable = 1;
        left_grasp_pose.hand_side = 0;
        left_grasp_pose.hand_reset = 1;
        // left_grasp_pose.pose_req.position.x = L_grasp_pose[0]+d_x;
        // left_grasp_pose.pose_req.position.y = L_grasp_pose[1]+d_y;
        // left_grasp_pose.pose_req.position.z = L_grasp_pose[2];
        left_grasp_pose.pose_req.position.x = L_grasp_pose[0]+result(0)+ddddd(0);
        left_grasp_pose.pose_req.position.y = L_grasp_pose[1]+result(1)+ddddd(1);
        left_grasp_pose.pose_req.position.z = L_grasp_pose[2]+result(2)+ddddd(2);;
        left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];
        left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4];
        left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5];
        left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6];
        left_grasp_pose.pose_req.orientation.x = qx;
        left_grasp_pose.pose_req.orientation.y = qy;
        left_grasp_pose.pose_req.orientation.z = qz;
        left_grasp_pose.pose_req.orientation.w = qw;

        geometry_msgs::Point trajectory_point;

        trajectory_point.x=L_grasp_pose[0];
        trajectory_point.y=L_grasp_pose[1];
        trajectory_point.z=L_grasp_pose[2];
        test_trajectory_pub_.publish(trajectory_point);
    }

    void publishGraspPoses() {
        pub_.publish(left_grasp_pose);
        // pub_.publish(right_grasp_pose); // 如果需要右手抓握位置，可以取消注释
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
    ros::Publisher pub_;
    ros::Publisher test_trajectory_pub_;
    ros::Subscriber sub_;
    ros::Subscriber joint_state_sub;
    llm_msgs::hand_pose_req left_grasp_pose, right_grasp_pose;
    ArmParam arm_param_;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    std::vector<double> L_grasp_pose{7}, R_grasp_pose{7};

    double xx;
    double yy;
    double zz;
    double qx;
    double qy;
    double qz;
    double qw;

    SmoothingFilter filter_; // 添加平滑滤波器
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test_arm");
    ros::NodeHandle nh;

    ArmController arm_controller(nh);

    return 0;
}
