#ifndef TEST_ARM_H  
#define TEST_ARM_H  
  
#include <ros/ros.h>  
#include <geometry_msgs/Wrench.h>  
#include <geometry_msgs/WrenchStamped.h>  
#include <geometry_msgs/Vector3.h>  
#include <sensor_msgs/JointState.h>  
#include <llm_msgs/hand_pose_req.h>  
#include <Eigen/Dense>  
#include <array>  
#include <vector>  
#include <arm_kinematics_solver/arm_kinematics_solver.h>  
  
#define SIM  
  
class SmoothingFilter {  
public:  
    SmoothingFilter(size_t windowSize);  
    std::array<double, 3> filter(const std::array<double, 3>& input_force);  
  
private:  
    size_t windowSize;  
    std::vector<std::array<double, 3>> values;  
    int index;  
};  
  
class ArmController {  
public:  
    struct ArmParam 
    {  
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
  
        void resize();  
    };  
  
    ArmController(ros::NodeHandle& nh);  
    Eigen::VectorXd optimizeNullSpaceIncrement(  Eigen::MatrixXd& jaco, 
                                                Eigen::VectorXd& joint_angles, 
                                                Eigen::VectorXd& delta_q_desired, 
                                                Eigen::VectorXd& joint_limits_min, 
                                                Eigen::VectorXd& joint_limits_max, 
                                                Eigen::VectorXd& prev_delta_q, 
                                            double weight_limit_avoidance, 
                                            double weight_singularity_avoidance, 
                                            double weight_smoothness);
    void first();
    void test_jacob();  
    //遥操作叠加雅克比计算出的关节导纳速度
    void right_jacob();
    void left_jacob ();
    void Joints_pub(std::vector<double> pos, ros::Publisher pub);
    //仿真用
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);  
    //获取编码器状态
    void jointStateCallback_left(const sensor_msgs::JointState::ConstPtr& msg);
    void jointStateCallback_right(const sensor_msgs::JointState::ConstPtr& msg);  
    //接收遥操作信号  
    void jointStateCallback_right_teleoperation(const sensor_msgs::JointState::ConstPtr& msg);  
    void jointStateCallback_left_teleoperation(const sensor_msgs::JointState::ConstPtr& msg);  
    void positionCallback_left(const geometry_msgs::Pose::ConstPtr& msg);  
    void positionCallback_right(const geometry_msgs::Pose::ConstPtr& msg);  
    Eigen::Matrix4d Quat2T_matrix(const std::vector<double>& quat);  
  
private:  
    ros::Publisher left_arm_pub_;  
    ros::Publisher left_joints_publisher;
    ros::Publisher right_joints_publisher;
    ros::Publisher right_arm_pub_;  
    ros::Publisher joint_pub;  
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
    std::vector<double> L_grasp_pose;  
    std::vector<double> R_grasp_pose;  
  
    double xx_left, yy_left, zz_left, qx_left, qy_left, qz_left, qw_left;  
    double xx_right, yy_right, zz_right, qx_right, qy_right, qz_right, qw_right;  
    bool first_R_, first_L_;  
  
    SmoothingFilter filter_left;  
    SmoothingFilter filter_right;  
    Eigen::VectorXd q_dot_last = Eigen::VectorXd::Zero(7);
    std::vector<double> joint_positions_L={0,0,0,0,0,0,0};  
    std::vector<double> joint_positions_R={0,0,0,0,0,0,0};  
    std::vector<double>  desired_joint_L={0,0,0,0,0,0,0};  
    std::vector<double>  desired_joint_R={0,0,0,0,0,0,0};  
    std::vector<double> left_min={-3.1415926, -0.2, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
    std::vector<double> left_max={0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};
    std::vector<double> right_min={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
    std::vector<double> right_max={3.1415926  ,0.2         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454}; 
};  
  
#endif // TEST_ARM_H