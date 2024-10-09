#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <arm_kinematics_solver/arm_kinematics_solver.h>
#include <aubo_arm_planning/arm_qpik_solver.h>
#include <sensor_msgs/JointState.h>
#include "trajectory_planner/traj_planner_base.h"
#include "trajectory_planner/traj_s_planner.h"
#include "trajectory_planner/traj_cp_planner.h"
#include <llm_msgs/hand_pose_req.h>
#include <llm_msgs/pose_action_status.h>
#include <stdint.h>

struct ArmParam
{
    int n_joints;
    std::string name;
    std::string base_link;
    std::string eef_link;
    std::vector<std::string> joint_names;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joints_min;
    std::vector<double> joints_max;
    int which_arm;//0-left, 1-right
    int arm_move_enable;
    void resize()
    {
        joint_names.resize(n_joints);
        joint_pos.resize(n_joints);
        joint_vel.resize(n_joints);
        joints_min.resize(n_joints);
        joints_max.resize(n_joints);
    };
};

class DualArm
{
    public:
        DualArm(ros::NodeHandle& nh);
        ~DualArm(){};
        void LeftJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
        void RightJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

        void LeftJoints_pub(const std::vector<double> pos, const Eigen::VectorXd vel, ros::Publisher pub);
        void RightJoints_pub(const std::vector<double> pos, const Eigen::VectorXd vel, ros::Publisher pub);
        void LeftGraspPose_CallBack(const llm_msgs::hand_pose_req::ConstPtr& grasp_pos);
        void RightGraspPose_CallBack(const llm_msgs::hand_pose_req::ConstPtr& grasp_pos);

        bool Judge_arrived(std::vector<double> cur_pose,std::vector<double> end_grasp_pose);
        // void Cartesian_Plan(std::vector<double> &goal_pose , ArmParam &arm_param_,uint32_t id);
        // void Joints_Plan(std::vector<double> &goal_joints );
        void Right_Cartesian_Plan(std::vector<double> &goal_pose , ArmParam &arm_param_,uint32_t id);
        void Left_Cartesian_Plan(std::vector<double> &goal_pose , ArmParam &arm_param_,uint32_t id);
        
        void LeftJoints_Plan(std::vector<double> &goal_joints);
        void RightJoints_Plan(std::vector<double> &goal_joints);
        void Wave_Arm(std::vector<double> &goal_joints);
        bool Arrrived_GoalPose(arm_kinematics::ArmKinematicsSolver &kin_solver,ArmParam &arm_param_,std::vector<double> &goal_pose,uint32_t id);
        std::vector<Eigen::Matrix4d> minimumJerkInterpolationCartesian(const Eigen::Matrix4d& start, const Eigen::Matrix4d& end, double duration, int num_samples);
        ArmParam left_arm_param_, right_arm_param_;
        Traj_S_Planner *trajSPlannerObj; 
        
    private:
        
        ros::Publisher  left_joints_publisher;
        ros::Publisher  right_joints_publisher;

        ros::Publisher  arrive_pose_publisher;
        // ros::Subscriber grasp_pose_subscriber;
        ros::Subscriber left_grasp_pose_subscriber;
        ros::Subscriber right_grasp_pose_subscriber;

        ros::Publisher test_trajectory_pub_; // test

        ros::Subscriber left_joints_subscriber;
        ros::Subscriber right_joints_subscriber;
        ros::Publisher  joint_state_pub;
        std::unique_ptr<ros::AsyncSpinner> spinner_;
        double frequency;
        std::string error_string;
};
