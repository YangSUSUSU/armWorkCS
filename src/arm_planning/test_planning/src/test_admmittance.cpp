#include<iostream>
#include<ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include"llm_msgs/hand_pose_req.h"
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include <arm_kinematics_solver/arm_kinematics_solver.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace Eigen;


class AdmittanceController {
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
        int which_arm;//0-left, 1-right
        double max_line_vel, max_rot_vel, max_line_acc, max_rot_acc, max_line_jerk, max_rot_jerk;

        void resize()
        {
            joint_names.resize(n_joints);
            joint_pos.resize(n_joints);
            joint_vel.resize(n_joints);
            joint_eff.resize(n_joints);
            joints_min.resize(n_joints);
            joints_max.resize(n_joints);
        };
    };


    AdmittanceController(ros::NodeHandle& nh) {
        // 初始化订阅者和发布者
        force_sub_ = nh.subscribe("filtered_force", 100, &AdmittanceController::forceCallback, this);
        joint_state_sub = nh.subscribe("/human_arm_state_left", 10, &AdmittanceController::jointStateCallback, this);
                // joint_state_sub = nh.subscribe("/joint_states", 10, &AdmittanceController::jointStateCallback, this);

        position_pub_ = nh.advertise<geometry_msgs::Vector3>("desired_position", 100);

    }

    /*单自由度的导纳*/
    double admittanceController_torque()
    {
        
    }
    double admittanceController_force(double last, double now, double force)
    {
    const double M = 10.125;       //虚拟质量
    const double B = 10.28;   //阻尼
    double T=0.005;             //控制周期
    double dv= (now-last)/T;
    double dxe=last-now-(-dv)*T/2;  //减去自身期望运动分量
    return (force-B*dxe)/M;    //返回 ddxe
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

        arm_param_.name == "left";
        arm_param_.base_link="base_link";
        arm_param_.eef_link="left_flange";
        arm_param_.joints_min ={-3.1415926,0        ,-2.268928,-1.779,-2.268928,-0.6454,-0.6454};
        arm_param_.joints_max ={0.7853981 ,1.7453292,2.2689280,0        , 2.268928, 0.6454, 0.6454};
                if(first_force_received)
        {
            sensorFrameMatrix<<0,-1,0,
                              1,0,0,
                              0,0, 1;
            // auto temp=sensorFrameMatrix.Eigen::inverse();
            // sensorFrameMatrix=temp;
            // sensorFrameMatrix<<1,0,0,
            //                     0,1,0,
            //                     0,0,1;
            first_force_received=false;
            Eigen::Vector3d force;
            force(0)=msg->wrench.force.x;
            force(1)=msg->wrench.force.y;
            force(2)=msg->wrench.force.z;
            force=sensorFrameMatrix*force;
            first_force_x=-force(0);
            first_force_y=-force(1);
            first_force_z=force(2);
            // slee
            position_pub_.publish(desired_position_);

        }
        else
        {
            Eigen::Vector3d force;
            force(0)=msg->wrench.force.x;
            force(1)=msg->wrench.force.y;
            force(2)=msg->wrench.force.z;
            force=sensorFrameMatrix*force;
            force(0)=-force(0);
            force(1)=-force(1);

            //std::cout << "===Position=======: " << position.transpose() << std::endl;

       // 获取施加的力
        double fx = force(0)-first_force_x;
        double fy = force(1)-first_force_y;
        double fz = force(2)-first_force_z;

        double delta_x;
        delta_x=admittanceController_force(last_x, now_x, fx);
        last_x=now_x;
        double delta_y;
        delta_y=admittanceController_force(last_y, now_y, fy);
        last_y=now_y;
        double delta_z;
        delta_z=admittanceController_force(last_z, now_z, fz);
        last_z=now_z;

        desired_position_.x = 0.001*(delta_x);
        desired_position_.y = 0.001*(delta_y);
        desired_position_.z = 0.001*(delta_z);

        //更新期望位置
        // desired_position_.x = 0.001*(position(0));
        // desired_position_.y = 0.001*(position(1));
        // desired_position_.z = 0.001*(position(2));

        // std::cout<<desired_position_.x<<", "<<desired_position_.y<<", "<<desired_position_.z<<std::endl;  // 打印滤波后的数据

        // 发布期望位置
        position_pub_.publish(desired_position_);
        }
 
    }
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 处理接收到的 joint state 数据
        joint_positions = msg->position; // 保存 joint position
        joint_velocities = msg->velocity; // 保存 joint velocity
        joint_efforts = msg->effort; // 保存 joint effort
        std::string error_string;
        arm_param_.name == "left";
        arm_param_.base_link="base_link";
        arm_param_.eef_link="left_flange";
        arm_param_.joints_min ={-3.1415926,0        ,-2.268928,-1.779,-2.268928,-0.6454,-0.6454};
        arm_param_.joints_max ={0.7853981 ,1.7453292,2.2689280,0        , 2.268928, 0.6454, 0.6454};
                
        Eigen::MatrixXd InitPoseT;
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        arm_kinematics::ArmKinematicsSolver kin_solver(urdf_path, arm_param_.base_link, arm_param_.eef_link, arm_param_.joints_min, arm_param_.joints_max);

        kin_solver.getFkSolution(InitPoseT, joint_positions, error_string);
        std::cout << "InitPoseT: " << InitPoseT(0,3)<<InitPoseT(1,3)<<InitPoseT(2,3) << std::endl;
        now_x=InitPoseT(0,3);
        now_y=InitPoseT(1,3);
        now_z=InitPoseT(2,3);
        //
    }

private:
    ros::Subscriber force_sub_;
    ros::Publisher position_pub_;
    ros::Publisher joint_state_pub;
    ros::Subscriber joint_state_sub;
    Eigen::Matrix3d sensorFrameMatrix;

    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    
    geometry_msgs::Vector3 desired_position_;
    double mass_;
    double damping_;
    double stiffness_;

    double last_x=0.0;
    double last_y=0.0;
    double last_z=0.0;

    double now_x=0.0;
    double now_y=0.0;
    double now_z=0.0;

    double first_force_x=0.0;
    double first_force_y=0.0;
    double first_force_z=0.0;
    bool first_force_received=true;
    ArmParam arm_param_;
    // arm_kinematics::ArmKinematicsSolver *arm_solver;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "admittance_controller");
    ros::NodeHandle nh;
    
    AdmittanceController controller(nh);
    
    ros::spin();
    return 0;
}
