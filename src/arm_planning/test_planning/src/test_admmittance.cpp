#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include "llm_msgs/hand_pose_req.h"
#include <eigen3/Eigen/Eigen>
#include <arm_kinematics_solver/arm_kinematics_solver.h>
#include <sensor_msgs/JointState.h>


// 定义仿真模式
// #define SIMULATION_MODE

// 定义六个导纳控制的常量，通过条件编译区分仿真和真机模式
#ifdef SIMULATION_MODE
    // 仿真模式下的力 mbk 参数
    #define FORCE_MASS 0.5        // 力控制中的质量 (kg)
    #define FORCE_DAMPING 0.1     // 力控制中的阻尼 (Ns/m)
    #define FORCE_STIFFNESS 50.0  // 力控制中的刚度 (N/m)

    // 仿真模式下的力矩 mbk 参数
    #define TORQUE_MASS 0.1       // 力矩控制中的质量 (kg·m²)
    #define TORQUE_DAMPING 0.05   // 力矩控制中的阻尼 (Ns·m/rad)
    #define TORQUE_STIFFNESS 20.0 // 力矩控制中的刚度 (N·m/rad)
#else
    // 真机模式下的力 mbk 参数
    #define FORCE_MASS 0.175        // 力控制中的质量 (kg)
    #define FORCE_DAMPING 0.175     // 力控制中的阻尼 (Ns/m)
    #define FORCE_STIFFNESS 100.0 // 力控制中的刚度 (N/m)

    // 真机模式下的力矩 mbk 参数
    #define TORQUE_MASS 0.075       // 力矩控制中的质量 (kg·m²)
    #define TORQUE_DAMPING 0.075    // 力矩控制中的阻尼 (Ns·m/rad)
    #define TORQUE_STIFFNESS 40.0 // 力矩控制中的刚度 (N·m/rad)
#endif


using namespace std;
using namespace Eigen;

class ArmController {
public:
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
        double max_line_vel, max_rot_vel, max_line_acc, max_rot_acc, max_line_jerk, max_rot_jerk;

        void resize() {
            joint_names.resize(n_joints);
            joint_pos.resize(n_joints);
            joint_vel.resize(n_joints);
            joint_eff.resize(n_joints);
            joints_min.resize(n_joints);
            joints_max.resize(n_joints);
        }
    };

    ArmController(ros::NodeHandle& nh, const std::string& arm_name) : arm_param_{}, first_force_received(true), last_x(0.0), last_y(0.0), last_z(0.0) {
        arm_param_.name = arm_name;
        initializeArmParams();
        
        force_sub_ = nh.subscribe("/filtered_force_"+arm_name , 10, &ArmController::forceCallback, this);
        #ifdef SIMULATION_MODE
            joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);
        #else
            joint_state_sub = nh.subscribe("/human_arm_state_"+arm_name , 10, &ArmController::jointStateCallback, this);

        #endif
        position_pub_ = nh.advertise<geometry_msgs::Pose>("/desired_pose_"+arm_name , 100);
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        // Force and torque transformation
        if (first_force_received) {
            initializeForceTorque(msg);
            first_force_received = false;
        } else {
            updateForceTorque(msg);
            publishDesiredPose();
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        joint_positions = msg->position;

        // ROS_INFO("Header:");
        // ROS_INFO("  seq: %d", msg->header.seq);
        // ROS_INFO("  stamp: %d.%d", msg->header.stamp.sec, msg->header.stamp.nsec);
        // ROS_INFO("  frame_id: %s", msg->header.frame_id.c_str());

        // // 打印关节名称
        // ROS_INFO("Joint Names:");
        // for (size_t i = 0; i < msg->name.size(); ++i) {
        //     ROS_INFO("  Name[%lu]: %s", i, msg->name[i].c_str());
        // }

        // // 打印关节位置
        // ROS_INFO("Joint Positions:");
        // for (size_t i = 0; i < msg->position.size(); ++i) {
        //     ROS_INFO("  Position[%lu]: %f", i, msg->position[i]);
        // }

        // // 打印关节速度
        // ROS_INFO("Joint Velocities:");
        // for (size_t i = 0; i < msg->velocity.size(); ++i) {
        //     ROS_INFO("  Velocity[%lu]: %f", i, msg->velocity[i]);
        // }

        // // 打印关节力矩
        // ROS_INFO("Joint Efforts:");
        // for (size_t i = 0; i < msg->effort.size(); ++i) {
        //     ROS_INFO("  Effort[%lu]: %f", i, msg->effort[i]);
        // }
        Eigen::MatrixXd InitPoseT;
        std::string error_string;
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        arm_kinematics::ArmKinematicsSolver kin_solver(urdf_path, arm_param_.base_link, arm_param_.eef_link, arm_param_.joints_min, arm_param_.joints_max);
        kin_solver.getFkSolution(InitPoseT, joint_positions, error_string);

        now_R = InitPoseT.block<3, 3>(0, 0);
        now_x = InitPoseT(0, 3);
        now_y = InitPoseT(1, 3);
        now_z = InitPoseT(2, 3);
    }

private:
    void initializeArmParams() {
        if (arm_param_.name == "left") {
            arm_param_.base_link = "base_link";
            arm_param_.eef_link = "left_flange";
            arm_param_.joints_min = {-3.1415926, 0, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
            arm_param_.joints_max = {0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};
        } else if (arm_param_.name == "right") {
            arm_param_.base_link = "base_link";
            arm_param_.eef_link = "right_flange";
            arm_param_.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
            arm_param_.joints_max ={3.1415926  ,0         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};
        }
    }

    void initializeForceTorque(const geometry_msgs::WrenchStamped::ConstPtr& msg) {

    // 打印完整的 WrenchStamped 消息
    // ROS_INFO("WrenchStamped Message:");
    // ROS_INFO("Header: seq: %d, stamp: [sec: %d, nsec: %d], frame_id: %s", 
    //          msg->header.seq, 
    //          msg->header.stamp.sec, 
    //          msg->header.stamp.nsec, 
    //          msg->header.frame_id.c_str());

    // ROS_INFO("Force: [x: %.2f, y: %.2f, z: %.2f]", 
    //          msg->wrench.force.x, 
    //          msg->wrench.force.y, 
    //          msg->wrench.force.z);
             
    // ROS_INFO("Torque: [x: %.2f, y: %.2f, z: %.2f]", 
    //          msg->wrench.torque.x, 
    //          msg->wrench.torque.y, 
    //          msg->wrench.torque.z); 

        #ifdef SIMULATION_MODE

        sensorFrameMatrix << 1, 0, 0,
                            0, 1, 0,
                            0, 0, 1;
        Eigen::Vector3d force;
        force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
        force = sensorFrameMatrix * force;

        first_force_x = 0;
        first_force_y = 0;
        first_force_z = 0;

        Eigen::Vector3d torque;
        torque << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
        torque = sensorFrameMatrix * torque;

        first_torque_x = torque(0);
        first_torque_y = torque(1);
        first_torque_z = torque(2);

        #else
            if (arm_param_.name == "left") 
            {
               
            sensorFrameMatrix << 0, -1, 0,
                                 1, 0, 0,
                                 0, 0, 1;
            Eigen::Vector3d force;
            force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
            force = sensorFrameMatrix * force;

            first_force_x = -force(0) * 30;
            first_force_y = -force(1) * 30;
            first_force_z =  force(2) * 30;

            Eigen::Vector3d torque;
            torque << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
            torque = sensorFrameMatrix * torque;
            first_torque_x = -torque(0)*30;
            first_torque_y = -torque(1)*30;
            first_torque_z =  torque(2)*30;
            }
            else
            {
            sensorFrameMatrix << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;
            Eigen::Vector3d force;
            force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
            force = sensorFrameMatrix * force;

            first_force_x = -force(0) * 30;
            first_force_y = -force(1) * 30;
            first_force_z = force(2) * 30;

            Eigen::Vector3d torque;
            torque << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
            torque = sensorFrameMatrix * torque;

            first_torque_x = -torque(0)*30;
            first_torque_y = -torque(1)*30;
            first_torque_z =  torque(2)*30;
            }

        #endif
       

    }

    void updateForceTorque(const geometry_msgs::WrenchStamped::ConstPtr& msg) 
    {

         #ifdef SIMULATION_MODE
                Eigen::Vector3d force;
                force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
                force = sensorFrameMatrix * force;

                Eigen::Vector3d torque;
                torque << msg->wrench.torque.x - first_torque_x, msg->wrench.torque.y - first_torque_y, msg->wrench.torque.z - first_torque_z;
                torque = sensorFrameMatrix * torque;

                force(0) = force(0);
                force(1) = force(1);
                torque(0) = torque(0);
                torque(1) = torque(1);

                double fx = force(0) - first_force_x;
                double fy = force(1) - first_force_y;
                double fz = force(2) - first_force_z;

                double delta_x = admittanceController_force(last_x, now_x, fx);
                last_x = now_x;
                double delta_y = admittanceController_force(last_y, now_y, fy);
                last_y = now_y;
                double delta_z = admittanceController_force(last_z, now_z, fz);
                last_z = now_z;

                double temp_tor[3]={0};
                temp_tor[0]=torque(0);
                temp_tor[1]=torque(1);
                temp_tor[2]=torque(2);
                auto result_ori = new_admittanceController_torque(temp_tor);

                // auto result_ori = admittanceController_torque(last_R, now_R, torque);
                last_R = now_R;
                double test_force[3]={0};
                double test_result[3]={0};
                test_force[0]=fx;
                test_force[1]=fy;
                test_force[2]=fz;
                // std::cout<<test_force[ 0]<<","<<test_force[ 1]<<","<<test_force[ 2]<<std::endl;
                double test_now[3];
                double test_last[3];
                test_now[0]=now_x;
                test_now[1]=now_y;
                test_now[2]=now_z;
                test_last[0]=last_x;
                test_last[1]=last_y;
                test_last[2]=last_z;
                if (arm_param_.name == "left") 
                {
                // pose_msg.position.x = 0.001 * delta_x;  // 设置位置 x
                // pose_msg.position.y = 0.001 * delta_y;  // 设置位置 y
                // pose_msg.position.z = 0.001 * delta_z;  // 设置位置 z
                    new_admittanceController(test_force,test_result);
                    pose_msg.position.x = 0.001 * test_result[0];  // 设置位置 x
                    pose_msg.position.y = 0.001 * test_result[1];  // 设置位置 y
                    pose_msg.position.z = 0.001 * test_result[2];  // 设置位置 z
                } 
                else 
                {
                    new_admittanceController(test_force,test_result);
                    pose_msg.position.x = 0.001 * test_result[0];  // 设置位置 x
                    pose_msg.position.y = 0.001 * test_result[1];  // 设置位置 y
                    pose_msg.position.z = 0.001 * test_result[2];  // 设置位置 z

                }

               
                // pose_msg.position.x = 0.001 * delta_x;  // 设置位置 x
                // pose_msg.position.y = 0.001 * delta_y;  // 设置位置 y
                // pose_msg.position.z = 0.001 * delta_z;  // 设置位置 z

                pose_msg.orientation.x = result_ori.x();  // 设置姿态四元数 x
                pose_msg.orientation.y = result_ori.y();  // 设置姿态四元数 y
                pose_msg.orientation.z = result_ori.z();  // 设置姿态四元数 z
                pose_msg.orientation.w = result_ori.w();    // 设置姿态四元数 w
         #else
            if (arm_param_.name == "left") 
            {
                Eigen::Vector3d force;
                force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
                force = sensorFrameMatrix * force;

                Eigen::Vector3d torque;
                torque << msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
                torque = sensorFrameMatrix * torque;

                force(0) = -force(0);
                force(1) = -force(1);
                torque(0) = -torque(0)-first_torque_x;
                torque(1) = -torque(1)-first_torque_y;
                torque(2) =  torque(2)-first_torque_z;
                
                
                double fx = force(0) - first_force_x;
                double fy = force(1) - first_force_y;
                double fz = force(2) - first_force_z;

                double delta_x = admittanceController_force(last_x, now_x, fx);
                last_x = now_x;
                double delta_y = admittanceController_force(last_y, now_y, fy);
                last_y = now_y;
                double delta_z = admittanceController_force(last_z, now_z, fz);
                last_z = now_z;

                // auto result_ori = admittanceController_torque(last_R, now_R, torque);
                last_R = now_R;
                double temp_tor[3]={0};
                temp_tor[0]=torque(0);
                temp_tor[1]=torque(1);
                temp_tor[2]=torque(2);
                auto result_ori = new_admittanceController_torque(temp_tor);


                double test_force[3]={0};
                double test_result[3]={0};
                test_force[0]=fx;
                test_force[1]=fy;
                test_force[2]=fz;
                // std::cout<<test_force[ 0]<<","<<test_force[ 1]<<","<<test_force[ 2]<<std::endl;
                double test_now[3];
                double test_last[3];
                test_now[0]=now_x;
                test_now[1]=now_y;
                test_now[2]=now_z;
                test_last[0]=last_x;
                test_last[1]=last_y;
                test_last[2]=last_z;

                new_admittanceController(test_force,test_result);
                pose_msg.position.x = 0.001 * test_result[0];  // 设置位置 x
                pose_msg.position.y = 0.001 * test_result[1];  // 设置位置 y
                pose_msg.position.z = 0.001 * test_result[2];  // 设置位置 z

                // pose_msg.position.x = 0.001 * delta_x;  // 设置位置 x
                // pose_msg.position.y = 0.001 * delta_y;  // 设置位置 y
                // pose_msg.position.z = 0.001 * delta_z;  // 设置位置 z

                pose_msg.orientation.x = result_ori.x();  // 设置姿态四元数 x
                pose_msg.orientation.y = result_ori.y();  // 设置姿态四元数 y
                pose_msg.orientation.z = result_ori.z();  // 设置姿态四元数 z
                pose_msg.orientation.w = result_ori.w();    // 设置姿态四元数 w
            }
            else
            {
                Eigen::Vector3d force;
                force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
                force = sensorFrameMatrix * force;

                Eigen::Vector3d torque;
                torque << msg->wrench.torque.x , msg->wrench.torque.y , msg->wrench.torque.z ;
                torque = sensorFrameMatrix * torque;

                force(0) = -force(0);
                force(1) = -force(1);
                // torque(0) = -torque(0);
                // torque(1) = -torque(1);
                // torque(2) = torque(2);

                torque(0) = -torque(0)-first_torque_x;
                torque(1) = -torque(1)-first_torque_y;
                torque(2) =  torque(2)-first_torque_z;

                double fx = force(0) - first_force_x;
                double fy = force(1) - first_force_y;
                double fz = force(2) - first_force_z;

                double delta_x = admittanceController_force(last_x, now_x, fx);
                last_x = now_x;
                double delta_y = admittanceController_force(last_y, now_y, fy);
                last_y = now_y;
                double delta_z = admittanceController_force(last_z, now_z, fz);
                last_z = now_z;

                auto result_ori = admittanceController_torque(last_R, now_R, torque);
                last_R = now_R;
                double temp_tor[3]={0};
                temp_tor[0]=torque(0);
                temp_tor[1]=torque(1);
                temp_tor[2]=torque(2);
                // auto result_ori = new_admittanceController_torque(temp_tor);

                double test_force[3]={0};
                double test_result[3]={0};
                test_force[0]=fx;
                test_force[1]=fy;
                test_force[2]=fz;
                // std::cout<<test_force[ 0]<<","<<test_force[ 1]<<","<<test_force[ 2]<<std::endl;
                double test_now[3];
                double test_last[3];
                test_now[0]=now_x;
                test_now[1]=now_y;
                test_now[2]=now_z;
                test_last[0]=last_x;
                test_last[1]=last_y;
                test_last[2]=last_z;

                new_admittanceController(test_force,test_result);
                pose_msg.position.x = 0.001 * test_result[0];  // 设置位置 x
                pose_msg.position.y = 0.001 * test_result[1];  // 设置位置 y
                pose_msg.position.z = 0.001 * test_result[2];  // 设置位置 z

                // pose_msg.position.x = 0.001 * delta_x;  // 设置位置 x
                // pose_msg.position.y = 0.001 * delta_y;  // 设置位置 y
                // pose_msg.position.z = 0.001 * delta_z;  // 设置位置 z

                pose_msg.orientation.x = result_ori.x();  // 设置姿态四元数 x
                pose_msg.orientation.y = result_ori.y();  // 设置姿态四元数 y
                pose_msg.orientation.z = result_ori.z();  // 设置姿态四元数 z
                pose_msg.orientation.w = result_ori.w();    // 设置姿态四元数 w

            }
         #endif
 
    }

    void publishDesiredPose() {

        // ROS_INFO("[%s] Publishing Pose message: Position[x: %.4f, y: %.4f, z: %.4f], Orientation[x: %.4f, y: %.4f, z: %.4f, w: %.4f]",
        //  (arm_param_.name).c_str(),  // 打印 arm_name
        //  pose_msg.position.x, 
        //  pose_msg.position.y, 
        //  pose_msg.position.z, 
        //  pose_msg.orientation.x, 
        //  pose_msg.orientation.y, 
        //  pose_msg.orientation.z, 
        //  pose_msg.orientation.w);
        position_pub_.publish(pose_msg);  // 发布消息
    }

    double admittanceController_force(double last, double now, double force) {
        const double M = FORCE_MASS;  // 虚拟质量
        const double B = FORCE_DAMPING; // 阻尼
        double T = 0.005;      // 控制周期
        double dv = (now - last) / T;
        double dxe = last - now - (-dv) * T / 2;
        return (force - B * dxe) / M; // 返回 ddxe
    }
    void new_admittanceController(double *TCP_force,double *result)
    {
        double dt = 0.005;
        const double M = FORCE_MASS;  // 虚拟质量
        const double B = FORCE_DAMPING; // 阻尼
        for ( int i=0; i < 3; i++ )
        {
            // force_vel_offset[i]=(now[i]-last[i])/dt;
            force_acc_offset[ i ] = ( TCP_force[ i ] - B * force_vel_offset[ i ]) / M;
            force_vel_offset[ i ] = dt * ( force_acc_offset[ i ] + force_last_acc_offset[ i ] ) / 2.0 + force_vel_offset[ i ];
            // std::cout<<force_acc_offset[i]<<std::endl;

            // std::cout<<result[i]<<std::endl;

            force_last_acc_offset[ i ] = force_acc_offset[ i ];
            //最后输出
            force_last_vel_offset[ i ] = force_vel_offset[ i ];

            result[i]=force_vel_offset[ i ];
            // result[i]=dt*force_acc_offset[ i ];


        }
        // std::cout<<result[0]<<","<<result[1]<<","<<result[2]<<std::endl;
    }
    Eigen::Quaterniond  new_admittanceController_torque(double *TCP_torque)
    {
        double dt = 0.005;
        const double M = TORQUE_MASS;       //虚拟质量
        const double B = TORQUE_DAMPING;   //阻尼   
        KDL::Vector delta_rot;
        KDL::Vector current_rot;
        KDL::Rotation template_rot;

        for ( int i{ 0 }; i < 3; i++ )
        {
            torque_acc_offset[ i ] = ( TCP_torque[ i ] - B * torque_vel_offset[ i ] ) / M;
            torque_vel_offset[ i ] = dt * ( torque_acc_offset[ i ] + torque_last_acc_offset[ i ] ) / 2 + torque_vel_offset[ i ];

            delta_rot( i )   = 10*dt * ( torque_vel_offset[ i ] + torque_last_vel_offset[ i ] ) / 2;
            current_rot( i ) = torque_pos_offset[ i ];

            torque_last_acc_offset[ i ] = torque_acc_offset[ i ];
            torque_last_vel_offset[ i ] = torque_vel_offset[ i ];
        }

        template_rot = KDL::Rotation::Rot( delta_rot, delta_rot.Norm( ) ) * KDL::Rotation::Rot( current_rot, current_rot.Norm( ) );

        current_rot = template_rot.GetRot( );

        for ( int i{ 0 }; i < 3; i++ )
            torque_pos_offset[ i ] = current_rot[ i ];



        Eigen::Matrix3d eigen_rotation;
        // 将 KDL::Rotation 转换为 Eigen::Matrix3d
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                eigen_rotation(i, j) = template_rot(i, j);  // 通过索引访问元素
            }
        }
        // return template_rot;
        Eigen::Quaterniond quaternion(eigen_rotation); 
        return quaternion;
      
    }

    Eigen::Quaterniond admittanceController_torque(const Eigen::Matrix3d &last_RR, const Eigen::Matrix3d &now_RR, const Eigen::Vector3d &torque) {

        Eigen::Vector3d rotation_diff_A = rotationDifference(now_RR,last_RR);
        const double M = TORQUE_MASS;       //虚拟质量
        const double B = TORQUE_DAMPING;   //阻尼        
        double T=0.005;             //控制周期
        Eigen::Vector3d  rotation_diff = rotationDifference(last_RR,now_RR);
        Eigen::Vector3d  dv= rotation_diff/T;
        Eigen::Vector3d  dxe=rotation_diff_A-(-dv)*T/2;  //减去自身期望运动分量
        Eigen::Vector3d  temp_result = (torque-B*dxe)/M;
        // 使用 AngleAxis 直接将旋转矢量转换为旋转矩阵
        Eigen::AngleAxisd angle_axis(temp_result.norm(), temp_result.normalized());
        Eigen::Matrix3d rotation_matrix = angle_axis.toRotationMatrix();
        //这里直接计算了目标姿态，now_RR去掉就是基于当前的增量
        auto result = now_RR*rotation_matrix;

        // std::cout<<temp_result.transpose()<<std::endl;

        Eigen::Quaterniond quaternion(result); 
        return quaternion;
    
    
    
    
    }
    Eigen::Vector3d rotationDifference(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2) 
    {
        Eigen::Matrix3d diff_R = R1 * R2.transpose();
        Eigen::AngleAxisd angle_axis(diff_R);
        return angle_axis.angle() * angle_axis.axis(); // 返回旋转矢量
    }

    ros::Subscriber force_sub_;
    ros::Subscriber joint_state_sub;
    ros::Subscriber joint_state_sub_left;
    ros::Subscriber joint_state_sub_right;

    ros::Publisher position_pub_;
    //
    double force_vel_offset[3]={ 0, 0, 0 };
    double force_last_vel_offset[3]={ 0, 0, 0 };
    double force_acc_offset[3]={ 0, 0, 0 };
    double force_last_acc_offset[3]={ 0, 0, 0 };

    double torque_vel_offset[3]={ 0, 0, 0 };
    double torque_pos_offset[3]={ 0, 0, 0 };
    double torque_last_vel_offset[3]={ 0, 0, 0 };
    double torque_acc_offset[3]={ 0, 0, 0 };
    double torque_last_acc_offset[3]={ 0, 0, 0 };
    //
    ArmParam arm_param_;
    geometry_msgs::Pose pose_msg;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    Eigen::Matrix3d sensorFrameMatrix;
    Eigen::Matrix3d now_R;
    Eigen::Matrix3d last_R;
    double now_x, now_y, now_z;
    double first_force_x, first_force_y, first_force_z;
    double first_torque_x, first_torque_y, first_torque_z;
    double last_x, last_y, last_z;
    bool first_force_received;
};
    int main(int argc, char** argv) { 
        ros::init(argc, argv, "arm_controller"); ros::NodeHandle nh;
        ArmController left_arm_controller(nh, "left");
        ArmController right_arm_controller(nh, "right");

    ros::spin();
    return 0;
    }
