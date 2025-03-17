/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <mujoco/mujoco.h>
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>  
// #include <ros/ros.h>
#include "PDcon.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

class MJ_Interface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    PDController pd;
    Eigen::VectorXd last_q;


    Eigen::VectorXd m_q;
    // double rpy[3]{0}; // roll,pitch and yaw of baselink
    // double baseQuat[4]{0}; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    // double f3d[3][2]{0}; // 3D foot-end contact force, L for 1st col, R for 2nd col
    // double basePos[3]{0}; // position of baselink, in world frame
    // double baseAcc[3]{0};  // acceleration of baselink, in body frame
    // double baseAngVel[3]{0}; // angular velocity of baselink, in body frame
    // double baseLinVel[3]{0}; // linear velocity of baselink, in body frame
    const std::vector<std::string> JointName={"waist_Z_joint","waist_roll_joint", "waist_yaw_joint", 
                                            "left_joint1", "left_joint2", "left_joint3", "left_joint4", 
                                            "left_joint5", "left_joint6", "left_joint7", "right_joint1", 
                                            "right_joint2", "right_joint3", "right_joint4", "right_joint5", 
                                            "right_joint6", "right_joint7"}; // joint name in XML file, the corresponds motors name should be M_*, ref to line 29 of MJ_Interface.cpp
    // const std::string baseName="base_link";
    const std::string baseName="pelvis";
    // const std::string orientationSensorName="baselink-quat"; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    // const std::string velSensorName="baselink-velocity";
    // const std::string gyroSensorName="baselink-gyro";
    // const std::string accSensorName="baselink-baseAcc";
    
    MJ_Interface(mjModel *mj_modelIn, mjData  *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(Eigen::VectorXd& qr);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    // void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    double timeStep{1}; // second
    bool isIni{false};
};



