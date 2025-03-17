/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

// #include <mujoco/mujoco.h>
#include "../include/mujoco/mujoco.h"
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>  
// #include <ros/ros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include "../robotData/robotData.h"
#include "../robotData/robotStructs.h"

class MJ_Interface {
public:
    int jointNum{0};
    std::vector<double> motor_pos;
    std::vector<double> motor_pos_Old;
    std::vector<double> motor_vel;
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    Eigen::VectorXd last_q;


    Eigen::VectorXd m_q;
    // double rpy[3]{0}; // roll,pitch and yaw of baselink
    // double baseQuat[4]{0}; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    // double f3d[3][2]{0}; // 3D foot-end contact force, L for 1st col, R for 2nd col
    // double basePos[3]{0}; // position of baselink, in world frame
    // double baseAcc[3]{0};  // acceleration of baselink, in body frame
    // double baseAngVel[3]{0}; // angular velocity of baselink, in body frame
    // double baseLinVel[3]{0}; // linear velocity of baselink, in body frame
    const std::vector<std::string> JointName={"hip_roll_l_joint", "hip_pitch_l_joint","hip_yaw_l_joint",
                                             "knee_pitch_l_joint", "ankle_pitch_l_joint", "ankle_roll_l_joint", 
                                             "hip_roll_r_joint", "hip_pitch_r_joint","hip_yaw_r_joint",
                                             "knee_pitch_r_joint", "ankle_pitch_r_joint", "ankle_roll_r_joint",
                                             "body_yaw_joint",
                                             "shoulder_pitch_l_joint","shoulder_roll_l_joint","shoulder_yaw_l_joint","elbow_pitch_l_joint",
                                             "shoulder_pitch_r_joint","shoulder_roll_r_joint","shoulder_yaw_r_joint","elbow_pitch_r_joint"}; // joint name in XML file, the corresponds motors name should be M_*, ref to line 29 of MJ_Interface.cpp
// const std::string baseName="base_link";
// Joint 2: body_yaw_joint
// Joint 3: shoulder_pitch_l_joint
// Joint 4: shoulder_roll_l_joint
// Joint 5: shoulder_yaw_l_joint
// Joint 6: elbow_pitch_l_joint
// Joint 7: shoulder_pitch_r_joint


// Joint 8: shoulder_roll_r_joint
// Joint 9: shoulder_yaw_r_joint
// Joint 10: elbow_pitch_r_joint
// Joint 11: hip_roll_l_joint
// Joint 12: hip_pitch_l_joint
// Joint 13: hip_yaw_l_joint
// Joint 14: knee_pitch_l_joint
// Joint 15: ankle_pitch_l_joint
// Joint 16: ankle_roll_l_joint
// Joint 17: hip_roll_r_joint
// Joint 18: hip_pitch_r_joint
// Joint 19: hip_yaw_r_joint
// Joint 20: knee_pitch_r_joint
// Joint 21: ankle_pitch_r_joint
// Joint 22: ankle_roll_r_joint
    
    const std::string baseName="Base_link";
    
    const std::string orientationSensorName="baseline_quat"; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    const std::string velSensorName="baseline_vel";
    const std::string gyroSensorName="baseline_gyro";
    const std::string accSensorName="baselink-baseAcc";
    const std::string bposSensorName="baseline_position";
    MJ_Interface(mjModel *mj_modelIn, mjData  *mj_dataIn);
    void updateSensorValues();
    void setMotorsTorque(Eigen::VectorXd& qr);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void robotStateUpData();
    int  sim = 0;
    bool first = true;
    Eigen::VectorXd nowq;
    Eigen::VectorXd nowqd;
        // void dataBusWrite(DataBus &busIn);

private:
    mjModel *mj_model;
    mjData  *mj_data;
    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl, jntId_qacc;

    RobotData *m_robot;
    RobotStructs m_state;
    
    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;
    int bposID;

    double timeStep{1}; // second
    bool isIni{false};
};



