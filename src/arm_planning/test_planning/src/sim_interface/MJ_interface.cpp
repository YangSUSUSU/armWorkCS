/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "MJ_interface.h"
#include <iostream>
MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn) 
{
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);
    mj_model=mj_modelIn;
    mj_data=mj_dataIn;
    // mj_model->opt.collision = 0;
    timeStep=mj_model->opt.timestep;
    jointNum=JointName.size();
    jntId_qpos.assign(jointNum,0);
    jntId_qvel.assign(jointNum,0);
    jntId_dctl.assign(jointNum,0);
    motor_pos.assign(jointNum,0);
    motor_vel.assign(jointNum,0);
    motor_pos_Old.assign(jointNum,0);

    m_q = Eigen::VectorXd::Zero(17);
    kp = Eigen::VectorXd::Zero(17);
    kp.setConstant(200);
    kd = Eigen::VectorXd::Zero(17);
    kd.setConstant(50);
    pd = PDController();
    last_q = Eigen::VectorXd::Zero(jointNum);
    for (int i=0;i<jointNum;i++)
    {
        int tmpId= mj_name2id(mj_model,mjOBJ_JOINT,JointName[i].c_str());
        if (tmpId==-1)
        {
            std::cerr <<JointName[i]<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_qpos[i]=mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i]=mj_model->jnt_dofadr[tmpId];
        std::string motorName=JointName[i];
        motorName=motorName;
        tmpId= mj_name2id(mj_model,mjOBJ_ACTUATOR,motorName.c_str());
        if (tmpId==-1)
        {
            std::cerr <<motorName<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i]=tmpId;
    }
//    int adr = m->sensor_adr[sensorId];
//    int dim = m->sensor_dim[sensorId];
//    mjtNum sensor_data[dim];
//    mju_copy(sensor_data, &d->sensordata[adr], dim);
    baseBodyId= mj_name2id(mj_model,mjOBJ_BODY, baseName.c_str());
    // orientataionSensorId= mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    // velSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,velSensorName.c_str());
    // gyroSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,gyroSensorName.c_str());
    // accSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,accSensorName.c_str());

}
void MJ_Interface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
   std::cout<<"=====callback==="<<std::endl;
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

    for (size_t i = 0; i < joint_names.size(); ++i) 
    {
        auto pos_it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (pos_it != msg->name.end()) 
        {
            size_t index = std::distance(msg->name.begin(), pos_it);
            m_q(i) = msg->position[index];
        }
    }
}
void MJ_Interface::updateSensorValues() {
    // for (int i=0;i<jointNum;i++)
    // {
    //     motor_pos_Old[i]=motor_pos[i];
    //     motor_pos[i]=mj_data->qpos[jntId_qpos[i]];
    //     motor_vel[i]=mj_data->qvel[jntId_qvel[i]];
    // }
    // for (int i=0;i<4;i++)
    //     baseQuat[i]=mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+i];
    // double tmp=baseQuat[0];
    // baseQuat[0]=baseQuat[1];
    // baseQuat[1]=baseQuat[2];
    // baseQuat[2]=baseQuat[3];
    // baseQuat[3]=tmp;

    // rpy[0]= atan2(2*(baseQuat[3]*baseQuat[0]+baseQuat[1]*baseQuat[2]),1-2*(baseQuat[0]*baseQuat[0]+baseQuat[1]*baseQuat[1]));
    // rpy[1]= asin(2*(baseQuat[3]*baseQuat[1]-baseQuat[0]*baseQuat[2]));
    // rpy[2]= atan2(2*(baseQuat[3]*baseQuat[2]+baseQuat[0]*baseQuat[1]),1-2*(baseQuat[1]*baseQuat[1]+baseQuat[2]*baseQuat[2]));

    // for (int i=0;i<3;i++)
    // {
    //     double posOld=basePos[i];
    //     basePos[i]=mj_data->xpos[3*baseBodyId+i];
    //     baseAcc[i]=mj_data->sensordata[mj_model->sensor_adr[accSensorId]+i];
    //     baseAngVel[i]=mj_data->sensordata[mj_model->sensor_adr[gyroSensorId]+i];
    //     baseLinVel[i]=(basePos[i]-posOld)/(mj_model->opt.timestep);
    // }

}

void MJ_Interface::setMotorsTorque(Eigen::VectorXd& qrrrr) {
    
    Eigen::VectorXd nowq = Eigen::VectorXd::Zero(jointNum);
    double t = static_cast<double>(mj_data->time);
    // double time =ros::Time::now().toSec(); // 当前时间;

    for (int i = 0; i < jointNum; i++)
    {
        nowq(i) = mj_data->qpos[i];
    }
    
    for (int i=0;i<jointNum;i++)
    {
        mj_data->ctrl[i]=0;
    }
    Eigen::VectorXd qr = Eigen::VectorXd::Zero(jointNum);
    qr = m_q;
    std::cout<<"qr=="<<m_q.transpose()<<std::endl;
    // qr(5) =  -1.64 + 0.5*sin(5*t);
    // qr(6) =  -1.23;
    // qr(12) =  1.64;
    // qr(13) =  1.23;    

    auto tau = pd.computeControl(qr, nowq, t);
    for (int i=0;i<jointNum;i++)
        {
            mj_data->ctrl[i]= nowq(i)  + tau(i);
        }
    // mj_data->ctrl[0]=-10* mj_data->qpos[0];
    // mj_data->ctrl[5]= 1*(-1.64 - mj_data->qpos[5]);
    // mj_data->ctrl[6]= 1*(-1.23 - mj_data->qpos[6]);
    // mj_data->ctrl[12]= 1*(1.64 - mj_data->qpos[12]);
    // mj_data->ctrl[13]= 1*(1.23 - mj_data->qpos[13]);

        //     desired_position(5) = -1.64;
        // desired_position(6) = -1.23;
        // desired_position(12) = 1.64;
        // desired_position(13) = 1.23;
}










