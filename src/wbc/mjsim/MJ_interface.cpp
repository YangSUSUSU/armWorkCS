
#include "MJ_interface.h"
#include <iostream>
MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn) 
{

    m_robot = &RobotData::getInstance();
    m_state = m_robot->getRobotState();
    // joint_state_sub_ = nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);
    mj_model=mj_modelIn;
    mj_data=mj_dataIn;

    double total_mass = 0.0;
    for (int i = 0; i < mj_model->nbody; i++) 
    {
        total_mass += mj_model->body_mass[i];
    }
    std::cout << "Total mass: " << total_mass << " kg" << std::endl;
    // mj_model->opt.collision = 0;
    // timeStep=mj_model->opt.timestep;
    // mj_model->opt.timestep = 0.001;
    jointNum=JointName.size();
    std::cout<<"=============总关节数量："<<jointNum<<std::endl;
    jntId_qpos.assign(jointNum,0);
    jntId_qvel.assign(jointNum,0);
    jntId_dctl.assign(jointNum,0);
    jntId_qacc.assign(jointNum,0);
    // motor_pos.assign(jointNum,0);
    // motor_vel.assign(jointNum,0);
    // motor_pos_Old.assign(jointNum,0);
    last_q = Eigen::VectorXd::Zero(jointNum);
   nowq = Eigen::VectorXd::Zero(jointNum);
    nowqd = Eigen::VectorXd::Zero(jointNum);
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
        jntId_qacc[i]=mj_model->jnt_dofadr[tmpId];
        // std::cout <<"jntId_qpos[i]"<<jntId_qpos[i]<<std::endl;
        // std::cout <<"jntId_qvel[i]"<<jntId_qvel[i]<<std::endl;

        std::string motorName = JointName[i];
        motorName=motorName;
        tmpId= mj_name2id(mj_model,mjOBJ_ACTUATOR,motorName.c_str());
        if (tmpId==-1)
        {
            std::cerr <<motorName<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i]=tmpId;
    }
    baseBodyId= mj_name2id(mj_model,mjOBJ_BODY, baseName.c_str());
    orientataionSensorId= mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    velSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,velSensorName.c_str());
    gyroSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,gyroSensorName.c_str());
    bposID =  mj_name2id(mj_model,mjOBJ_SENSOR,bposSensorName.c_str());
    std::cout<<"==baseBodyId============"<<baseBodyId<<std::endl;
    std::cout<<"==orientataionSensorId=="<<orientataionSensorId<<std::endl;
    std::cout<<"==velSensorId==========="<<velSensorId<<std::endl;
    std::cout<<"==gyroSensorId=========="<<gyroSensorId<<std::endl;

    // accSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,accSensorName.c_str());

}
void MJ_Interface::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
}
void MJ_Interface::robotStateUpData()
{
   
    for (int i = 0; i < jointNum; i++)
    {
        m_state.q(i+7) = mj_data->qpos[jntId_qpos[i]];
        m_state.qd(i+6) = mj_data->qvel[jntId_qvel[i]];
        m_state.qdd(i+6) = mj_data->qacc[jntId_qacc[i]];

    }
    //  floating  base 的 姿态
    for (int i=0;i<3;i++)
    m_state.q(i)=mj_data->sensordata[mj_model->sensor_adr[bposID]+i];
    double temp[4] = {0};
    for (int i=0;i<4;i++)
    {
        temp[i]=mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+i];
    }
    std::cout<<"=====quat====" <<temp[0] <<";"<< temp[1]<<";"<< temp[2] <<";"<< temp[3]<<std::endl;

    //  pinocchio   xyzw    mujoco  wxyz
    m_state.q(0+3) = temp[1];
    m_state.q(1+3) = temp[2];
    m_state.q(2+3) = temp[3];
    m_state.q(3+3) = temp[0];//mujoco  w 

    

    double baseQuat[4] = {0};
    for (int i=0;i<4;i++)
    baseQuat[i]=mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+i];
 

    double tmp=baseQuat[0];
    baseQuat[0]=baseQuat[1];
    baseQuat[1]=baseQuat[2];
    baseQuat[2]=baseQuat[3];
    baseQuat[3]=tmp;

    m_state.rpy[0]= atan2(2*(baseQuat[3]*baseQuat[0]+baseQuat[1]*baseQuat[2]),1-2*(baseQuat[0]*baseQuat[0]+baseQuat[1]*baseQuat[1]));
    m_state.rpy[1]= asin(2*(baseQuat[3]*baseQuat[1]-baseQuat[0]*baseQuat[2]));
    m_state.rpy[2]= atan2(2*(baseQuat[3]*baseQuat[2]+baseQuat[0]*baseQuat[1]),1-2*(baseQuat[1]*baseQuat[1]+baseQuat[2]*baseQuat[2]));

    //  floating  base 的 速度

    for (int i=0;i<3;i++)
    {
        m_state.qd(i) = mj_data->sensordata[mj_model->sensor_adr[velSensorId]+i];
        m_state.qd(i+3) = mj_data->sensordata[mj_model->sensor_adr[gyroSensorId]+i];
        // baseLinVel[i]=(basePos[i]-posOld)/(mj_model->opt.timestep);
    }
    // std::cout<<"==quat=="<<  m_state.q.head(3).transpose()<<std::endl;

    m_robot->setRobotState(m_state);

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

void MJ_Interface::setMotorsTorque(Eigen::VectorXd& input) 
{
    sim++;
    // std::cout<<"=====input===="<<input.size()<<std::endl;

    if (sim>80)
    {
    // std::cout<<"====1====="<<sim<<std::endl;

    // 输入向量：前 20 个是前馈力矩，后 20 个是期望加速度
    Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(21); 
    Eigen::VectorXd qdd_des = Eigen::VectorXd::Zero(21);
    // std::cout<<"====2====="<<sim<<std::endl;

    tau_ff = input.head(21);  // 前馈力矩
    qdd_des = input.tail(21); // 期望加速度
    // std::cout<<"====3====="<<sim<<std::endl;

    // 获取当前关节位置和速度
    Eigen::VectorXd nowq1 = Eigen::VectorXd::Zero(jointNum);
    Eigen::VectorXd nowqd1 = Eigen::VectorXd::Zero(jointNum);
    // std::cout<<"====4====="<<sim<<std::endl;

    if(1)
    {
        for (int i = 0; i < jointNum; i++) 
        {
        nowq(i) = mj_data->qpos[jntId_qpos[i]];
        nowqd(i) = mj_data->qvel[jntId_qvel[i]];
        }
        // first = false;
    }
    for (int i = 0; i < jointNum; i++) 
    {
        nowq1(i) = mj_data->qpos[jntId_qpos[i]];
        nowqd1(i) = mj_data->qvel[jntId_qvel[i]];
    }
    // std::cout<<"====5====="<<sim<<std::endl;

    // 期望位置和速度（通过积分期望加速度生成）
    static Eigen::VectorXd q_des = nowq1;  // 期望位置
    static Eigen::VectorXd qd_des = nowqd1; // 期望速度
    // std::cout<<"====6====="<<sim<<std::endl;

    double dt = 0.01;  // 控制周期，根据实际情况调整

    // 积分期望加速度生成期望速度和位置
    qd_des += qdd_des * dt;
    q_des += 0.5* qdd_des * dt * dt;
    // PD 控制参数
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(21); 
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(21);
    Kp<<3000.0,  3000.0,  3000.2,  2500,  1800.0,  1800.0,
        3000.0,  3000.0,  3000.2,  2500,  1800.0,  1800.0, 
        3000,
        1200,1200,1200,1200,
        1200,1200,1200,1200;
    Kd<<18,    18,    18,   12.0,   4.5,  4.5,
        18,    18,    18,   12.0,   4.5,  4.5,
        18,
        10, 10, 10, 10,
        10, 10, 10, 10;

    // 计算 PD 控制力矩
    Eigen::VectorXd pos_error = q_des - nowq1;       // 位置误差
    Eigen::VectorXd vel_error = qd_des - nowqd1;     // 速度误差
    // Eigen::VectorXd tau_pd = Kp * pos_error + Kd * vel_error;  // PD 控制力矩
    // std::cout<<"====8====="<<sim<<std::endl;

    // 计算总下发力矩（前馈 + PD 控制）
    // Eigen::VectorXd tau_total = tau_ff+tau_pd ; 7 14 18  19 25 
    // std::cout<<"====9====="<<sim<<std::endl;

    // 将力矩下发至电机 tau_ff(i)+ tau_ff(i)+ 
    for (int i = 0; i < 21; i++) 
    {
        // mj_data->ctrl[i] = tau_ff(i)+ 1*Kp(i) * pos_error(i) + 0.6 * Kd(i)  * vel_error(i);

        mj_data->ctrl[i] = tau_ff(i) + 1.0*Kp(i) * pos_error(i) + 0.6 * Kd(i)  * vel_error(i);
        // std::cout<<"====9====="<<i<<"--"<<mj_data->ctrl[i]  <<std::endl;

    }
    // mj_data->ctrl[12]= 100 * (0- mj_data->qpos[jntId_qpos[12]]) -5 * mj_data->qvel[jntId_qvel[12]];
    // for (int i = 0; i < 8; i++) 
    // {
    //     mj_data->ctrl[i+13] = 80 * pos_error(i+13) + 5 * vel_error(13+i) ;
    // }
    Eigen::VectorXd qr = Eigen::VectorXd::Zero(jointNum);

    // for (int i=17;i<21;i++)
    // {
    //     // mj_data->ctrl[i] =  500 * (qr(i) - mj_data->qpos[jntId_qpos[i]]) - 3* mj_data->qvel[jntId_qvel[i]];
    //     // std::cout<<"=====jointNum===="<<i<<"--"<< 50 * (qr(i) - mj_data->qpos[jntId_qpos[i]]) - 0.1 * mj_data->qvel[jntId_qvel[i]]<<std::endl;
    //             // std::cout<<"====9====="<<i<<"--"<<mj_data->ctrl[i]<<std::endl;

    // }
    }
    else
    {
        Eigen::VectorXd qr = Eigen::VectorXd::Zero(jointNum);
        qr(1) =  -0.35;
        qr(3) =  0.7;
        qr(4) =  -0.35;
    
        qr(1+6) =  -0.35;
        qr(3+6) = 0.7;
        qr(4+6) =  -0.35;
    
        for (int i=0;i<jointNum;i++)
        {
            mj_data->ctrl[i]= 500 * (qr(i) - mj_data->qpos[jntId_qpos[i]]) -5 * mj_data->qvel[jntId_qvel[i]];
            // std::cout<<"=====jointNum===="<<i<<"--"<< 50 * (qr(i) - mj_data->qpos[jntId_qpos[i]]) - 0.1 * mj_data->qvel[jntId_qvel[i]]<<std::endl;

        }
    }
    
   
}










