#include "robotData.h"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/joint/joint-generic.hpp> // 确保包含这个文件
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/algorithm/centroidal.hpp>
#include <iostream>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
// 初始化静态成员变量
RobotData* RobotData::instance = nullptr;

// 获取单例实例
RobotData& RobotData::getInstance() 
{
    if (instance == nullptr) {
        instance = new RobotData();
    }
    return *instance;
}
RobotData::RobotData() 
{
    std::cout << "==============RobotData 单例初始化完成============" << std::endl;
    std::string urdf_path = "/home/nikoo/workWS/armWorkCS/src/wbc/models/tiangong2_EVT_V2/urdf/tiangong2_EVT_V2.urdf";
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::urdf::buildModel(urdf_path,root_joint, modelFree);
    dataFree = pinocchio::Data(modelFree);
    std::cout << "==============Pinocchio 模型加载完成，关节数nq：" << modelFree.nq << std::endl;
    int njoints = modelFree.njoints;
    std::cout << "==============Pinocchio 模型加载完成，关节数njoints：" << modelFree.njoints << std::endl;
    int nvs = modelFree.nv;
    std::cout << "==============Pinocchio 模型加载完成，关节数nvs：" <<nvs << std::endl;
    std::vector<std::string> joint_names(njoints);
    std::vector<int> joint_ids(njoints);
    double total_mass = 0.0;
    for (size_t i = 0; i < modelFree.inertias.size(); ++i) {
        total_mass += modelFree.inertias[i].mass();
    }

    std::cout << "pinocchio Total mass: " << total_mass << " kg" << std::endl;
    for (int i = 0; i < njoints; ++i) 
    {
        pinocchio::JointIndex joint_idx = i;
        joint_names[i] = modelFree.names[joint_idx];
        joint_ids[i] = joint_idx;
        std::cout << "Joint " << joint_idx << ": " << joint_names[i] << std::endl;
    }
    
    // //===========================测试例子=============================
    // Eigen::VectorXd q;
    // q = Eigen::VectorXd::Zero(nvs+1);
    // auto num = modelFree.getJointId("elbow_pitch_r_joint");
    // pinocchio::forwardKinematics(modelFree,dataFree,q);
    // pinocchio::computeJointJacobians(modelFree,dataFree,q);
    // pinocchio::jacobianCenterOfMass(modelFree, dataFree, q, true);
    // pinocchio::updateGlobalPlacements(modelFree,dataFree);
    // // Eigen::Matrix<double,6,-1> J_test;
    // // J_test=Eigen::MatrixXd::Zero(6,modelFree.nq);
    // pinocchio::FrameIndex frame_id = modelFree.getFrameId("elbow_pitch_r_link");
    // auto J_test = pinocchio::getJointJacobian(modelFree, dataFree, num, pinocchio::LOCAL_WORLD_ALIGNED);
    // std::cout<<J_test<<std::endl;
    // std::cout << "Jacobian dimensions: " << J_test.rows() << "x" << J_test.cols() << std::endl;
    // //===========================测试end=============================


    // Eigen::VectorXd q1;
    // q1 = Eigen::VectorXd::Zero(nvs+1);
    // q1(0) = 0.5;
    // q1(1) = 0.5;

    // auto num1 = modelFree.getJointId("elbow_pitch_r_joint");
    // pinocchio::forwardKinematics(modelFree,dataFree,q1);
    // pinocchio::computeJointJacobians(modelFree,dataFree,q1);
    // pinocchio::jacobianCenterOfMass(modelFree, dataFree, q1, true);
    // pinocchio::updateGlobalPlacements(modelFree,dataFree);
    // auto J_test1 = pinocchio::getJointJacobian(modelFree, dataFree, num1, pinocchio::LOCAL_WORLD_ALIGNED);
    // std::cout<<J_test1<<std::endl;
    // std::cout << "Jacobian dimensions: " << J_test1.rows() << "x" << J_test1.cols() << std::endl;

}
// 设置机器人状态
void RobotData::setRobotState(RobotStructs& state) 
{
    // Eigen::VectorXd q;
    // q = Eigen::VectorXd::Zero(nvs+1);
    // auto num = modelFree.getJointId("elbow_pitch_r_joint");
    // pinocchio::forwardKinematics(modelFree,dataFree,q);
    // pinocchio::computeJointJacobians(modelFree,dataFree,q);
    // pinocchio::jacobianCenterOfMass(modelFree, dataFree, q, true);
    // pinocchio::updateGlobalPlacements(modelFree,dataFree);
    // // Eigen::Matrix<double,6,-1> J_test;
    // // J_test=Eigen::MatrixXd::Zero(6,modelFree.nq);
    // pinocchio::FrameIndex frame_id = modelFree.getFrameId("elbow_pitch_r_link");
    // auto J_test = pinocchio::getJointJacobian(modelFree, dataFree, num, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs = state;
}
Eigen::Matrix<double, 3, 3> RobotData::eul2Rot(double roll, double pitch, double yaw) {
    Eigen::Matrix<double,3,3> Rx,Ry,Rz;
    Rz<<cos(yaw),-sin(yaw),0,
            sin(yaw),cos(yaw),0,
            0,0,1;
    Ry<<cos(pitch),0,sin(pitch),
            0,1,0,
            -sin(pitch),0,cos(pitch);
    Rx<<1,0,0,
            0,cos(roll),-sin(roll),
            0,sin(roll),cos(roll);
    return Rz*Ry*Rx;
}
// 获取机器人状态
RobotStructs RobotData::getRobotState() 
{

    Eigen::VectorXd tempW = Eigen::VectorXd::Zero(3);
    tempW<<robotStructs.qd(3),robotStructs.qd(4),robotStructs.qd(5);
    auto Rcur= eul2Rot(robotStructs.rpy(0), robotStructs.rpy(1), robotStructs.rpy(2));
    tempW = Rcur * tempW;

    // robotStructs.qd(3) = tempW(0);
    // robotStructs.qd(4)= tempW(1);
    // robotStructs.qd(5)= tempW(2);
    // robotStructs.qd
    pinocchio::forwardKinematics(modelFree, dataFree, robotStructs.q);
    pinocchio::jacobianCenterOfMass(modelFree, dataFree, robotStructs.q);
    pinocchio::computeCentroidalMap(modelFree, dataFree, robotStructs.q);
    pinocchio::updateGlobalPlacements(modelFree,dataFree);
    pinocchio::updateFramePlacements(modelFree, dataFree);
    pinocchio::FrameIndex handLID = modelFree.getFrameId("hand_l_temp_link");
    pinocchio::FrameIndex handRID = modelFree.getFrameId("hand_r_temp_link");
    pinocchio::FrameIndex footL1ID = modelFree.getFrameId("foot_l_1");
    pinocchio::FrameIndex footL2ID = modelFree.getFrameId("foot_l_2");
    pinocchio::FrameIndex footL3ID = modelFree.getFrameId("foot_l_3");
    pinocchio::FrameIndex footL4ID = modelFree.getFrameId("foot_l_4");
    pinocchio::FrameIndex footR3ID = modelFree.getFrameId("foot_r_3");
    pinocchio::FrameIndex footR4ID = modelFree.getFrameId("foot_r_4");
    pinocchio::FrameIndex footR1ID = modelFree.getFrameId("foot_r_1");
    pinocchio::FrameIndex footR2ID = modelFree.getFrameId("foot_r_2");
    pinocchio::FrameIndex baseID = modelFree.getJointId("root_joint");
    robotStructs.baseRot = dataFree.oMi[baseID].rotation();
    robotStructs.Jbase  = pinocchio::getFrameJacobian(modelFree, dataFree, baseID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JLHand  = pinocchio::getFrameJacobian(modelFree, dataFree, handLID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JRHand  = pinocchio::getFrameJacobian(modelFree, dataFree, handRID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JLFoot1 = pinocchio::getFrameJacobian(modelFree, dataFree, footL1ID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JLFoot2 = pinocchio::getFrameJacobian(modelFree, dataFree, footL2ID, pinocchio::LOCAL_WORLD_ALIGNED);

    robotStructs.JLFoot3 = pinocchio::getFrameJacobian(modelFree, dataFree, footL3ID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JLFoot4 = pinocchio::getFrameJacobian(modelFree, dataFree, footL4ID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JRFoot3 = pinocchio::getFrameJacobian(modelFree, dataFree, footR3ID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JRFoot4 = pinocchio::getFrameJacobian(modelFree, dataFree, footR4ID, pinocchio::LOCAL_WORLD_ALIGNED);

    robotStructs.JRFoot1 = pinocchio::getFrameJacobian(modelFree, dataFree, footR1ID, pinocchio::LOCAL_WORLD_ALIGNED);
    robotStructs.JRFoot2 = pinocchio::getFrameJacobian(modelFree, dataFree, footR2ID, pinocchio::LOCAL_WORLD_ALIGNED);
    // robotStructs.dJLFoot1 = robotStructs.dJLFoot1 * 
    auto tempqd = robotStructs.qd;
    // tempqd.segment(0,3) = Rcur.transpose() * tempqd.segment(0,3);
    // tempqd.segment(3,3) = Rcur.transpose() * tempqd.segment(3,3);
    // std::cout<<"=qd="<<robotStructs.q.head(6).transpose()<<std::endl;

    pinocchio::computeJointJacobiansTimeVariation(modelFree,dataFree,robotStructs.q,tempqd);
    pinocchio::dccrba(modelFree,dataFree,robotStructs.q,tempqd);
    pinocchio::computeCentroidalMomentum(modelFree,dataFree,robotStructs.q,tempqd);
    pinocchio::ccrba(modelFree, dataFree,robotStructs.q,tempqd);
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footL1ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJLFoot1);
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footL2ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJLFoot2 );
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footL3ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJLFoot3);
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footL4ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJLFoot4 );
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footR1ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJRFoot1 );
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footR2ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJRFoot2);
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footR3ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJRFoot3 );
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,footR4ID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJRFoot4);

    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,baseID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJbase);

    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,handLID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJLHand );
    pinocchio::getFrameJacobianTimeVariation(modelFree,dataFree,handRID,pinocchio::LOCAL_WORLD_ALIGNED,robotStructs.dJRHand);


  
    Eigen::MatrixXd Mpj; // transform into world frame, and accept dq that in world frame
    Mpj=Eigen::MatrixXd::Identity(27,27);
    Mpj.block(0,0,3,3)=Rcur;
    Mpj.block(3,3,3,3)=Rcur;

    //浮动基自由度雅可比转到世界坐标？还没搞清楚
    robotStructs.JLHand  = robotStructs.JLHand ;
    robotStructs.JRHand  =  robotStructs.JRHand ;
    robotStructs.JLFoot1 =  robotStructs.JLFoot1;
    robotStructs.JLFoot2 = robotStructs.JLFoot2;
    robotStructs.JRFoot1 = robotStructs.JRFoot1;
    robotStructs.JRFoot2 = robotStructs.JRFoot2;
    robotStructs.Jbase   =   robotStructs.Jbase;
    
    robotStructs.dJbase = robotStructs.dJbase ;
    robotStructs.dJLFoot1 = robotStructs.dJLFoot1;
    robotStructs.dJLFoot2 = robotStructs.dJLFoot2;
    robotStructs.dJRFoot1 = robotStructs.dJRFoot1;
    robotStructs.dJRFoot2 =  robotStructs.dJRFoot2;

    pinocchio::FrameIndex b1 = modelFree.getFrameId("foot_l_1");
    pinocchio::FrameIndex b2 = modelFree.getFrameId("foot_l_2");
    pinocchio::FrameIndex b3 = modelFree.getFrameId("foot_r_1");
    pinocchio::FrameIndex b4 = modelFree.getFrameId("foot_r_2");
    // pinocchio::FrameIndex handLID = modelFree.getFrameId("hand_l_temp_link");
    // pinocchio::FrameIndex handRID = modelFree.getFrameId("hand_r_temp_link");
    robotStructs.LHandPos  = dataFree.oMf[handLID].translation();
    robotStructs.RHandPos  = dataFree.oMf[handRID].translation();
    // std::cout<<"==handLpose="<< robotStructs.LHandPos.transpose()<<std::endl;
    // std::cout<<"==handRpose="<< robotStructs.RHandPos.transpose()<<std::endl;

    
    Eigen::Vector3d pos_b1 = dataFree.oMf[b1].translation();
    Eigen::Vector3d pos_b2 = dataFree.oMf[b2].translation();
    Eigen::Vector3d pos_b3 = dataFree.oMf[b3].translation();
    Eigen::Vector3d pos_b4 = dataFree.oMf[b4].translation();



    // std::cout<<"= ="<<pos_b1.transpose()<<pos_b2.transpose()<<pos_b3.transpose()<<pos_b4.transpose()<<std::endl;
    robotStructs.Jcom = dataFree.Jcom ;

    // robotStructs.Xcom = robotStructs.baseRot * (dataFree.com[0]);
    // robotStructs.Xcom =(dataFree.com[0]);
    robotStructs.Xcom = dataFree.com[0];
    // std::cout<<"= com="<<robotStructs.Xcom.transpose()<<std::endl;

    // std::cout<<"= ="<<robotStructs.Xcom.transpose()<<std::endl;

    // robotStructs.JLFoot1.block<3,26>(0,0) = robotStructs.JLFoot1.block<3,26>(0,0) - robotStructs.Jcom;
    // robotStructs.JLFoot2.block<3,26>(0,0) = robotStructs.JLFoot2.block<3,26>(0,0) - robotStructs.Jcom;
    // robotStructs.JRFoot1.block<3,26>(0,0) = robotStructs.JRFoot1.block<3,26>(0,0) - robotStructs.Jcom;
    // robotStructs.JRFoot2.block<3,26>(0,0) = robotStructs.JRFoot2.block<3,26>(0,0) - robotStructs.Jcom;

    // robotStructs.Com2RFootPoint1 = Rcur*(pos_b3) + robotStructs.q.head(3)- robotStructs.Xcom;
    // robotStructs.Com2RFootPoint2 = Rcur*(pos_b4) +  robotStructs.q.head(3)- robotStructs.Xcom;
    // robotStructs.Com2LFootPoint1 = Rcur*(pos_b1) +  robotStructs.q.head(3)- robotStructs.Xcom;
    // robotStructs.Com2LFootPoint2 = Rcur*(pos_b2) +  robotStructs.q.head(3)- robotStructs.Xcom;



    // Eigen::Matrix4d  bw;
    // bw.block<3,3>(0,0) = robotStructs.baseRot;
    // bw(0,3) = robotStructs.q(0);
    // bw(1,3) = robotStructs.q(1);
    // bw(2,3) = robotStructs.q(2);
    // bw(3,3) = 1;
    // pos_b1  = (bw * pos_b1.homogeneous()).hnormalized();
    // pos_b2  = (bw * pos_b2.homogeneous()).hnormalized();
    // pos_b3  = (bw * pos_b3.homogeneous()).hnormalized();
    // pos_b4  = (bw * pos_b4.homogeneous()).hnormalized();

    // std::cout<<"pos_b3==="<<pos_b3.transpose()<<std::endl;
    robotStructs.Com2RFootPoint1 = 1*((pos_b3)  - robotStructs.Xcom);
    robotStructs.Com2RFootPoint2 = 1*((pos_b4)  - robotStructs.Xcom);
    robotStructs.Com2LFootPoint1 = 1*((pos_b1)  - robotStructs.Xcom);
    robotStructs.Com2LFootPoint2 = 1*((pos_b2)  - robotStructs.Xcom);
    // std::cout<< "===robotStructs.Jcomcom==="<<robotStructs.Jcom.rows() << "x" << robotStructs.Jcom.cols() <<std::endl;
    // std::cout<< "===pos_b1==="<<pos_b1.transpose()<<std::endl;
    // std::cout<< "===pos_b2==="<<pos_b2.transpose()<<std::endl;
    // std::cout<< "===pos_b3==="<<pos_b3.transpose()<<std::endl;
    // std::cout<< "===pos_b4==="<<pos_b4.transpose()<<std::endl;
    // 计算质心雅可比矩阵
    

    robotStructs.inertia = dataFree.Ig.inertia().matrix();
    robotStructs.Ag = dataFree.Ag ;;
    robotStructs.dAg = dataFree.dAg;;

    pinocchio::crba(modelFree, dataFree, robotStructs.q);
    dataFree.M.triangularView<Eigen::Lower>() = dataFree.M.transpose().triangularView<Eigen::Lower>();
    auto dyn_M = dataFree.M;

    pinocchio::computeMinverse(modelFree, dataFree, robotStructs.q);
    dataFree.Minv.triangularView<Eigen::Lower>() = dataFree.Minv.transpose().triangularView<Eigen::Lower>();
    auto dyn_M_inv = dataFree.Minv;

    pinocchio::computeCoriolisMatrix(modelFree, dataFree, robotStructs.q, robotStructs.qd);
    auto dyn_C = dataFree.C;

    pinocchio::computeGeneralizedGravity(modelFree, dataFree, robotStructs.q);
    auto dyn_G = dataFree.g;
    // robotStructs.DynM = dyn_M;
    // robotStructs.DynMinv = dyn_M_inv;
    // robotStructs.DynC = dyn_C;
    // robotStructs.DynG = dyn_G;

    Eigen::MatrixXd Mpj_inv; // transform into world frame
    Mpj_inv=Eigen::MatrixXd::Identity(27,27);
    Mpj_inv.block(0,0,3,3)=robotStructs.baseRot;
    Mpj_inv.block(3,3,3,3)=robotStructs.baseRot;
    // dyn_M=Mpj_inv*dyn_M*Mpj;
    // dyn_M_inv=Mpj_inv*dyn_M_inv*Mpj;
    // dyn_C=Mpj_inv*dyn_C*Mpj;
    // dyn_G=Mpj_inv*dyn_G;
    robotStructs.DynM = dyn_M;
    robotStructs.DynMinv = dyn_M_inv;
    robotStructs.DynC = dyn_C;
    robotStructs.DynG = dyn_G;
    // Eigen::VectorXd tempQQ = Eigen::VectorXd::Zero(28);
    // tempQQ.tail(21) = robotStructs.q.tail(21);
    // tempQQ(6) = 1;
    // pinocchio::forwardKinematics(modelFree, dataFree, tempQQ);
    // pinocchio::updateGlobalPlacements(modelFree,dataFree);
    // pinocchio::updateFramePlacements(modelFree, dataFree);
    // robotStructs.RotFoot1 = dataFree.oMf[b1].rotation();//L
    // robotStructs.RotFoot2 = dataFree.oMf[b3].rotation();//R
    return robotStructs;
}
