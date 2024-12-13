#include "robotPara.hpp"
#include <pinocchio/urdf/model.hpp>
#include <iostream>

// 私有构造函数，加载机器人URDF模型
RobotPara::RobotPara()
{
    const std::string& urdf_path = "/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model/humanoid.urdf";
    if (!urdf_path.empty()) 
    {
        pinocchio::urdf::loadModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        joint_angles_ = Eigen::VectorXd::Zero(model_.nq);
    } 
    else 
    {
        std::cerr << "URDF path is empty, failed to load robot model." << std::endl;
    }

     q = Eigen::VectorXd::Zero(18);
     qd = Eigen::VectorXd::Zero(18);
     G = Eigen::VectorXd::Zero(18);
     M = Eigen::MatrixXd::Identity(18, 18);
     C = Eigen::MatrixXd::Identity(18, 18);
     jac_l = Eigen::MatrixXd::Identity(6, 18);
     jac_r = Eigen::MatrixXd::Identity(18, 18);
     tcp = Eigen::VectorXd::Zero(14); // xyz rxryrz W  LR
}

void RobotPara::setQ(const Eigen::VectorXd& q_)
{
    q = q_;
}
void RobotPara::setQd(const Eigen::VectorXd& qd_)
{
    qd = qd_;
}
Eigen::MatrixXd RobotPara::getM()
{
    pinocchio::crba(model_,data_,q);
    pinocchio::computeCoriolisMatrix(model_, data_, q, qd);
    Eigen::MatrixXd C_full = data_.C; 
    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q);
    Eigen::MatrixXd M_full = data_.M;
    return M_full;
}
Eigen::MatrixXd RobotPara::getC()
{
    pinocchio::crba(model_,data_,q);
    pinocchio::computeCoriolisMatrix(model_, data_, q, qd);
    Eigen::MatrixXd C_full = data_.C; 
    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q);
    Eigen::MatrixXd M_full = data_.M;
    return C_full;
}
Eigen::VectorXd RobotPara::getG()
{
    pinocchio::crba(model_,data_,q);
    pinocchio::computeCoriolisMatrix(model_, data_, q, qd);
    Eigen::MatrixXd C_full = data_.C; 
    Eigen::VectorXd G_full = pinocchio::computeGeneralizedGravity(model_, data_, q); 
    Eigen::MatrixXd M_full = data_.M; 
    return G_full;
}
Eigen::VectorXd RobotPara::getArmTcp()
{
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Vector3d pos_l = data_.oMf[22].translation();
    Eigen::Vector3d pos_r = data_.oMf[74].translation();
    Eigen::Quaterniond quaternionL(data_.oMf[22].rotation());
    Eigen::Quaterniond quaternionR(data_.oMf[74].rotation());
    tcp.head(3)=pos_l;
    tcp.segment(7,3)=pos_r;

    tcp(3)=quaternionL.x();
    tcp(4)=quaternionL.y();
    tcp(5)=quaternionL.z();
    tcp(6)=quaternionL.w();

    tcp(10)=quaternionR.x();
    tcp(11)=quaternionR.y();
    tcp(12)=quaternionR.z();
    tcp(13)=quaternionR.w();

}
Eigen::MatrixXd RobotPara::getJacL()
{    
    pinocchio::forwardKinematics(model_,data_,q);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::FrameIndex frame_id_left = model_.getFrameId("left_flange");
    Eigen::MatrixXd jacobian_l = pinocchio::getFrameJacobian(model_, data_, frame_id_left, pinocchio::LOCAL_WORLD_ALIGNED);

}
Eigen::MatrixXd RobotPara::getJacR()
{
    pinocchio::forwardKinematics(model_,data_,q);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::FrameIndex frame_id_right = model_.getFrameId("right_flange");
    Eigen::MatrixXd jacobian_r = pinocchio::getFrameJacobian(model_, data_, frame_id_right, pinocchio::LOCAL_WORLD_ALIGNED);
}