/**
 * @file kdl_kinematics_usage.cpp
 * @brief Slove IK by QP
 * @author He Yan
 * @date 2024-04-18
 * @version alpha
 */

#include <aubo_arm_planning/arm_qpik_solver.h>
#include <algorithm>
using namespace Eigen;

namespace arm_kinematics {

ArmQpIkSolver::ArmQpIkSolver()
{
  cond_thres_ = 800;
  joint_limit_thres_ = 0.01;
  max_abs_cart_linear_vel_ = 0.8;   // max 0.8m/s
  max_abs_omega_ = 1.0;   // max 1.0rad/s 
}

ArmQpIkSolver::ArmQpIkSolver(double cond_thres, double joint_limit_thres){
  cond_thres_ = cond_thres;
  joint_limit_thres_ = joint_limit_thres;
}

ArmQpIkSolver::~ArmQpIkSolver()
{

}

Eigen::VectorXd ArmQpIkSolver::getRPY(Eigen::Matrix3d T_matrix)
{
    Eigen::Vector3d rpy;
    double epsilon=1E-12;

    rpy(1) = atan2(-T_matrix(2,0), sqrt( pow(T_matrix(0,0),2) +pow(T_matrix(1,0),2)));
    if (abs(rpy(1)) > (M_PI_2-epsilon))
    {
       rpy(0)  = 0.0 ;
       rpy(2) = atan2(-T_matrix(0,1), T_matrix(1,1));
    }
    else
    {
       rpy(0)  = atan2(T_matrix(2,1), T_matrix(2,2));
       rpy(2)  = atan2(T_matrix(1,0), T_matrix(0,0));
    }
    return rpy;
}

bool ArmQpIkSolver::getIK_QP( ArmKinematicsSolver *kin_solver,
                              Eigen::VectorXd &q,
                              const double &frequency,
                              const Eigen::MatrixXd &T_end_des,
                              const Eigen::VectorXd &q_now,
                              std::string &error_string)
{
  q = q_now;   // return with the current-joint-states in the worst-case !

  Eigen::MatrixXd T_end_now;
  std::vector<double> q_current(q_now.data(), q_now.data() + q_now.size());
  if(!kin_solver->getFkSolution(T_end_now, q_current, error_string))
  {
    return false;
  }
  
  Eigen::Vector3d delta_pos = T_end_des.block<3,1>(0,3) - T_end_now.block<3,1>(0,3);
  Eigen::Vector3d delta_orien;

  Eigen::Isometry3d iso3d_end_des, iso3d_end_now;
  iso3d_end_des.matrix()=T_end_des;
  iso3d_end_now.matrix()=T_end_now;

  KDL::Frame frame_end_des, frame_end_now;
  tf::transformEigenToKDL(iso3d_end_des, frame_end_des);
  tf::transformEigenToKDL(iso3d_end_now, frame_end_now);

  KDL::Vector delta_rot=frame_end_now.M*((frame_end_now.M.Inverse()*frame_end_des.M).GetRot());

  tf::vectorKDLToEigen(delta_rot, delta_orien);

  // for(int i=0; i<3; i++){
  //   if(std::fabs(delta_orien(i)) > max_abs_omega_){
  //     std::cout << "robot end omega out of bound:    delta_orien(i)*frequency = " << delta_orien(i) << std::endl;
  //     return false;
  //   }
  //   if(std::fabs(delta_pos(i)) > max_abs_cart_linear_vel_){
  //     std::cout << "robot end linear-vel out of bound:    delta_pos(i)*frequency) = " << delta_pos(i)*frequency << std::endl;
  //     return false;
  //   }
  // }
  
  Eigen::VectorXd delta_x(6);
  delta_x << delta_pos, delta_orien;

  qpsolver_.data()->clearHessianMatrix();
  qpsolver_.data()->clearLinearConstraintsMatrix();
  qpsolver_.clearSolver();

  Eigen::VectorXd qp_vel(kin_solver->getDof()), qp_acc(kin_solver->getDof());
  int error = solveIK_withOSQP(kin_solver, q_now, delta_x, frequency, q, qp_vel, qp_acc);

  if(error){
    std::cout << "QP_IK program exits with error!" << std::endl;
    return false;
  }else{
    return true;
  }
}

int ArmQpIkSolver::solveIK_withOSQP(ArmKinematicsSolver *kin_solver,
                                    const Eigen::VectorXd &q_current,
                                    const Eigen::VectorXd &delta_x,
                                    const double &frequency,
                                    Eigen::VectorXd &q_result,
                                    Eigen::VectorXd &qp_vel,
                                    Eigen::VectorXd &qp_acc)
{
  int Nj = int(kin_solver->getDof());

  KDL::JntArray q_min, q_max;
  kin_solver->getJointLimits(q_min, q_max);
  
  Eigen::VectorXd upper_gap(Nj), lower_gap(Nj);
  upper_gap = q_max.data - q_current;
  lower_gap = q_current - q_min.data;
  double closest_dist_to_ub = upper_gap.maxCoeff();
  double closest_dist_to_lb = lower_gap.minCoeff();
  if(closest_dist_to_lb < joint_limit_thres_){
    std::cout << "joint exceeds lower limit ! \n";
    std::cout << "q_current = \n" << q_current << "\nlower bound = \n" << q_min.data << std::endl;
    return 1; 
  }
  if(closest_dist_to_ub < joint_limit_thres_){
    std::cout << "joint exceeds upper limit ! \n";
    std::cout << "q_current = \n" << q_current << "\nupper bound = \n" << q_max.data << std::endl;
    return 1;
  }

  Eigen::VectorXd q_gap(Nj);
  q_gap = q_current - (q_max.data + q_min.data) / 2;

  Eigen::VectorXd diagonalElement(Nj+6);
  diagonalElement << Eigen::MatrixXd::Ones(Nj,1)*8, Eigen::MatrixXd::Ones(6,1)*10000;

  Eigen::SparseMatrix<double> hessian; //P or H
  hessian.resize(Nj+6, Nj+6);
  for (int i=0; i<Nj+6; ++i) { hessian.insert(i,i) = diagonalElement(i); }

  Eigen::VectorXd gradient = Eigen::MatrixXd::Zero(Nj+6,1);  //f or q
  double R_i = 0.005;
  for(int i=0; i<Nj; i++){
    gradient(i) = 2 * R_i * q_gap(i);
  }

  Eigen::VectorXd joint_low(Nj), joint_up(Nj);
  for (int i=0; i<Nj; ++i)
  {
      joint_low(i) = std::max(-0.3,  (q_min.data(i) - q_current(i)));   // min joint :  -0.3 rad 
      joint_up(i) = std::min(0.3,  (q_max.data(i) - q_current(i)));     // max joint :   0.3 rad 
  }

  Eigen::SparseMatrix<double> linearMatrix;  //A Aeq
  linearMatrix.resize(Nj+6, Nj+6);
  Eigen::MatrixXd eye_matrix = Eigen::MatrixXd::Identity(Nj, Nj);

  Eigen::MatrixXd jacob;
  std::vector<double> q_vec(q_current.data(), q_current.data() + q_current.size());
  std::string error_string;
  if(!kin_solver->getJacobian(jacob, q_vec, error_string))
  {    return 1;  }


  for (int i=0; i<6; ++i)
  {
      for (int j=0; j<Nj; ++j)
          linearMatrix.insert(i,j) = jacob(i,j);

      for (int k=Nj; k<Nj+6; ++k)
          linearMatrix.insert(i,k) = -eye_matrix(i,k-Nj);
  }
  for (int i=Nj-1; i<Nj+6; ++i)
  {
      for (int k=0; k<Nj; ++k)
          linearMatrix.insert(i,k) = eye_matrix(i-Nj+1,k);
  }

  // std::cout << "linearMatrix = \n" << linearMatrix << std::endl;

  Eigen::VectorXd lowerBound(Nj+6); //l
  Eigen::VectorXd upperBound(Nj+6); //u

  lowerBound << delta_x, joint_low;
  upperBound << delta_x, joint_up;

  int NumberOfVariables = Nj+6; //A矩阵的列数
  int NumberOfConstraints = Nj+6; //A矩阵的行数

  // set the initial data of the QP solver
  qpsolver_.settings()->setVerbosity(false);
  qpsolver_.settings()->setWarmStart(true);
  qpsolver_.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
  qpsolver_.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
  if(!qpsolver_.data()->setHessianMatrix(hessian)){
    return 1;//设置P矩阵
  }
  if(!qpsolver_.data()->setGradient(gradient)){
    return 1; //设置q or f矩阵。当没有时设置为全0向量
  }
  if(!qpsolver_.data()->setLinearConstraintsMatrix(linearMatrix)){
    return 1;//设置线性约束的A矩阵
  }
  if(!qpsolver_.data()->setLowerBound(lowerBound)){
    return 1;//设置下边界
  }
  if(!qpsolver_.data()->setUpperBound(upperBound)){
    return 1;//设置上边界
  }

  // instantiate the solver
  if(!qpsolver_.initSolver()){
    return 1;
  }

  // solve the QP problem
  if(!qpsolver_.solve()){
    return 1;
  }

  Eigen::VectorXd QPSolution(Nj+6);

  // get the controller input
  QPSolution = qpsolver_.getSolution();

  // std::cout << "qp solution = \n" << QPSolution.block<7,1>(0,0) << std::endl;
  // std::cout << "relaxation = \n" << QPSolution.block<6,1>(7,0) << std::endl;

  for (int i=0; i<Nj; i++)
  {
      // q_result(i) = q_current(i) + QPSolution(i)*(1.0/ frequency);
      q_result(i) = q_current(i) + QPSolution(i);
      qp_vel(i)=QPSolution(i)*frequency;
      qp_acc(i)=qp_vel(i)*frequency;
  }

  Eigen::MatrixXd jacob_res;
  std::vector<double> q_vec_res(q_result.data(), q_result.data() + q_result.size());
  if(!kin_solver->getJacobian(jacob_res, q_vec_res, error_string))
  {    return 1;  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_res(jacob_res);

  double cond_res = svd_res.singularValues()(0) / svd_res.singularValues()(svd_res.singularValues().size()-1);
  // std::cout << "conditional number of next step -------->> " << cond_res << std::endl;
  if(cond_res > cond_thres_){
    std::cout << "Quadratic Programming exits with error: cond of Jacobian matrix too large!!" << std::endl;
    return 1;
  }

  qpsolver_.data()->clearHessianMatrix();
  qpsolver_.data()->clearLinearConstraintsMatrix();
  qpsolver_.clearSolver();

  return 0;
}


//inverse kinematics
//q_current---第一帧读取当前位置，从第二帧开始用上一帧QP输出的计算关节角
Eigen::VectorXd ArmQpIkSolver::IKqpSolution(ArmKinematicsSolver *kin_solver,const double &frequency,const Eigen::Matrix4d &T, const Eigen::VectorXd &q_current,std::string &error_string,std::vector<double> joints_min,std::vector<double> joints_max)
{

    //用上一次QP计算得到的关节计算末端位置（构建了计算闭环），从而微分计算末端，这样考虑了QP的计算误差，提高了轨迹的跟踪精度
    MatrixXd T_fb;
    std::vector<double> q_cur(q_current.data(),q_current.data()+q_current.size());
    kin_solver->getFkSolution(T_fb, q_cur, error_string);
    
    Vector3d distance_error_fb;
    Matrix3d pose_matrix_fb;
    Vector3d pose_error_fb;
    for(int i = 0; i < 3 ;i++)
        distance_error_fb[i] = (T(i,3) - T_fb(i,3));

    pose_matrix_fb = (T_fb.block<3,3>(0,0)).inverse()*(T.block<3,3>(0,0));
    pose_error_fb = getRPY(pose_matrix_fb);
    pose_error_fb = T_fb.block<3,3>(0,0) * pose_error_fb;

    Vector3d distance_vel;
    Vector3d pose_vel;
    distance_vel = distance_error_fb*frequency;
    pose_vel = pose_error_fb*frequency;

    VectorXd cartesian_vel(6);
    cartesian_vel << distance_vel, pose_vel;
    VectorXd q_result(7);
    qpsolver_.data()->clearHessianMatrix();
    qpsolver_.data()->clearLinearConstraintsMatrix();
    qpsolver_.clearSolver();
    int ret = qpSolution(kin_solver,q_current, frequency,cartesian_vel, q_result,joints_min,joints_max);
    if (ret ==1)
    {
        std::cout<<"QP_SETTING_FAILED"<<std::endl;
        return q_current;
    }
    return q_result;
}

//q_current---第一帧读取当前位置，从第二帧开始用上一帧QP输出的计算关节角
int ArmQpIkSolver::qpSolution(ArmKinematicsSolver *kin_solver,const Eigen::VectorXd &q_current, const double &frequency, const Eigen::VectorXd &cartesian_vel, Eigen::VectorXd &q_result,std::vector<double> joints_min,std::vector<double> joints_max)
{
    VectorXd q_vel_low(7), q_vel_up(7);
    for (int i=0; i<7; ++i)
    {
        q_vel_low(i) = std::max(-2.0944, frequency*(joints_min[i]-q_current(i)));
        q_vel_up(i) = std::min(2.0944, frequency*(joints_max[i]-q_current(i)));
        // std::cout<<"q_vel_low"<<q_vel_low<<std::endl;
    }

    //--------------------------------- allocate QP problem matrices and vectores--------------------------//
    Eigen::SparseMatrix<double> linearMatrix;  //A Aeq
    linearMatrix.resize(13,13);
    Eigen::MatrixXd eye_matrix = Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd jacob;
    std::vector<double> q_vec(q_current.data(), q_current.data() + q_current.size());
    std::string error_string;
    if(!kin_solver->getJacobian(jacob, q_vec, error_string))
    {    return 1;  }

    // linearMatrix ?
    for (int i=0; i<6; ++i)
    {
        for (int j=0; j<7; ++j)
            linearMatrix.insert(i,j) = jacob(i,j);

        for (int k=7; k<13; ++k)
            linearMatrix.insert(i,k) = eye_matrix(i,k-7);
    }

    for (int i=6; i<13; ++i)
    {
        for (int k=0; k<7; ++k)
            linearMatrix.insert(i,k) = eye_matrix(i-6,k);
    }

    VectorXd lowerBound(13); //l
    VectorXd upperBound(13); //u

    //equal constraiant and unequal constraiant
    lowerBound << cartesian_vel, q_vel_low;
    upperBound << cartesian_vel, q_vel_up;

    int NumberOfVariables = 13; //A矩阵的列数
    int NumberOfConstraints = 13; //A矩阵的行数

    Eigen::VectorXd diagonalElement(13);
    diagonalElement << MatrixXd::Ones(7,1)*10, MatrixXd::Ones(6,1)*30;
    Eigen::SparseMatrix<double> hessian;
    hessian.resize(13,13);
    diagonalElement(4)=30;
    for (int i=0; i<13; ++i)
        hessian.insert(i,i) = diagonalElement(i);

    Eigen::VectorXd gradient = MatrixXd::Zero(13,1);  //f or q
    // set the initial data of the QP solver
    qpsolver_.settings()->setVerbosity(false);

    qpsolver_.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    qpsolver_.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    if(!qpsolver_.data()->setHessianMatrix(hessian)) return 1;//设置H矩阵,二次项系数矩阵
    if(!qpsolver_.data()->setGradient(gradient)) return 1; //设置f矩阵。当没有时设置为全0向量，一次项系数矩阵
    if(!qpsolver_.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    if(!qpsolver_.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    if(!qpsolver_.data()->setUpperBound(upperBound)) return 1;//设置上边界

    // instantiate the qpsolver_
    if(!qpsolver_.initSolver()) return 1;

    // solve the QP problem
    if(!qpsolver_.solve()) 
    {
      ROS_ERROR("qpsolver fail!!!");
      return 1;
    }

    VectorXd QPSolution(12);
    // get the controller input
    QPSolution = qpsolver_.getSolution();

    for (int i=0; i<7; i++)
    {
        q_result(i) = q_current(i) + QPSolution(i)*(1.0/frequency);
    }

    // MatrixXd jacob_result;
    // std::vector<double> q_new(q_result.data(),q_result.data()+q_result.size());
    // if(!kin_solver->getJacobian(jacob_result, q_new, error_string))
    // {    return 1;  }
    // double K = jacob_result.norm()*jacob_result.inverse().norm();

    // if(K > 200)
    // {
    //     q_result(5)=q_current(5);
    // }

    // Eigen::JacobiSVD<Eigen::MatrixXd> svd_res(jacob_result);

    // double cond_res = svd_res.singularValues()(0) / svd_res.singularValues()(svd_res.singularValues().size()-1);
    // if(cond_res > cond_thres_){
    //   std::cout << "Quadratic Programming exits with error: cond of Jacobian matrix too large!!" << std::endl;
    //   return 1;
    // }
    qpsolver_.data()->clearHessianMatrix();
    qpsolver_.data()->clearLinearConstraintsMatrix();
    qpsolver_.clearSolver();

    return 0;
}

}
