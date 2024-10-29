#pragma once

#include <arm_kinematics_solver/arm_kinematics_solver.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

namespace arm_kinematics {

class ArmQpIkSolver{
public:
  ArmQpIkSolver();
  ArmQpIkSolver(double cond_thres, double joint_limit_thres);
  ~ArmQpIkSolver();

  Eigen::VectorXd getRPY(Eigen::Matrix3d T_matrix);

  bool getIK_QP(ArmKinematicsSolver *kin_solver,
                Eigen::VectorXd &q,
                const double &frequency,
                const Eigen::MatrixXd &T_end_des,
                const Eigen::VectorXd &q_now,
                std::string &error_string);
  Eigen::VectorXd IKqpSolution(ArmKinematicsSolver *kin_solver,const double &frequency,const Eigen::Matrix4d &T, const Eigen::VectorXd &q_current,std::string &error_string,std::vector<double> joints_min,std::vector<double> joints_max);
  int qpSolution(ArmKinematicsSolver *kin_solver,const Eigen::VectorXd &q_current,const double &frequency, const Eigen::VectorXd &cartesian_vel, Eigen::VectorXd &q_result,std::vector<double> joints_min,std::vector<double> joints_max);
  

private:
  int solveIK_withOSQP(ArmKinematicsSolver *kin_solver,
                        const Eigen::VectorXd &q_current,
                        const Eigen::VectorXd &cartesian_vel,
                        const double &frequency,
                        Eigen::VectorXd &q_result,
                        Eigen::VectorXd &qp_vel,
                        Eigen::VectorXd &qp_acc);

  OsqpEigen::Solver qpsolver_;
  double cond_thres_;
  double joint_limit_thres_;
  double max_abs_cart_linear_vel_;
  double max_abs_omega_;
  Eigen::VectorXd g_last_QPSolution = Eigen::VectorXd(13);
};

}
