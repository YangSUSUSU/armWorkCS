#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/uuid/uuid.hpp>
// #include <nlohmann/json.hpp>

namespace arm_kinematics {

enum DHType { DH };

class ArmKinematicsSolver{
public:
  ArmKinematicsSolver(std::string urdf_path, std::string base_link, std::string tip_link, std::vector<double> q_min, std::vector<double> q_max);
  ~ArmKinematicsSolver();

  size_t getDof();
  void getJointLimits(KDL::JntArray &q_min, KDL::JntArray &q_max);

  bool setFloatingBase(const Eigen::MatrixXd &matrix,
                       std::string &error_string);

  bool getFkSolution(Eigen::MatrixXd &matrix,
                     const std::vector<double> &q,
                     std::string &error_string);

  bool getIkSolution(std::vector<double> &q,
                     const Eigen::MatrixXd &matrix,
                     const std::vector<double> &q_init,
                     std::string &error_string);

  bool getJacobian(Eigen::MatrixXd &jac,
                   const std::vector<double> &q,
                   std::string &error_string);

  inline void setTracIkSolveType(TRAC_IK::SolveType type)
  {
    trac_ik_solver_->SetSolveType(type);
  }

private:
  void updateChain();

  KDL::Chain kdl_chain_;

  std::shared_ptr<KDL::ChainJntToJacSolver> kdl_jac_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  std::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;

  KDL::Jacobian kdl_jacobian_;

  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
};

}
