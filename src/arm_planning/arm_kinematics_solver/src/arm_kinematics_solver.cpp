#include <arm_kinematics_solver/arm_kinematics_solver.h>

namespace arm_kinematics {

ArmKinematicsSolver::ArmKinematicsSolver(std::string urdf_path, std::string base_link, std::string tip_link, std::vector<double> q_min, std::vector<double> q_max)
{
  KDL::Tree tree;
  KDL::Chain chain;
  kdl_parser::treeFromFile(urdf_path, tree);

  tree.getChain(base_link, tip_link, chain);

  kdl_chain_.addSegment(KDL::Segment("floating_base"));
  kdl_chain_.addChain(chain);

  q_min_.data=Eigen::Map<const Eigen::VectorXd>(&q_min[0], long(q_min.size()));
  q_max_.data=Eigen::Map<const Eigen::VectorXd>(&q_max[0], long(q_max.size()));

  updateChain();
}

ArmKinematicsSolver::~ArmKinematicsSolver()
{

}

void ArmKinematicsSolver::updateChain()
{
  kdl_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  trac_ik_solver_.reset(new TRAC_IK::TRAC_IK(kdl_chain_, q_min_, q_max_, 3.0));

  kdl_jacobian_.resize(kdl_chain_.getNrOfJoints());
}

size_t ArmKinematicsSolver::getDof()
{
  return kdl_chain_.getNrOfJoints();
}

void ArmKinematicsSolver::getJointLimits(KDL::JntArray &q_min, KDL::JntArray &q_max)
{
  q_min=q_min_;
  q_max=q_max_;
}

bool ArmKinematicsSolver::setFloatingBase(const Eigen::MatrixXd &matrix,
                                          std::string &error_string)
{
  Eigen::Isometry3d iso3d;
  iso3d.matrix()=matrix;

  KDL::Frame frame;
  tf::transformEigenToKDL(iso3d, frame);

  kdl_chain_.segments[0].setFrameToTip(frame);

  return true;
}

bool ArmKinematicsSolver::getFkSolution(Eigen::MatrixXd &matrix,
                                        const std::vector<double> &q,
                                        std::string &error_string)
{
  KDL::Frame frame;
  KDL::JntArray kdl_jnt_array;

  kdl_jnt_array.data=Eigen::Map<const Eigen::VectorXd>(&q[0], long(q.size()));
  int ret=kdl_fk_solver_->JntToCart(kdl_jnt_array, frame);

  if(ret!=kdl_jac_solver_->E_NOERROR)
  {
    error_string=kdl_jac_solver_->strError(ret);
    return false;
  }

  Eigen::Isometry3d iso3d;
  tf::transformKDLToEigen(frame, iso3d);

  matrix=iso3d.matrix();

  return true;
}

bool ArmKinematicsSolver::getIkSolution(std::vector<double> &q,
                                        const Eigen::MatrixXd &matrix,
                                        const std::vector<double> &q_init,
                                        std::string &error_string)
{
  KDL::Frame frame;
  KDL::JntArray kdl_jnt_array;
  KDL::JntArray kdl_jnt_array_init;

  kdl_jnt_array_init.data=Eigen::Map<const Eigen::VectorXd>(&q_init[0], long(q_init.size()));

  Eigen::Isometry3d iso3d;
  iso3d.matrix()=matrix;
  tf::transformEigenToKDL(iso3d, frame);

  int ret=trac_ik_solver_->CartToJnt(kdl_jnt_array_init, frame, kdl_jnt_array);

  if(ret<=0)
  {
    error_string="Failed to get ik solution";
    return false;
  }

  q=std::vector<double>(kdl_jnt_array.data.data(), kdl_jnt_array.data.data()+kdl_jnt_array.data.size());

  return true;
}

bool ArmKinematicsSolver::getJacobian(Eigen::MatrixXd &jac,
                                      const std::vector<double> &q,
                                      std::string &error_string)
{
  if(q.size()!=kdl_chain_.getNrOfJoints())
  {
    error_string="Wrong length of q";
    return false;
  }

  KDL::JntArray kdl_jnt_array;

  kdl_jnt_array.data=Eigen::Map<const Eigen::VectorXd>(&q[0], long(q.size()));
  int ret=kdl_jac_solver_->JntToJac(kdl_jnt_array, kdl_jacobian_);

  if(ret!=kdl_jac_solver_->E_NOERROR)
  {
    error_string=kdl_jac_solver_->strError(ret);
    return false;
  }

  jac=kdl_jacobian_.data;

  return true;
}


}
