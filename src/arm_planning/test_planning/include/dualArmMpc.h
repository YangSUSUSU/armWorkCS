#ifndef DUALARMMPC_H  
#define DUALARMMPC_H  
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <array>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "robotPara.h"
class DualArmMpc
{
private:

    Eigen::VectorXd f_q;
    Eigen::VectorXd f_qd;
    Eigen::VectorXd f_c;
    Eigen::VectorXd f_u;

    Eigen::MatrixXd H_q;
    Eigen::MatrixXd H_qd;
    Eigen::MatrixXd H_c;
    Eigen::MatrixXd H_u;

    Eigen::VectorXd up;
    Eigen::VectorXd lp;

    Eigen::MatrixXd Constraints;
    Eigen::MatrixXd W1;
    Eigen::MatrixXd W2;
    Eigen::MatrixXd W3;
    Eigen::MatrixXd W4;

    Eigen::VectorXd q;
    Eigen::VectorXd qd;
    Eigen::VectorXd qr;
    Eigen::VectorXd qrd;

public:
     DualArmMpc(/* args */);
    ~DualArmMpc();
    RobotPara * robot;
    // init();

};




#endif 