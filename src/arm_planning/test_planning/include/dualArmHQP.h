// dual_arm_hqp.h

#ifndef DUAL_ARM_HQP_H
#define DUAL_ARM_HQP_H

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <memory>

class DualArmHQP {
public:
    DualArmHQP(int n_joints, int task_dim1, int task_dim2, double lambda1, double lambda2);

    bool solveFirstLevel(const Eigen::MatrixXd& J1, const Eigen::VectorXd& x_dot_d1);
    bool solveSecondLevel(const Eigen::MatrixXd& J2, const Eigen::VectorXd& q_dot_d2);

    Eigen::VectorXd getFirstLevelSolution() const;  
    Eigen::VectorXd getSecondLevelSolution() const;
    Eigen::VectorXd WQP(const Eigen::MatrixXd& Jl,
                                    const Eigen::MatrixXd& Jr,
                                    const Eigen::VectorXd& car_err,
                                    const Eigen::VectorXd& nowQ
                                    const Eigen::VectorXd& Qr);
private:
    int n_joints_;
    int task_dim1_, task_dim2_; // Task dimensions for level 1 and 2
    double lambda1_, lambda2_;  // Regularization weights

    Eigen::MatrixXd J1_; // Store J1 for second level

    Eigen::VectorXd solution1_; // Solution of first level
    Eigen::VectorXd solution2_; // Solution of second level

    std::shared_ptr<OsqpEigen::Solver> solver1_;
    std::shared_ptr<OsqpEigen::Solver> solver2_;
};

#endif // DUAL_ARM_HQP_H