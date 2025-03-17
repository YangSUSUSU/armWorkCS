// #include <OsqpEigen/OsqpEigen.h>
// #include <Eigen/Dense>
// #include <iostream>

// class QPSolver {
// public:
//     QPSolver() 
//     {
//         // 设置求解器参数
//         solver.settings()->setVerbosity(false); // 关闭冗余输出
//         solver.settings()->setWarmStart(true);  // 启用热启动
//     }

//     bool solve( Eigen::MatrixXd& H,  Eigen::VectorXd& f,
//                 Eigen::MatrixXd& A,  Eigen::VectorXd& lbA,  Eigen::VectorXd& ubA) {
//         // 获取问题的维度
//         int numVariables = H.cols();
//         int numConstraints = A.rows();

//         // 初始化求解器
//         solver.data()->setNumberOfVariables(numVariables);
//         solver.data()->setNumberOfConstraints(numConstraints);

//         // 设置 Hessian 矩阵 (H)
//         if (!solver.data()->setHessianMatrix(H)) {
//             std::cerr << "Failed to set Hessian matrix." << std::endl;
//             return false;
//         }

//         // 设置梯度向量 (f)
//         if (!solver.data()->setGradient(f)) {
//             std::cerr << "Failed to set gradient vector." << std::endl;
//             return false;
//         }

//         // 设置约束矩阵 (A)
//         if (!solver.data()->setLinearConstraintsMatrix(A)) {
//             std::cerr << "Failed to set constraint matrix." << std::endl;
//             return false;
//         }

//         // 设置约束下界 (lbA)
//         if (!solver.data()->setLowerBound(lbA)) {
//             std::cerr << "Failed to set lower bound." << std::endl;
//             return false;
//         }

//         // 设置约束上界 (ubA)
//         if (!solver.data()->setUpperBound(ubA)) {
//             std::cerr << "Failed to set upper bound." << std::endl;
//             return false;
//         }

//         // 初始化求解器
//         if (!solver.initSolver()) {
//             std::cerr << "Failed to initialize solver." << std::endl;
//             return false;
//         }

//         // 求解问题
//         return solver.solve();
//     }

//     Eigen::VectorXd getSolution() const 
//     {
//         return solver.getSolution();
//     }

// private:
//     OsqpEigen::Solver solver; // OSQP-Eigen 求解器对象
// };

// // int main() {
// //     // 定义 QP 问题的参数
// //     Eigen::MatrixXd H(2, 2); // Hessian 矩阵
// //     H << 4, 1,
// //          1, 2;

// //     Eigen::VectorXd f(2); // 梯度向量
// //     f << 1, 1;

// //     Eigen::MatrixXd A(1, 2); // 约束矩阵
// //     A << 1, 1;

// //     Eigen::VectorXd lbA(1); // 约束下界
// //     lbA << 1;

// //     Eigen::VectorXd ubA(1); // 约束上界
// //     ubA << 1;

// //     // 创建 QPSolver 对象并求解
// //     QPSolver qpSolver;
// //     if (qpSolver.solve(H, f, A, lbA, ubA)) {
// //         Eigen::VectorXd solution = qpSolver.getSolution();
// //         std::cout << "Solution: " << solution.transpose() << std::endl;
// //     } else {
// //         std::cerr << "Failed to solve QP problem." << std::endl;
// //     }

// //     return 0;
// // }