    
#include "dualArmHQP.h"    
    
    DualArmHQP(int n_joints, int task_dim1, int task_dim2, double lambda1, double lambda2)
        : n_joints_(n_joints), task_dim1_(task_dim1), task_dim2_(task_dim2), lambda1_(lambda1), lambda2_(lambda2) 
    {
        // Initialize the OSQP solver objects for each level
        solver1_ = std::make_shared<OsqpEigen::Solver>();
        solver2_ = std::make_shared<OsqpEigen::Solver>();
    }

    // Set up and solve the first level
    bool DualArmHQP::solveFirstLevel(const Eigen::MatrixXd& J1, const Eigen::VectorXd& x_dot_d1) 
    {
        int n_vars = n_joints_ + task_dim1_;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);

        // Construct H and f
        H.block(0, 0, n_joints_, n_joints_) = J1.transpose() * J1;
        H.block(n_joints_, n_joints_, task_dim1_, task_dim1_) = lambda1_ * Eigen::MatrixXd::Identity(task_dim1_, task_dim1_);
        f.head(n_joints_) = -J1.transpose() * x_dot_d1;

        // Constraints (Inequality)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * task_dim1_, n_vars);
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(2 * task_dim1_, -1e9);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(2 * task_dim1_);

        // A = [J1 -I; -J1 -I]
        A.block(0, 0, task_dim1_, n_joints_) = J1;
        A.block(0, n_joints_, task_dim1_, task_dim1_) = -Eigen::MatrixXd::Identity(task_dim1_, task_dim1_);
        A.block(task_dim1_, 0, task_dim1_, n_joints_) = -J1;
        A.block(task_dim1_, n_joints_, task_dim1_, task_dim1_) = -Eigen::MatrixXd::Identity(task_dim1_, task_dim1_);

        // Update upper bounds
        ub.head(task_dim1_) = x_dot_d1;
        ub.tail(task_dim1_) = -x_dot_d1;

        // Configure and solve with OSQP
        solver1_->data()->setNumberOfVariables(n_vars);
        solver1_->data()->setNumberOfConstraints(2 * task_dim1_);
        if (!solver1_->data()->setHessianMatrix(H.sparseView())) return false;
        if (!solver1_->data()->setGradient(f)) return false;
        if (!solver1_->data()->setLinearConstraintsMatrix(A.sparseView())) return false;
        if (!solver1_->data()->setBounds(lb, ub)) return false;

        solver1_->initSolver();
        if (solver1_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

        // Retrieve solution
        solution1_ = solver1_->getSolution();
        return true;
    }

    // Set up and solve the second level
    bool DualArmHQP::solveSecondLevel(const Eigen::MatrixXd& J2, const Eigen::VectorXd& q_dot_d2) 
    {
        int n_vars = n_joints_ + task_dim2_;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);

        // Construct H and f
        H.block(0, 0, n_joints_, n_joints_) = J2.transpose() * J2;
        H.block(n_joints_, n_joints_, task_dim2_, task_dim2_) = lambda2_ * Eigen::MatrixXd::Identity(task_dim2_, task_dim2_);
        f.head(n_joints_) = -J2.transpose() * q_dot_d2;

        // Constraints (Equality)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(task_dim1_, n_vars);
        Eigen::VectorXd lb = Eigen::VectorXd::Zero(task_dim1_);
        Eigen::VectorXd ub = Eigen::VectorXd::Zero(task_dim1_);

        // Use first layer's solution as constraints for second layer
        A.block(0, 0, task_dim1_, n_joints_) = J1_;
        lb = solution1_.head(task_dim1_);
        ub = solution1_.head(task_dim1_);

        // Configure and solve with OSQP
        solver2_->data()->setNumberOfVariables(n_vars);
        solver2_->data()->setNumberOfConstraints(task_dim1_);
        if (!solver2_->data()->setHessianMatrix(H.sparseView())) return false;
        if (!solver2_->data()->setGradient(f)) return false;
        if (!solver2_->data()->setLinearConstraintsMatrix(A.sparseView())) return false;
        if (!solver2_->data()->setBounds(lb, ub)) return false;

        solver2_->initSolver();
        if (solver2_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

        // Retrieve solution
        solution2_ = solver2_->getSolution();
        return true;
    }
    Eigen::VectorXd DualArmHQP::WQP(const Eigen::MatrixXd& Jl,
                                    const Eigen::MatrixXd& Jr,
                                    const Eigen::VectorXd& car_err,
                                    const Eigen::VectorXd& nowQ
                                    const Eigen::VectorXd& Qr)
    {

            double eta_car = 1.0;
            double eta_qpos= 1.0;                    
            Eigen::MatrixXd H = eta_car*Jl.transpose()*Jl 
                            + eta_car*Jr.transpose()*Jr 
                            + eta_qpos * Eigen::MatrixXd::Identity(18, 18);
            Eigen::VectorXd c = - Jl.transpose()*car_err.head(6) 
                                - Jr.transpose()*car_err.tail(6)
                                - Qr;
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(48,18);
            A.block<6,18>(0,0) = Jl;
            A.block<6,18>(6,0) = Jr;
            A.block<18,18>(12,0) = Eigen::MatrixXd::Identity(18, 18);
            A.block<18,18>(30,0) = 1/400*5*Eigen::MatrixXd::Identity(18, 18);

            //==========约束1 笛卡尔线速度角速度
            Eigen::VectorXd Car_max = Eigen::VectorXd::Zero(12);
            Car_max.setConstant(0.1);

            Eigen::VectorXd Car_min = Eigen::VectorXd::Zero(12);
            Car_min.setConstant(0.1);
            //==========约束2 关节速度

            Eigen::VectorXd qv_max = Eigen::VectorXd::Zero(18);
            qv_max.setConstant(0.1);
            
            Eigen::VectorXd qv_min = Eigen::VectorXd::Zero(18);
            qv_min .setConstant(0.1);
            //==========约束3 关节位置
            // △q ＜ qmax -qnow
            Eigen::VectorXd q_max = Eigen::VectorXd::Zero(18);
            Eigen::VectorXd q_min = Eigen::VectorXd::Zero(18);
            q_max<<0,0.25,0.25,0.25, //腰部关节
            0.5,2.96,3.4,2.96,0,2.96,1.04,1.48,//l
            0.5,2.96,3.4,2.96,0,2.96,1.04,1.48;//r
            q_min<<-0.15,-0.25,-0.25,-0.25,
            -0.5,-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66,
            -0.5,-2.96,-0.26,-2.96,-2.96,-2.96,-1.04,-1.66;
            //
            Eigen::VectorXd up = Eigen::VectorXd::Zero(48);
            Eigen::VectorXd lp = Eigen::VectorXd::Zero(48);



            Eigen::VectorXd car_temp_ = Eigen::VectorXd::Zero(6);
            car_temp_<< 0.1,0.1,0.1,
                        0.1,0.1,0.1;
            up.head(6) = car_temp_ ;
            up.segment(6, 6) = car_temp_;
            lp.head(6) = -car_temp_ ;
            lp.segment(6, 6) = -car_temp_;

            up.segment(12, 18) = qv_max;
            lp.segment(12, 18) = qv_min;

            up.segment(30, 18) = q_max-nowQ;
            lp.segment(30, 18) = q_min-nowQ;


            OsqpEigen::Solver solver;



            // solver.settings()->setWarmStart(true);
            solver.settings()->setVerbosity(false);
            solver.data()->setNumberOfVariables(18);
            solver.data()->setNumberOfConstraints(48);
            Eigen::SparseMatrix<double> H_sparse = H.sparseView();
            if (!solver.data()->setHessianMatrix(H_sparse)) 
            {
                std::cerr << "Error setting Hessian matrix" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }

            if (!solver.data()->setGradient(c)) {
                std::cerr << "Error setting gradient vector" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }
            Eigen::SparseMatrix<double> A_sparse = A.sparseView();
            if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
                std::cerr << "Error setting constraint matrix" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }

            if (!solver.data()->setLowerBound(lp)) {
                std::cerr << "Error setting lower bounds" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }

            if (!solver.data()->setUpperBound(up)) {
                std::cerr << "Error setting upper bounds" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }

            if (!solver.initSolver()) {
                std::cerr << "Solver initialization failed" << std::endl;
                return Eigen::VectorXd::Zero(n);
            }

            auto result = solver.solve();
            return solver.getSolution();
    }

    // Getters for solutions
    Eigen::VectorXd DualArmHQP::getFirstLevelSolution() const { return solution1_; }
    Eigen::VectorXd DualArmHQP::getSecondLevelSolution() const { return solution2_; }