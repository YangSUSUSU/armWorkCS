#include <ImpedanceControl.h>

ImpedanceControl::ImpedanceControl(double M, double B, double K, size_t state_dim, bool sensor_used, Mode mode)
                                   : state_dim_(state_dim), sensor_used_(sensor_used), mode_(mode){
    if(mode_ == kJointSpace){
        M_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * M;
        B_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * B;
        K_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * K;
        q_deflection_.resize(state_dim_);
        q_deflection_v_.resize(state_dim_);
        q_des_a_.resize(state_dim_);
    }
    else if(mode_ == kCartesianSpace){
        M_ = Eigen::MatrixXd::Identity(cartesian_dim_, cartesian_dim_) * M;
        B_ = Eigen::MatrixXd::Identity(cartesian_dim_, cartesian_dim_) * B;
        K_ = Eigen::MatrixXd::Identity(cartesian_dim_, cartesian_dim_) * K;
        q_deflection_.resize(cartesian_dim_);
        q_deflection_v_.resize(cartesian_dim_);
        q_des_a_.resize(cartesian_dim_);
    }

    showMsg();
}

ImpedanceControl::ImpedanceControl(const Eigen::MatrixXd& M, const Eigen::MatrixXd& B, const Eigen::MatrixXd& K, size_t state_dim, bool sensor_used, Mode mode)
: state_dim_(state_dim), sensor_used_(sensor_used), M_(M), B_(B), K_(K), mode_(mode){
    if(mode_ == kJointSpace){
        if (M.rows() != state_dim_ || M.cols() != state_dim_) {
            throw std::invalid_argument("Matrix M must be a square matrix of size state_dim_");
        }
        if (B.rows() != state_dim_ || B.cols() != state_dim_) {
            throw std::invalid_argument("Matrix B must be a square matrix of size state_dim_");
        }
        if (K.rows() != state_dim_ || K.cols() != state_dim_) {
            throw std::invalid_argument("Matrix K must be a square matrix of size state_dim_");
        }
        q_deflection_.resize(state_dim_);
        q_deflection_v_.resize(state_dim_);
        q_des_a_.resize(state_dim_);
    }
    else if(mode_ == kCartesianSpace){
        if (M.rows() != cartesian_dim_ || M.cols() != cartesian_dim_) {
            throw std::invalid_argument("Matrix M must be a square matrix of size cartesian_dim_");
        }
        if (B.rows() != cartesian_dim_ || B.cols() != cartesian_dim_) {
            throw std::invalid_argument("Matrix B must be a square matrix of size cartesian_dim_");
        }
        if (K.rows() != cartesian_dim_ || K.cols() != cartesian_dim_) {
            throw std::invalid_argument("Matrix K must be a square matrix of size cartesian_dim_");
        }
        q_deflection_.resize(cartesian_dim_);
        q_deflection_v_.resize(cartesian_dim_);
        q_des_a_.resize(cartesian_dim_);
    }

    showMsg();
 }

void ImpedanceControl::showMsg() const{
    std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << std::endl;

    std::cout << "ImpedanceControl object created successfully!" << std::endl;

    // Print the diagonal elements of the matrices
    std::cout << "M (Mass Matrix) Diagonal Elements: ";
    for (int i = 0; i < M_.rows(); ++i) {
        std::cout << M_(i, i) << " ";
    }
    std::cout << std::endl;

    std::cout << "B (Damping Matrix) Diagonal Elements: ";
    for (int i = 0; i < B_.rows(); ++i) {
        std::cout << B_(i, i) << " ";
    }
    std::cout << std::endl;

    std::cout << "K (Stiffness Matrix) Diagonal Elements: ";
    for (int i = 0; i < K_.rows(); ++i) {
        std::cout << K_(i, i) << " ";
    }
    std::cout << std::endl;

    std::cout << "Sensor used: " << (sensor_used_ ? "Yes" : "No, that means the mass matrix will represent the actual mass matrix. ") << std::endl;
    std::cout << "State Dimension: " << state_dim_ << std::endl;
    if(mode_ == kCartesianSpace){
        std::cout << "Impedance control defined in cartesian space." << std::endl;
    }
    else if(mode_ == kJointSpace){
        std::cout << "Impedance control defined in Joint space." << std::endl;
    }

    std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << std::endl;
}

bool ImpedanceControl::updateState(const ImpedanceControlState& state) {
    if(mode_ == kJointSpace){
        if (state.q_d.size() != state_dim_ || state.q_c.size() != state_dim_ || 
            state.q_d_v.size() != state_dim_ || state.q_c_v.size() != state_dim_ ||
            state.q_d_a.size() != state_dim_) {
            std::cerr << "Wrong state input size." << std::endl;
            return false;
        }
        q_deflection_ = state.q_d - state.q_c;
        q_deflection_v_ = state.q_d_v - state.q_c_v;
        q_des_a_ = state.q_d_a;
    }
    else if(mode_ == kCartesianSpace){        
        Eigen::AngleAxisd angle_axis(state.r_c * state.r_d.transpose());
        Eigen::Vector3d rotation_log = angle_axis.angle() * angle_axis.axis();
        q_deflection_.head(3) = state.x_d - state.x_c;
        q_deflection_.tail(3) = rotation_log;
        q_deflection_v_.head(3) = state.x_d_v - state.x_c_v;
        q_deflection_v_.tail(3) = state.r_d_v - state.r_c_v;
        q_des_a_.head(3) = state.x_d_a;
        q_des_a_.tail(3) = state.r_d_a;
    }

    // Force sensor
    if (sensor_used_) {
        if (state.force.size() != state_dim_) {
            std::cerr << "F/T sensor's msg missed or size mismatch." << std::endl;
            return false;
        } else {
            f_ = state.force;
        }
    }
    return true;
}

Eigen::VectorXd ImpedanceControl::getControlForceCartesian(const Eigen::MatrixXd& M_q, const Eigen::MatrixXd& Coriolis,
                            const Eigen::VectorXd& Gravity , const Eigen::MatrixXd& J_e, const Eigen::MatrixXd& J_e_dot, const Eigen::VectorXd& q_dot) const{
    Eigen::MatrixXd J_e_inv = get_inv(J_e);
    Eigen::MatrixXd M_inv = M_.inverse();
    if(sensor_used_){
        return (M_q * J_e_inv * M_inv) * (M_ * q_des_a_ + B_ * q_deflection_v_ + K_ * q_deflection_ - M_ * J_e_dot * q_dot) 
            + (J_e.transpose() - M_q * J_e_inv * M_inv) * f_ + Gravity + Coriolis * q_dot;
    }
    else{
        std::cout << "q_deflection_: " << q_deflection_.transpose() << std::endl;
        Eigen::MatrixXd M_new = J_e_inv.transpose() * M_q * J_e_inv ;
        return (J_e.transpose()) * ( B_ * q_deflection_v_ 
                + K_ * q_deflection_ - M_new * J_e_dot * q_dot) + Gravity + Coriolis * q_dot;
        return (J_e.transpose()) * ( -B_ * J_e * q_dot 
                + K_ * q_deflection_) + Gravity + Coriolis * q_dot;
    }
}

Eigen::VectorXd ImpedanceControl::getControlForceJoint(const Eigen::MatrixXd& M_q, const Eigen::MatrixXd& Coriolis
                                        , const Eigen::VectorXd& Gravity, const Eigen::VectorXd& q_dot) const{
    if(sensor_used_){
        Eigen::MatrixXd M_inv = M_.inverse();
        return M_q * q_des_a_ + Coriolis * q_dot + Gravity + M_q * M_inv * (B_ * q_deflection_v_ + K_ * q_deflection_) +
          (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - M_q * M_inv) * f_;
    }
    else{
        return M_q * q_des_a_ + Coriolis * q_dot + Gravity + (B_ * q_deflection_v_ + K_ * q_deflection_) ;
    }
}