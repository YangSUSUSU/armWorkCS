/**
 * @file ImpedanceControl.h
 * @brief head file of Impedance Controller
 * @version 1.0.0
 * @date 11.18 2024
 */
#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <iostream>

class ImpedanceControl {
public: 
    // Impedance mode : 0 -> Cartesian Space || 1 -> JointSpace
    enum Mode{
        kCartesianSpace = 0,
        kJointSpace = 1
    };
    // impedance control state
    struct ImpedanceControlState {
        // for joint space
        Eigen::VectorXd q_d;   // Desired position
        Eigen::VectorXd q_c;   // Current position
        Eigen::VectorXd q_d_v; // Desired velocity
        Eigen::VectorXd q_c_v; // Current velocity
        Eigen::VectorXd q_d_a; // Desired acceleration

        // for cartesian space
        Eigen::Matrix3d r_d;   // Desired rotation
        Eigen::Vector3d x_d;   // Desired translation
        Eigen::Matrix3d r_c;   // Current rotation
        Eigen::Vector3d x_c;   // Current translation

        Eigen::Vector3d r_d_v;   // Desired angular velocity
        Eigen::Vector3d x_d_v;   // Desired velocity
        Eigen::Vector3d r_c_v;   // Current angular velocity
        Eigen::Vector3d x_c_v;   // Current velocity

        Eigen::Vector3d r_d_a;   // Desired angular acceleration
        Eigen::Vector3d x_d_a;   // Desired acceleration

        Eigen::VectorXd force;   // Force sensor reading (optional)

        ImpedanceControlState() : q_d(Eigen::VectorXd::Zero(0)), q_c(Eigen::VectorXd::Zero(0)),
                                q_d_v(Eigen::VectorXd::Zero(0)), q_c_v(Eigen::VectorXd::Zero(0)),
                                q_d_a(Eigen::VectorXd::Zero(0)), force(Eigen::VectorXd::Zero(0)) {}
    };
    /** constructor */
    /**
     * @brief : Assign matrix with the same diagonal elements to the M, B, and K matrices.
     * @param [in] M: Mass
     * @param [in] B: Damping
     * @param [in] K: Stiff
     */
    explicit ImpedanceControl(double M, double B, double K, size_t state_dim, bool sensor_used, Mode mode);
    /**
     * @brief : Assign matrix with M, B, and K matrices.
     * @param [in] M: Mass Matrix
     * @param [in] B: Damping Matrix
     * @param [in] K: Stiff Matrix
     */
    explicit ImpedanceControl(const Eigen::MatrixXd& M, const Eigen::MatrixXd& B, const Eigen::MatrixXd& K, size_t state_dim, bool sensor_used, Mode mode);

    /** destructor */
    virtual ~ImpedanceControl() = default; 

    /**
     * Update the robot current state
     *
     * @param [in] q_d: Desired configuration
     * @param [in] q_c: current configuration
     * @param [in] q_d_v: Desired velocity
     * @param [in] q_c_v: current velocity
     * @param [in] q_d_a: Desired acceleration
     * @param [in] force: detected force from sensor
     * @return true if the state's dimention is correct
     */
    virtual bool updateState(const ImpedanceControlState& state);
    /**
     * @brief Get control force (Cartesian space)
     *
     * @param [in] M_q: Generalized mass matrix (orthogonal)
     * @param [in] Coriolis: Coriolis and centrifugal terms.
     * @param [in] Gravity: Gravitational terms.
     * @param [in] J_e: End-effector Jacobian matrix (mapping from joint velocities to Cartesian velocities)
     * @param [in] J_e_dot: Derivative of the Jacobian matrix (Jacobian velocity)
     * @param [in] q_dot: Joint velocity
     * @ref : https://www.bilibili.com/video/BV17T4y1K7yK?spm_id_from=333.788.videopod.episodes&p=4
     * @return Joint control torque
     */
    virtual Eigen::VectorXd getControlForceCartesian(const Eigen::MatrixXd& M_q, const Eigen::MatrixXd& Coriolis, const Eigen::VectorXd& Gravity , 
                                             const Eigen::MatrixXd& J_e, const Eigen::MatrixXd& J_e_dot, const Eigen::VectorXd& q_dot) const;
    /**
     * @brief Get control force (Joint space)
     *
     * @param [in] M_q: Generalized mass matrix (orthogonal)
     * @param [in] Coriolis: Coriolis and centrifugal terms.
     * @param [in] Gravity: Gravitational terms.
     * @param [in] q_dot: Joint velocity
     * @ref : https://www.bilibili.com/video/BV17T4y1K7yK?spm_id_from=333.788.videopod.episodes&p=3
     * @return  joint control torque
     */
    virtual Eigen::VectorXd getControlForceJoint(const Eigen::MatrixXd& M_q, const Eigen::MatrixXd& Coriolis
                                        , const Eigen::VectorXd& Gravity, const Eigen::VectorXd& q_dot) const;
protected:
    /**
    * @brief Get pseudoinverse matrix
    * @detail
    *  Get pseudoinverse matrix of the input matrix
    * @param[in] input_matrix        input matrix
    * @param[out] get_inv  (MatrixXd input_matrix)        the pseudoinverse matrix of input matrix
    */
    Eigen::MatrixXd get_inv(const Eigen::MatrixXd &input_matrix, const double threshold = 0.3) const
    {
        // SVD Singular Value Decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd U = svd.matrixU();
        Eigen::MatrixXd V = svd.matrixV();
        Eigen::MatrixXd S = U.inverse() * input_matrix * V.transpose().inverse();

        // In avoiad of Singurity
        for (int i = 0; i < S.rows(); i++)
        {

            if (S(i, i) < threshold)
            {
                S(i, i) = 1.0 / threshold;
            }
            else
            {
                S(i, i) = 1 / S(i, i);
            }
        }

        // Return a inverse matrix
        Eigen::MatrixXd inv;
        inv = V * S.transpose() * U.transpose();
        return inv;
    }

    // show the Impedence control settings.
    virtual void showMsg() const;
    // Impedance parameter, Mass, Damping and Stiffness
    Eigen::MatrixXd M_, B_, K_;
    // configuration deflection
    Eigen::VectorXd q_deflection_;
    // velocity deflection
    Eigen::VectorXd q_deflection_v_;
    // Desired acceleration
    Eigen::VectorXd q_des_a_;
    // detected force
    Eigen::VectorXd f_;
    // state Dim 
    const size_t state_dim_;
    // if the external force can be used or not
    const bool sensor_used_;
    // joint space or cartesian space
    const Mode mode_;
    // ee size
    static constexpr int cartesian_dim_ = 6;
};