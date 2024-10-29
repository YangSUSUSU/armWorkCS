#include <Eigen/Dense>
#include <osqp.h>
#include <iostream>

Eigen::VectorXd computeNullSpaceVelocityWithOSQP(
    const Eigen::MatrixXd& jaco, 
    const Eigen::VectorXd& joint_angles, 
    const Eigen::VectorXd& delta_q_desired, 
    const Eigen::VectorXd& joint_limits_min, 
    const Eigen::VectorXd& joint_limits_max, 
    const Eigen::VectorXd& prev_delta_q, 
    double weight_limit_avoidance, 
    double weight_singularity_avoidance, 
    double weight_smoothness) {

    int num_joints = joint_angles.size();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_joints, num_joints);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(num_joints);

    // 1. 关节限位规避项
    for (int i = 0; i < num_joints; ++i) {
        double mid_point = (joint_limits_min[i] + joint_limits_max[i]) / 2.0;
        H(i, i) += weight_limit_avoidance;
        f(i) -= weight_limit_avoidance * (mid_point - joint_angles[i]);
    }

    // 2. 奇异性规避项
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jaco);
    Eigen::MatrixXd jaco_pseudo_inv = cod.pseudoInverse();
    Eigen::MatrixXd null_space_projection = Eigen::MatrixXd::Identity(num_joints, num_joints) - jaco_pseudo_inv * jaco;

    // 生成随机单位向量并投影到零空间中
    Eigen::VectorXd u = Eigen::VectorXd::Random(num_joints); 
    Eigen::VectorXd null_space_unit_vector = null_space_projection * u;
    null_space_unit_vector.normalize();

    // 在目标函数中添加奇异性规避项
    H += weight_singularity_avoidance * null_space_projection.transpose() * null_space_projection;

    // 3. 速度平滑项
    for (int i = 0; i < num_joints; ++i) {
        H(i, i) += weight_smoothness;
        f(i) -= weight_smoothness * prev_delta_q[i];
    }

    // 设定QP约束
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_joints, num_joints);
    Eigen::VectorXd lower_bound = joint_limits_min - joint_angles;
    Eigen::VectorXd upper_bound = joint_limits_max - joint_angles;

    // 使用OSQP求解
    c_int num_variables = H.rows();
    c_int num_constraints = A.rows();
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();

    OSQPSettings* settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData* data = (OSQPData *)c_malloc(sizeof(OSQPData));

    osqp_set_default_settings(settings);
    settings->alpha = 1.0;

    data->n = num_variables;
    data->m = num_constraints;
    data->P = csc_matrix(num_variables, num_variables, H_sparse.nonZeros(), H_sparse.valuePtr(), H_sparse.innerIndexPtr(), H_sparse.outerIndexPtr());
    data->q = f.data();
    data->A = csc_matrix(num_constraints, num_variables, A_sparse.nonZeros(), A_sparse.valuePtr(), A_sparse.innerIndexPtr(), A_sparse.outerIndexPtr());
    data->l = lower_bound.data();
    data->u = upper_bound.data();

    OSQPWorkspace* work = osqp_setup(data, settings);
    osqp_solve(work);

    Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(num_variables);
    if (work->info->status_val == OSQP_SOLVED) {
        delta_q = Eigen::Map<Eigen::VectorXd>(work->solution->x, num_variables);
    }

    osqp_cleanup(work);
    if (settings) c_free(settings);
    if (data) {
        if (data->P) c_free(data->P);
        if (data->A) c_free(data->A);
        c_free(data);
    }

    return delta_q;
}

int main() {
    Eigen::MatrixXd jaco; // 定义和赋值雅克比矩阵
    Eigen::VectorXd joint_angles; // 当前关节角度
    Eigen::VectorXd delta_q_desired; // 期望的关节增量
    Eigen::VectorXd joint_limits_min; // 关节下限
    Eigen::VectorXd joint_limits_max; // 关节上限
    Eigen::VectorXd prev_delta_q; // 前一时刻的关节增量

    double weight_limit_avoidance = 1.0;
    double weight_singularity_avoidance = 0.5;
    double weight_smoothness = 0.3;

    Eigen::VectorXd delta_q = computeNullSpaceVelocityWithOSQP(
        jaco, joint_angles, delta_q_desired, 
        joint_limits_min, joint_limits_max, 
        prev_delta_q, 
        weight_limit_avoidance, weight_singularity_avoidance, 
        weight_smoothness
    );

    std::cout << "零空间分配的关节增量: \n" << delta_q << std::endl;

    return 0;
}
