#include <iostream>
#include <Eigen/Dense>

class PDController {
public:
    // // 构造函数，初始化比例增益和微分增益
    // PDController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd)
    //     : Kp(Kp), Kd(Kd), prev_position(Eigen::VectorXd::Zero(Kp.size())),
    //       prev_velocity(Eigen::VectorXd::Zero(Kp.size())), prev_time(0.0) {}
    PDController() {
        // 这里直接初始化 Kp 和 Kd
        Kp = Eigen::VectorXd::Constant(17, 1.0);  // 初始化为 17 维，值为 200
        Kd = Eigen::VectorXd::Constant(17, 0);   // 初始化为 17 维，值为 50
        Kp(3)= 2*Kp(3);
        Kp(10)= 2*Kp(10);
        // 初始化 prev_position 和 prev_velocity
        prev_position = Eigen::VectorXd::Zero(Kp.size());  // 和 Kp 的大小一致
        prev_velocity = Eigen::VectorXd::Zero(Kd.size());  // 和 Kd 的大小一致

        prev_time = 0.0;  // 初始化 prev_time 为 0
    }
    // 控制器更新函数
    Eigen::VectorXd computeControl(const Eigen::VectorXd& desired_position,
                                   const Eigen::VectorXd& actual_position,
                                   double current_time) {
        // 计算时间差（防止没有时间差的情况）
        double dt = current_time - prev_time;
        if (dt <= 0.0) dt = 1e-6; // 防止除以零

        // 计算当前速度（通过前向差分法）
        Eigen::VectorXd current_velocity = (actual_position - prev_position) / dt;

        // 计算期望速度（假设期望速度是期望位置的变化率，使用前向差分法）desired_position
        Eigen::VectorXd desired_velocity = ( - prev_position) / dt;

        // 计算位置误差和速度误差
        Eigen::VectorXd position_error = desired_position - actual_position;
        Eigen::VectorXd velocity_error = desired_velocity - current_velocity;
        std::cout<< position_error(5)<<std::endl;
        // 计算控制信号：PD 控制公式
        Eigen::VectorXd control_input = Kp.cwiseProduct(position_error) + Kd.cwiseProduct(velocity_error);

        // 更新前一次的位置和时间
        prev_position = actual_position;
        prev_velocity = current_velocity;
        prev_time = current_time;

        return control_input;
    }

private:
    Eigen::VectorXd Kp;             // 比例增益向量
    Eigen::VectorXd Kd;             // 微分增益向量
    Eigen::VectorXd prev_position;  // 前一次的实际位置
    Eigen::VectorXd prev_velocity;  // 前一次的速度
    double prev_time;               // 前一次的时间
};

// int main() {
//     // 设置比例增益和微分增益，假设有 7 个关节
//     int num_joints = 7;
//     Eigen::VectorXd Kp(num_joints);
//     Eigen::VectorXd Kd(num_joints);

//     // 设置比例和微分增益的值
//     Kp.setConstant(100.0);  // 所有关节的比例增益为 100
//     Kd.setConstant(20.0);   // 所有关节的微分增益为 20

//     // 创建 PD 控制器
//     PDController pd_controller(Kp, Kd);

//     // 模拟一些关节的期望和实际值
//     Eigen::VectorXd desired_position(num_joints);
//     Eigen::VectorXd actual_position(num_joints);

//     // 设定期望位置和实际位置
//     desired_position.setConstant(1.0);  // 期望所有关节的位置为 1.0（单位：弧度）
//     actual_position.setConstant(0.8);   // 当前实际位置为 0.8（单位：弧度）

//     // 模拟时间（单位：秒）
//     double current_time = 0.1;

//     // 计算控制输入
//     Eigen::VectorXd control_input = pd_controller.computeControl(desired_position, actual_position, current_time);

//     // 输出控制输入
//     std::cout << "Control input: \n" << control_input << std::endl;

//     return 0;
// }
