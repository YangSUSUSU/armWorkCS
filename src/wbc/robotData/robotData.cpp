#include "robotData.h"

// 定义并初始化静态成员变量
RobotData* RobotData::instance = nullptr;

// 私有构造函数，确保外部无法直接创建实例
RobotData::RobotData() {

    robotState.Xcom = Eigen::VectorXd(3);
    robotState.Xdcom = Eigen::VectorXd(3);
    robotState.Xddcom = Eigen::VectorXd(3);

    robotState.LFootPoint1 = Eigen::VectorXd(3);
    robotState.LFootPoint2 = Eigen::VectorXd(3);
    robotState.LFootPoint3 = Eigen::VectorXd(3);
    robotState.LFootPoint4 = Eigen::VectorXd(3);

    robotState.RFootPoint1 = Eigen::VectorXd(3);
    robotState.RFootPoint2 = Eigen::VectorXd(3);
    robotState.RFootPoint3 = Eigen::VectorXd(3);
    robotState.RFootPoint4 = Eigen::VectorXd(3);

    robotState.Com2RFootPoint = Eigen::VectorXd(3);
    robotState.Com2LFootPoint = Eigen::VectorXd(3);

    robotState.JLFoot = Eigen::MatrixXd(3, 3);  // 举例初始化
    robotState.RLFoot = Eigen::MatrixXd(3, 3);  // 举例初始化

    robotState.Jcom = Eigen::MatrixXd(3, 3);  // 举例初始化
    robotState.Jd = Eigen::MatrixXd(3, 3);    // 举例初始化

    robotState.Jworld = Eigen::MatrixXd(3, 3); // 举例初始化
    robotState.Jbody = Eigen::MatrixXd(3, 3);  // 举例初始化
}

RobotData& RobotData::getInstance() 
{
    if (instance == nullptr) {
        instance = new RobotData();
    }
    return *instance;
}

// 设置机器人状态
void RobotData::setRobotState(const RobotState& state) {
    robotState = state;
}

// 获取机器人状态，返回结构体
void getRobotState() {
    // return robotState;
}
