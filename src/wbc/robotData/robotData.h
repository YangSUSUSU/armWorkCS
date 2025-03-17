// robotData.
#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H
#include <map>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <pinocchio/multibody/joint/joint-generic.hpp> // 确保包含这个文件
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "robotStructs.h"
// 单例模式的机器人数据管理类
class RobotData {
public:
    static RobotData& getInstance();
    RobotData(const RobotData&) = delete;
    RobotData& operator=(const RobotData&) = delete;
    void setRobotState(RobotStructs& state);
    RobotStructs getRobotState();
    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw);




private:

    pinocchio::Model modelFixed;
    pinocchio::Model modelFree;
    pinocchio::Data dataFixed;
    pinocchio::Data dataFree;
    RobotData();
    static RobotData* instance;
    RobotStructs robotStructs;
};

#endif // ROBOT_DATA_H
