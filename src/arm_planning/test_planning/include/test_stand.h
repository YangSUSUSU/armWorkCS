// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>
#include <iostream>
#include <sensor_msgs/JointState.h>  
#include <std_msgs/String.h>
#include <nlohmann/json.hpp>
#include <interactive_markers/interactive_marker_server.h>

// #include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>  // 用于时间记录
// Pinocchio
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <array>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>

class wbc
{
private:
    /* data */
public:
     wbc();
    ~wbc();

    OsqpEigen::Solver solver;

    double mass = 0.0;
    Eigen::VectorXd Xcom;
    Eigen::VectorXd Xdcom;
    Eigen::VectorXd Xddcom;


    Eigen::VectorXd LFootPoint1;
    Eigen::VectorXd LFootPoint2;
    Eigen::VectorXd LFootPoint3;
    Eigen::VectorXd LFootPoint4;

    Eigen::VectorXd RFootPoint1;
    Eigen::VectorXd RFootPoint2;
    Eigen::VectorXd RFootPoint3;
    Eigen::VectorXd RFootPoint4;

    Eigen::VectorXd Com2RFootPoint;
    Eigen::VectorXd Com2LFootPoint;


    Eigen::MatrixXd JLFoot;
    Eigen::MatrixXd RLFoot;

    Eigen::MatrixXd Jcom;
    Eigen::MatrixXd Jd;

    Eigen::MatrixXd Jworld;
    Eigen::MatrixXd Jbody;
    
};
