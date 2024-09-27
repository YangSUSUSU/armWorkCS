#ifndef trajectory_planner_BASE_H
#define trajectory_planner_BASE_H

#include "ros/ros.h"

#include <vector>
#include <eigen3/Eigen/Eigen>

class TrajPlannerBase
{
public:
    typedef enum
    {
        S_Veclocity     = 1,
        TimeOptimal     = 2,
        CubicPolynomial = 3,
        Topp            = 4
    }PlanType;

    TrajPlannerBase(uint dof, double rate, ros::NodeHandle *n);
    virtual ~TrajPlannerBase();

    static std::vector<double> T_matrix2Quat(Eigen::Matrix4d T_matrix);
    static Eigen::Matrix4d Quat2T_matrix(std::vector<double> waypoint);
    Eigen::Quaterniond Euler2Quat(Eigen::VectorXd rpy);
    Eigen::VectorXd Quat2Euler(Eigen::Quaterniond quaternion);
    static Eigen::Matrix3d Euler2Rotation(Eigen::VectorXd rpy);
    static Eigen::VectorXd Rotation2Euler(Eigen::Matrix3d rotationMatrix);


protected:
    const uint DOF;
    const double control_frequency;
    ros::NodeHandle *nh;


};

#endif // trajectory_planner_BASE_H
