#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>

#include "sVelocityPlanner.h"
#include "StructDefine.h"

class Interpolation
{
public:
    //generate the interpolation point list according to the specified interpolation type
    std::vector<InterpolationPara> GenerateInterpPoints(std::vector<VelocityPlanPara> &allVelocityPlanParas);

    //conduct the joint interpolation
    int JointInterpolation(VelocityPlanPara &velPlanPara, std::vector<std::vector<double>> &interPoints);

    //conduct the cartesian interpolation
    int Decare_LInterpolation(VelocityPlanPara &velPlanPara, std::vector<std::vector<double>> &interPoints);

    //calculate the u parameter according to the input time for joint interpolation
    std::vector<double> CalJointByTime(double time, PlanningParameter thePlanPara);

    //calculate the u parameter according to the input time for cartesian interpolation, override +2
    std::vector<double> CalDecareByTime(double time, PlanningParameter thePlanPara, PlanningParameter thePlanParaSecond);
    std::vector<double> CalDecareByTime(double time, PlanningParameter thePlanParaFirst);

    //calculate the transit point according to the specified precision for multi-point cartesian interpolation
    void CalculateTransitPointNum(std::vector<InterpolationPara> &interPolationPara);

    //convert quaternion to rpy
    Eigen::VectorXd quaternion2rpy(Eigen::Quaterniond quaternion);

    Eigen::VectorXd transR2RPY(Eigen::Matrix3d R,std::string xyz);

    //convert quaternion to rotation matrix
    Eigen::Matrix3d quaternion2rotation(Eigen::Quaterniond quaternion);

    //convert rotation matrix to quaternion
    Eigen::Quaterniond rotation2quaternion(Eigen::Matrix3d rotationMatrix);

    //conduct the orientation interpolation
    Eigen::VectorXd slerp(Eigen::VectorXd starting, Eigen::VectorXd ending, double u);

};

#endif // INTERPOLATION_H
