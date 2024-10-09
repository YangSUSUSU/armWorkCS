#ifndef TRAJ_S_PLANNER_H
#define TRAJ_S_PLANNER_H

#include "trajectory_planner/traj_planner_base.h"
#include "trajectory_planner/StructDefine.h"
#include "trajectory_planner/interpolation.h"

//the s profile velocity plan for motion trajectory, include joint and cartesian space
class Traj_S_Planner:public TrajPlannerBase
{
public:

    Traj_S_Planner(uint dof, double rate);
    ~Traj_S_Planner();
    void transformPoints(std::vector<std::vector<double>> &allPathPoints, std::vector<SinglePoint> &trajAllPoints, std::string interpSpaceTypeName);
    std::vector<VelocityPlanPara> parseCommand(std::vector<SinglePoint> &trajAllPoints, double trajVelocity, std::string interpSpaceTypeName);
    double initInterpolation(std::vector<std::vector<double>> &allPathPoints, double trajVelocity, std::string interpSpaceTypeName);
    bool calCartesianInterpolation(double time, const Eigen::Matrix4d &lastJointPosition, Eigen::Matrix4d &T);

private:
    std::vector<std::vector<double>> waypoints;
    int plan_method;
    double veclocity;
    bool isCartesian;

    //instantiate an Interpolation object
    Interpolation interpolation_Obj;

    //a vector saved the all trajectory planning infomation
    std::vector<PlanningParameter> m_allPlanningParameters;

    //save the duration of the robot move along the planned trajectory
    double cal_duration;

    //save the time instant for each waypoint
    std::vector<double> m_timeList;

};

#endif // TRAJ_S_PLANNER_H
