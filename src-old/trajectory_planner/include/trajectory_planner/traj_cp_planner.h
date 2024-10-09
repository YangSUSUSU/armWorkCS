#ifndef TRAJ_CP_PLANNER_H
#define TRAJ_CP_PLANNER_H

#include "trajectory_planner/traj_planner_base.h"

//cubic polynomial plan for motion trajectory
class Traj_CP_Planner:public TrajPlannerBase
{
public:
    typedef struct
    {
        int flag;

        int cPoint_num;

        double duration;

        double *cqi;

        double *cti;

        double *ai;
        double *bi;
        double *ci;
        double *di;

        double *h;
        double *A;
        double *B;
        double *C;
        double *D;
        double *E;
        double *M;
    }CubicPolynomialParam;

    typedef struct
    {
        int len;
        double *q;
        double *jt;

    }CPJoint;

    typedef struct
    {
      CubicPolynomialParam *_CP_param;

      CPJoint *_Joints;

      int num;

    }CPList;


    Traj_CP_Planner(uint dof, double rate);
    ~Traj_CP_Planner();

    void parseWaypoints(std::vector<std::vector<double>> &initPathInfo);
    void excute_plan();

    void transformPoints(std::vector<std::vector<double>> &allPathPoints, std::vector<double> &timeStamp);
    void CPInit(std::vector<std::vector<double>> &allPathPoints, std::vector<double> &timeStamp);
    void CPSetup(double startSpeed, CPJoint *j, CubicPolynomialParam *cp);
    void TDMA(double *X, const int n, double *A, double *B, double *C, double *D);
    double calCPPosition(double t, CubicPolynomialParam *cp);
    double calCPSpeed(double t, CubicPolynomialParam *cp);
    double calCPAcceleration(double t, CubicPolynomialParam *cp);

    int getJointTimeStep(double t, CPJoint *j);

    CPList allJoint;

private:
//    std::vector<std::vector<double>> waypoints;
    int plan_method;
    double duration;

};

#endif // TRAJ_CP_PLANNER_H
