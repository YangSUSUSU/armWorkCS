#include "trajectory_planner/traj_cp_planner.h"

using namespace std;
using namespace Eigen;

Traj_CP_Planner::Traj_CP_Planner(uint dof, double rate):TrajPlannerBase (dof, rate)
{
    ROS_INFO("Traj_CP_Planner constructor called!");
    duration = 0.0;
}

Traj_CP_Planner::~Traj_CP_Planner()
{
    delete allJoint._Joints;
    delete allJoint._CP_param;
    ROS_INFO("Traj_CP_Planner descructor called!");
}

void Traj_CP_Planner::parseWaypoints(std::vector<std::vector<double> > &initPathInfo)
{
    //have not complete
}

void Traj_CP_Planner::excute_plan()
{
    //have not complete
}


void Traj_CP_Planner::transformPoints(vector<vector<double>> &allPathPoints, vector<double> &timeStamp)
{
    CPJoint *joint = new CPJoint[DOF];
    CubicPolynomialParam *cp_param = new CubicPolynomialParam[DOF];

    uint num = allPathPoints.size();

    double **waypoints = new double*[DOF];
    double *time = new double[num];

    for (uint i=0; i<DOF; ++i)
    {
         waypoints[i] = new double[num];
    }

    for (uint i=0; i<num; ++i)
    {
        for (uint j=0; j<DOF; ++j)
        {
            waypoints[j][i] = allPathPoints[i][j];
        }
        time[i] = timeStamp[i];
    }

    for (uint i=0; i<DOF; ++i)
    {
        joint[i].len = num;
        joint[i].q = waypoints[i];
        joint[i].jt = time;
    }

    allJoint.num = DOF;
    allJoint._Joints = joint;
    allJoint._CP_param = cp_param;

}

void Traj_CP_Planner::CPInit(vector<vector<double>> &allPathPoints, vector<double> &timeStamp)
{
    transformPoints(allPathPoints, timeStamp);

    int num = allJoint.num;
    CPJoint *joints = allJoint._Joints;
    CubicPolynomialParam *cp = allJoint._CP_param;

    for(int i=0; i<num; i++)
    {
        CPSetup(0, &joints[i], &cp[i]);
    }
}


int Traj_CP_Planner::getJointTimeStep(double t, CPJoint *j)
{
    int step=0;
    for (int i = 0; i < j->len; i++)
    {
        if (t >= j->jt[i] && t < j->jt[i + 1])
        {
            step = i;
            break;
        }
    }
    return step;
}


double Traj_CP_Planner::calCPPosition(double t, CubicPolynomialParam *cp)
{
    int step=0;
    for (int i = 0; i < cp->cPoint_num-1; i++)
    {
        if (t >= cp->cti[i] && t < cp->cti[i + 1])
        {
            step = i;

            break;
        }

        if (t >= cp->cti[cp->cPoint_num-1])
        {
            double y = cp->cqi[cp->cPoint_num-1];
            return y;
        }
    }
    double y =  cp->ai[step] +
            cp->bi[step]*(t - cp->cti[step]) +
            cp->ci[step] * (t - cp->cti[step]) * (t - cp->cti[step]) +
            cp->di[step] * (t - cp->cti[step]) * (t - cp->cti[step]) * (t - cp->cti[step]);
    return y;
}


double Traj_CP_Planner::calCPSpeed(double t, CubicPolynomialParam *cp)
{
    int step=0;
    for (int i = 0; i < cp->cPoint_num-1; i++)
    {
        if (t >= cp->cti[i] && t < cp->cti[i + 1])
        {
            step = i;

            break;
        }

        if (t >= cp->cti[cp->cPoint_num-1])
        {
            double y = 0;
            return y;
        }
    }
    double y =      cp->bi[step] +
            2 * cp->ci[step]*(t - cp->cti[step]) +
            3 * cp->di[step] * (t - cp->cti[step]) * (t - cp->cti[step]);
    return y;
}


double Traj_CP_Planner::calCPAcceleration(double t, CubicPolynomialParam *cp)
{
    int step=0;
    for (int i = 0; i < cp->cPoint_num-1; i++)
    {
        if (t >= cp->cti[i] && t < cp->cti[i + 1])
        {
            step = i;

            break;
        }

        if (t >= cp->cti[cp->cPoint_num-1])
        {
            double y = 0;
            return y;
        }
    }
    double y =  2 * cp->ci[step] +
            6 * cp->di[step] * (t - cp->cti[step]);
    return y;
}


void Traj_CP_Planner::CPSetup(double startSpeed, CPJoint *j, CubicPolynomialParam *cp)
{
    cp->cPoint_num = j->len;

    int n = cp->cPoint_num - 1;

    int nSize = cp->cPoint_num;

    cp->cqi = new double[nSize];
    cp->cti = new double[nSize];

    cp->ai = new double[nSize];
    cp->bi = new double[nSize];
    cp->ci = new double[nSize];
    cp->di = new double[nSize];

    cp->h = new double[nSize];

    cp->A = new double[nSize];
    cp->B = new double[nSize];
    cp->C = new double[nSize];
    cp->D = new double[nSize];
    cp->E = new double[nSize];
    cp->M = new double[nSize];

    cp->flag = 1;

    //根据每个路径点的时间戳，计算两个路径点的时间间隔，存放在h中
    int i;
    for ( i = 0; i <= n-1; i++)
    {
        cp->h[i] = j->jt[i+1] - j->jt[i];
    }

    cp->A[0] = 0;
    cp->B[0] = 2 * cp->h[0];
    cp->C[0] = cp->h[0];
    cp->D[0] = 6 * ((j->q[1] - j->q[0]) / cp->h[0] - startSpeed/control_frequency);

    cp->A[n] = cp->h[n-1];     //h[n-1]最后一段轨迹对应的时间间隔
    cp->B[n] = 2 * cp->h[n-1];
    cp->C[n] = 0;
    cp->D[n] = 6 * (0.0 - (j->q[n] - j->q[n-1]) / cp->h[n-1]);   //q[n]最后一个路径点对应的关节位置

//    cout<<"csi->A[n]: "<<csi->A[n]<<endl;
//    cout<<"csi->B[n]: "<<csi->B[n]<<endl;
//    cout<<"csi->C[n]: "<<csi->C[n]<<endl;
//    cout<<"csi->D[n]: "<<csi->D[n]<<endl;

    for( i = 1; i<= n - 1; i++) cp->A[i] = cp->h[i-1];
    for( i = 1; i<= n - 1; i++) cp->B[i] = 2 * (cp->h[i] + cp->h[i-1]);
    for( i = 1; i<= n - 1; i++) cp->C[i] = cp->h[i];


    for (i = 1; i<=n-1; i++)
    {
        cp->D[i] = 6 * ((j->q[i+1] - j->q[i]) / cp->h[i] -
                (j->q[i] - j->q[i-1]) / cp->h[i-1]);
    }


    //根据ABCD，求E
    TDMA(cp->E, n+1, cp->A, cp->B, cp->C, cp->D);

    cp->M[0] = cp->E[0];
    cp->M[n] = cp->E[n];
    for(i=1; i<=n-1; i++)
    {
        cp->M[i] = cp->E[i];
    }

    for( i = 0; i <= n-1; i++)
    {
        cp->ai[i] = j->q[i];
        cp->bi[i] = (j->q[i+1] - j->q[i]) / cp->h[i] - (2 * cp->h[i] * cp->M[i] + cp->h[i] * cp->M[i + 1]) / 6;
        cp->ci[i] = cp->M[i] / 2;
        cp->di[i] = (cp->M[i + 1] - cp->M[i]) / (6 * cp->h[i]);
    }


    for( i = 0; i <= n; i++)
    {
        cp->cqi[i] = j->q[i];
        cp->cti[i] = j->jt[i];
    }
    cp->duration = cp->cti[n];
}


void Traj_CP_Planner::TDMA(double* X, const int n, double* A, double* B, double* C, double* D)
{
    int i;
    double tmp;

    C[0] = C[0] / B[0];
    D[0] = D[0] / B[0];

    for(i = 1; i<=n-1; i++)
    {
        tmp = (B[i] - A[i] * C[i-1]);
        C[i] = C[i] / tmp;
        D[i] = (D[i] - A[i] * D[i-1]) / tmp;
    }

    X[n-1] = D[n-1];

    for(i = n-2; i>=0; i--)
    {
        X[i] = D[i] - C[i] * X[i+1];
    }
}


