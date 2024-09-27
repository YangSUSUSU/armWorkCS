#include <iostream>
#include <cmath>
#include <cstdlib>
#include <algorithm>

#include "trajectory_planner/sVelocityPlanner.h"
#include "trajectory_planner/interpolation.h"

using namespace std;
using namespace Eigen;

//default constructor
SVelocityPlanner::SVelocityPlanner()
{
  m_SPlanPara.startVel = 0.0;
  m_SPlanPara.constVel = 0.0;
  m_SPlanPara.endVel = 0.0;
  m_SPlanPara.acc = 0.0;
  m_SPlanPara.dec = 0.0;
  m_SPlanPara.jerk = 0.0;

  m_interpolationCycle = InterpolationCycle;

  m_SPlanPara.length = 0.0;

  m_SPlanPara.T1 = 0.0;
  m_SPlanPara.T2 = 0.0;
  m_SPlanPara.T3 = 0.0;
  m_SPlanPara.T4 = 0.0;
  m_SPlanPara.T5 = 0.0;
  m_SPlanPara.T6 = 0.0;
  m_SPlanPara.T7 = 0.0;

  m_Num1 = 0;
  m_Num2 = 0;
  m_Num3 = 0;
  m_Num4 = 0;
  m_Num5 = 0;
  m_Num6 = 0;
  m_Num7 = 0;

  m_Num12 = 0;
  m_Num13 = 0;
  m_Num14 = 0;
  m_Num15 = 0;
  m_Num16 = 0;

  m_TotalNum = 0;

  m_SPlanPara.vel1 = 0.0;
  m_SPlanPara.vel2 = 0.0;
  m_SPlanPara.vel3 = 0.0;
  m_SPlanPara.vel4 = 0.0;
  m_SPlanPara.vel5 = 0.0;
  m_SPlanPara.vel6 = 0.0;
  m_SPlanPara.vel7 = 0.0;

  m_SPlanPara.dis1 = 0.0;
  m_SPlanPara.dis2= 0.0;
  m_SPlanPara.dis3= 0.0;
  m_SPlanPara.dis4= 0.0;
  m_SPlanPara.dis5= 0.0;
  m_SPlanPara.dis6= 0.0;
  m_SPlanPara.dis7= 0.0;

  m_SPlanPara.jerk1 = 0.0;
  m_SPlanPara.jerk3 = 0.0;
  m_SPlanPara.jerk5 = 0.0;
  m_SPlanPara.jerk7 = 0.0;
}

// SVelocityPlanner::SVelocityPlanner(const SVelocityPlanner &cp)
// {
//   m_SPlanPara.startVel = cp.m_SPlanPara.startVel;
//   m_SPlanPara.constVel = cp.m_SPlanPara.constVel;
//   m_SPlanPara.endVel = cp.m_SPlanPara.endVel;
//   m_SPlanPara.acc = cp.m_SPlanPara.acc;
//   m_SPlanPara.dec = cp.m_SPlanPara.dec;
//   m_SPlanPara.jerk = cp.m_SPlanPara.jerk;
//   m_interpolationCycle = cp.m_interpolationCycle;
// }

SVelocityPlanner::~SVelocityPlanner()
{   
}

/**
 * @brief - initialize the acceleration and deceleration
 */
void SVelocityPlanner::initAcceDece()
{
    if ((m_SPlanPara.constVel - m_SPlanPara.startVel) >= 0)
    {
        m_SPlanPara.acc = abs(m_SPlanPara.acc);
    }
    else
    {
        m_SPlanPara.acc = -abs(m_SPlanPara.acc);
    }

    if ((m_SPlanPara.constVel - m_SPlanPara.endVel) >= 0)
    {
        m_SPlanPara.dec = abs(m_SPlanPara.dec);
    }
    else
    {
        m_SPlanPara.dec = -abs(m_SPlanPara.dec);
    }
}

/**
 * @brief - This function initializes the velocity plan parameter for joint S planning
 */
void SVelocityPlanner::initJointSPara(VelocityPlanPara &velPara)
{
    std::vector<double> startJointValue = velPara.waypoints[0];
    std::vector<double> endJointValue   = velPara.waypoints[1];

    double len_sq = 0;
    for (int i=0; i<RobotDOF; i++)
    {
        len_sq = len_sq + pow((endJointValue[i] - startJointValue[i]),2.0);
    }
    double len = sqrt(len_sq);

    velPara.sPlanPara.length = len;

    m_SPlanPara.length   = velPara.sPlanPara.length;
    m_SPlanPara.startVel = velPara.sPlanPara.startVel;
    m_SPlanPara.constVel = velPara.sPlanPara.constVel;
    m_SPlanPara.endVel   = velPara.sPlanPara.endVel;
    m_SPlanPara.acc      = velPara.sPlanPara.acc;
    m_SPlanPara.dec      = velPara.sPlanPara.dec;
    m_SPlanPara.jerk     = velPara.sPlanPara.jerk;
}

void SVelocityPlanner::initDecare_LSPara(VelocityPlanPara &velPara)
{
    double startX, startY, startZ;
    double endX, endY, endZ;

    startX = velPara.waypoints[0][0];
    startY = velPara.waypoints[0][1];
    startZ = velPara.waypoints[0][2];

    endX = velPara.waypoints[1][0];
    endY = velPara.waypoints[1][1];
    endZ = velPara.waypoints[1][2];

    double len_sq = pow((endX - startX),2.0) + pow((endY - startY),2.0) + pow((endZ - startZ),2.0);

    if(len_sq < 0.001)
    {
        Quaterniond quaternionS;
        Quaterniond quaternionE;

        quaternionS.x() = velPara.waypoints[0][3];
        quaternionS.y() = velPara.waypoints[0][4];
        quaternionS.z() = velPara.waypoints[0][5];
        quaternionS.w() = velPara.waypoints[0][6];

        quaternionE.x() = velPara.waypoints[1][3];
        quaternionE.y() = velPara.waypoints[1][4];
        quaternionE.z() = velPara.waypoints[1][5];
        quaternionE.w() = velPara.waypoints[1][6];

        Interpolation theCartesianInterp;

        Vector3d rpyS = theCartesianInterp.quaternion2rpy(quaternionS);
        Vector3d rpyE = theCartesianInterp.quaternion2rpy(quaternionE);


        startX = rpyS(0);
        startY = rpyS(1);
        startZ = rpyS(2);

        endX = rpyE(0);
        endY = rpyE(1);
        endZ = rpyE(2);

        len_sq = (endX - startX)*(endX - startX) + (endY - startY)*(endY - startY) + (endZ - startZ)*(endZ - startZ); // you may try to use all 6 values to calc len
    }

    velPara.sPlanPara.length = sqrt(len_sq);

    m_SPlanPara.length   = velPara.sPlanPara.length;
    m_SPlanPara.startVel = velPara.sPlanPara.startVel;
    m_SPlanPara.constVel = velPara.sPlanPara.constVel;
    m_SPlanPara.endVel   = velPara.sPlanPara.endVel;
    m_SPlanPara.acc      = velPara.sPlanPara.acc;
    m_SPlanPara.dec      = velPara.sPlanPara.dec;
    m_SPlanPara.jerk     = velPara.sPlanPara.jerk;
}

/**
 * @brief - This function returns the total time stamps number
 */
int SVelocityPlanner::getTotalNumPara()
{
    return m_TotalNum;
}

/**
 * @brief - This function returns the path length
 */
double SVelocityPlanner::getLength()
{
    return m_SPlanPara.length;
}

/**
 * @brief - This function checks the jerk condition and limit the jerk
 */
void SVelocityPlanner::checkJerkCondition()
{
    double Ts = m_interpolationCycle;
    double tmp1 = abs(m_SPlanPara.acc) / Ts;
    double tmp2 = abs(m_SPlanPara.dec) / Ts;

    double tmp3 = min(tmp1,tmp2);

    if (m_SPlanPara.jerk > tmp3)
    {
        m_SPlanPara.jerk = tmp3;
    }

    if (m_SPlanPara.acc >= 0)
    {
        m_SPlanPara.jerk1 = abs(m_SPlanPara.jerk);
    }
    else
    {
        m_SPlanPara.jerk1 = -abs(m_SPlanPara.jerk);
    }

    if (m_SPlanPara.dec >= 0)
    {
        m_SPlanPara.jerk5 = abs(m_SPlanPara.jerk);
    }
    else
    {
        m_SPlanPara.jerk5 = -abs(m_SPlanPara.jerk);
    }

    m_SPlanPara.jerk3 = m_SPlanPara.jerk1;
    m_SPlanPara.jerk7 = m_SPlanPara.jerk5;
}

/**
 * reference - https://blog.csdn.net/u010632165/article/details/104951091
 */
void SVelocityPlanner::checkAccCondition()
{
    m_SPlanPara.T2 = (m_SPlanPara.constVel - m_SPlanPara.startVel) / m_SPlanPara.acc - m_SPlanPara.acc / m_SPlanPara.jerk1;
            
    if (m_SPlanPara.T2 < 0) //will change acce
    {
        m_SPlanPara.T2 = 0;

        if (m_SPlanPara.acc >= 0)
        {
            m_SPlanPara.acc = sqrt(m_SPlanPara.jerk1 * (m_SPlanPara.constVel - m_SPlanPara.startVel));
        }
        else
        {
            m_SPlanPara.acc = -sqrt(m_SPlanPara.jerk1 * (m_SPlanPara.constVel - m_SPlanPara.startVel));
        }
    }
}


/**
 * reference - https://blog.csdn.net/u010632165/article/details/104951091
 */
void SVelocityPlanner::checkDecCondition()
{
    m_SPlanPara.T6 = (m_SPlanPara.constVel - m_SPlanPara.endVel) / m_SPlanPara.dec - m_SPlanPara.dec / m_SPlanPara.jerk5;

    if (m_SPlanPara.T6 < 0) //will change dece
    {
        m_SPlanPara.T6 = 0;

        if (m_SPlanPara.dec >= 0)
        {
            m_SPlanPara.dec = sqrt(m_SPlanPara.jerk5 * (m_SPlanPara.constVel - m_SPlanPara.endVel));
        }
        else
        {
            m_SPlanPara.dec = -sqrt(m_SPlanPara.jerk5 * (m_SPlanPara.constVel - m_SPlanPara.endVel));
        }
    }

}


int SVelocityPlanner::checkTotalLenCondition()
{
    int Rt;
    double Ts;
    double T1, T2, T3, T4, T5, T6, T7;
    double fs, x, y, Len, F;

    Rt = 0;
    Ts = m_interpolationCycle;

    m_SPlanPara.T1 = abs(m_SPlanPara.acc / m_SPlanPara.jerk1);
    m_SPlanPara.T3 = abs(m_SPlanPara.acc / m_SPlanPara.jerk3);
    m_SPlanPara.T5 = abs(m_SPlanPara.dec / m_SPlanPara.jerk5);
    m_SPlanPara.T7 = abs(m_SPlanPara.dec / m_SPlanPara.jerk7);

    T1 = m_SPlanPara.T1;
    T2 = m_SPlanPara.T2;
    T3 = m_SPlanPara.T3;
    T4 = m_SPlanPara.T4;
    T5 = m_SPlanPara.T5;
    T6 = m_SPlanPara.T6;
    T7 = m_SPlanPara.T7;

    fs = m_SPlanPara.startVel;
    x = m_SPlanPara.acc;
    y = m_SPlanPara.dec;
    Len = m_SPlanPara.length;
    F = m_SPlanPara.constVel;

    m_SPlanPara.T4 = -(x * T1 * T1 + 3 * x * T1 * T2 + 3 * x * T1 * T3
           + 6 * fs * T1 + 3 * x * T2 * T2 + 6 * x * T2 * T3
           + 6 * fs * T2 + 2 * x * T3 * T3 + 6 * fs * T3 - y * T5 * T5
           - 3 * y * T5 * T6 - 3 * y * T5 * T7 + 6 * F * T5 - 3 * y * T6 * T6
           - 6 * y * T6 * T7 + 6 * F * T6 - 2 * y * T7 * T7 + 6 * F * T7 - 6 * Len) / (6 * F);

    if (m_SPlanPara.T4 < 0) // will change Vmax(velConst)
    {
        m_SPlanPara.T4 = 0;

        m_SPlanPara.constVel = -(x * T1 * T1 + 3 * x * T1 * T2 + 3 * x * T1 * T3 + 6 * fs * T1 + 3 * x * T2 * T2
                        + 6 * x * T2 * T3 + 6 * fs * T2 + 2 * x * T3 * T3 + 6 * fs * T3 - y * T5 * T5
                        - 3 * y * T5 * T6 - 3 * y * T5 * T7 - 3 * y * T6 * T6
                        - 6 * y * T6 * T7 - 2 * y * T7 * T7
                        - 6 * Len) / (6 * T5 + 6 * T6 + 6 * T7);

        if (m_SPlanPara.constVel < 0)
        {
            Rt = -1;
        }
        else
        {
            Rt = 0;
        }
    }
    else
    {
        Rt = 1;
    }

    return Rt;
}


void SVelocityPlanner::updateADcc()
{
    double Ts;
    double T1, T2, T3, T4, T5, T6, T7;
    double fs, fe, x, y, Len, F;

    Ts = m_interpolationCycle;

    T1 = m_Num1 * Ts;
    T2 = m_Num2 * Ts;
    T3 = m_Num3 * Ts;
    T4 = m_Num4 * Ts;
    T5 = m_Num5 * Ts;
    T6 = m_Num6 * Ts;
    T7 = m_Num7 * Ts;

    fs = m_SPlanPara.startVel;
    fe = m_SPlanPara.endVel;
    x = m_SPlanPara.acc;
    y = m_SPlanPara.dec;
    Len = m_SPlanPara.length;
    F = m_SPlanPara.constVel;

    x = -(2 * (T5 * T5 * fe + 3 * T6 * T6 * fe + 2 * T7 * T7 * fe
        + 2 * T5 * T5 * fs + 3 * T6 * T6 * fs + T7 * T7 * fs - 3 * Len * T5
        - 6 * Len * T6 - 3 * Len * T7 + 3 * T5 * T6 * fe + 3 * T5 * T7 * fe
        + 6 * T6 * T7 * fe + 3 * T1 * T5 * fs + 6 * T1 * T6 * fs + 3 * T2 * T5 * fs
        + 3 * T1 * T7 * fs + 6 * T2 * T6 * fs + 3 * T3 * T5 * fs + 3 * T2 * T7 * fs
        + 6 * T3 * T6 * fs + 3 * T4 * T5 * fs + 3 * T3 * T7 * fs + 6 * T4 * T6 * fs
        + 3 * T4 * T7 * fs + 6 * T5 * T6 * fs + 3 * T5 * T7 * fs + 3 * T6 * T7 * fs))
        / (T1 * T1 * T5 + 2 * T1 * T1 * T6 + T1 * T1 * T7 + 3 * T1 * T2 * T5
        + 6 * T1 * T2 * T6 + 3 * T1 * T2 * T7 + 3 * T1 * T3 * T5 + 6 * T1 * T3 * T6
        + 3 * T1 * T3 * T7 + 2 * T1 * T5 * T5 + 6 * T1 * T5 * T6 + 3 * T1 * T5 * T7
        + 3 * T4 * T1 * T5 + 3 * T1 * T6 * T6 + 3 * T1 * T6 * T7 + 6 * T4 * T1 * T6
        + T1 * T7 * T7 + 3 * T4 * T1 * T7 + 3 * T2 * T2 * T5 + 6 * T2 * T2 * T6
        + 3 * T2 * T2 * T7 + 6 * T2 * T3 * T5 + 12 * T2 * T3 * T6 + 6 * T2 * T3 * T7
        + 4 * T2 * T5 * T5 + 12 * T2 * T5 * T6 + 6 * T2 * T5 * T7 + 6 * T4 * T2 * T5
        + 6 * T2 * T6 * T6 + 6 * T2 * T6 * T7 + 12 * T4 * T2 * T6 + 2 * T2 * T7 * T7
        + 6 * T4 * T2 * T7 + 2 * T3 * T3 * T5 + 4 * T3 * T3 * T6 + 2 * T3 * T3 * T7
        + 2 * T3 * T5 * T5 + 6 * T3 * T5 * T6 + 3 * T3 * T5 * T7 + 3 * T4 * T3 * T5
        + 3 * T3 * T6 * T6 + 3 * T3 * T6 * T7 + 6 * T4 * T3 * T6 + T3 * T7 * T7 + 3 * T4 * T3 * T7);
    

    y = -(2 * (T1 * T1 * fe + 3 * T2 * T2 * fe + 2 * T3 * T3 * fe + 2 * T1 * T1 * fs
        + 3 * T2 * T2 * fs + T3 * T3 * fs - 3 * Len * T1 - 6 * Len * T2 - 3 * Len * T3
        + 3 * T1 * T2 * fe + 3 * T1 * T3 * fe + 3 * T1 * T4 * fe + 6 * T2 * T3 * fe
        + 3 * T1 * T5 * fe + 6 * T2 * T4 * fe + 3 * T1 * T6 * fe + 6 * T2 * T5 * fe
        + 3 * T3 * T4 * fe + 3 * T1 * T7 * fe + 6 * T2 * T6 * fe + 3 * T3 * T5 * fe
        + 6 * T2 * T7 * fe + 3 * T3 * T6 * fe + 3 * T3 * T7 * fe + 6 * T1 * T2 * fs
        + 3 * T1 * T3 * fs + 3 * T2 * T3 * fs)) / (T1 * T1 * T5 + 2 * T1 * T1 * T6
        + T1 * T1 * T7 + 3 * T1 * T2 * T5 + 6 * T1 * T2 * T6 + 3 * T1 * T2 * T7
        + 3 * T1 * T3 * T5 + 6 * T1 * T3 * T6 + 3 * T1 * T3 * T7 + 2 * T1 * T5 * T5
        + 6 * T1 * T5 * T6 + 3 * T1 * T5 * T7 + 3 * T4 * T1 * T5 + 3 * T1 * T6 * T6
        + 3 * T1 * T6 * T7 + 6 * T4 * T1 * T6 + T1 * T7 * T7 + 3 * T4 * T1 * T7
        + 3 * T2 * T2 * T5 + 6 * T2 * T2 * T6 + 3 * T2 * T2 * T7 + 6 * T2 * T3 * T5
        + 12 * T2 * T3 * T6 + 6 * T2 * T3 * T7 + 4 * T2 * T5 * T5 + 12 * T2 * T5 * T6
        + 6 * T2 * T5 * T7 + 6 * T4 * T2 * T5 + 6 * T2 * T6 * T6 + 6 * T2 * T6 * T7
        + 12 * T4 * T2 * T6 + 2 * T2 * T7 * T7 + 6 * T4 * T2 * T7 + 2 * T3 * T3 * T5
        + 4 * T3 * T3 * T6 + 2 * T3 * T3 * T7 + 2 * T3 * T5 * T5 + 6 * T3 * T5 * T6
        + 3 * T3 * T5 * T7 + 3 * T4 * T3 * T5 + 3 * T3 * T6 * T6 + 3 * T3 * T6 * T7
        + 6 * T4 * T3 * T6 + T3 * T7 * T7 + 3 * T4 * T3 * T7);


    m_SPlanPara.acc = x;
    m_SPlanPara.dec = y;

    m_SPlanPara.jerk1 = x / T1;
    m_SPlanPara.jerk3 = x / T3;

    m_SPlanPara.jerk5 = y / T5;
    m_SPlanPara.jerk7 = y / T7;
}


void SVelocityPlanner::updateVelocity()
{
    double Ts;
    double T1, T2, T3, T4, T5, T6, T7;
    double F, fs, fe, A, D, x, y;
    double J1, J3, J5, J7;

    Ts = m_interpolationCycle;

    T1 = m_Num1 * Ts;
    T2 = m_Num2 * Ts;
    T3 = m_Num3 * Ts;
    T4 = m_Num4 * Ts;
    T5 = m_Num5 * Ts;
    T6 = m_Num6 * Ts;
    T7 = m_Num7 * Ts;

    F = m_SPlanPara.constVel;
    fs = m_SPlanPara.startVel;
    fe = m_SPlanPara.endVel;

    A = m_SPlanPara.acc;
    D = m_SPlanPara.dec;
    x = m_SPlanPara.acc;
    y = m_SPlanPara.dec;

    J1 = m_SPlanPara.jerk1;
    J3 = m_SPlanPara.jerk3;
    J5 = m_SPlanPara.jerk5;
    J7 = m_SPlanPara.jerk7;

    m_SPlanPara.vel1 = fs + 0.5 * J1 * T1 * T1;
    m_SPlanPara.vel2 = m_SPlanPara.vel1 + A * T2;
    m_SPlanPara.vel3 = m_SPlanPara.vel2 + A * T3 - 0.5 * J3 * T3 * T3;
    m_SPlanPara.vel4 = m_SPlanPara.vel3;
    m_SPlanPara.vel5 = m_SPlanPara.vel4 - 0.5 * J5 * T5 * T5;
    m_SPlanPara.vel6 = m_SPlanPara.vel5 - D * T6;
    m_SPlanPara.vel7 = m_SPlanPara.vel6 - D * T7 + 0.5 * J7 * T7 * T7;

    m_SPlanPara.dis1 = fs * T1 + J1 * T1 * T1 * T1 * 1 / 6;
    m_SPlanPara.dis2 = m_SPlanPara.dis1 + m_SPlanPara.vel1 * T2 + 0.5 * x * T2 * T2;
    m_SPlanPara.dis3 = m_SPlanPara.dis2 + m_SPlanPara.vel2 * T3 + 0.5 * x * T3 * T3 - J3 * T3 * T3 * T3 * 1 / 6;
    m_SPlanPara.dis4 = m_SPlanPara.dis3 + m_SPlanPara.vel3 * T4;
    m_SPlanPara.dis5 = m_SPlanPara.dis4 + m_SPlanPara.vel4 * T5 - (J5 * T5 * T5 * T5 * 1) / 6;
    m_SPlanPara.dis6 = m_SPlanPara.dis5 + m_SPlanPara.vel5 * T6 - 0.5 * y * T6 * T6;
    m_SPlanPara.dis7 = m_SPlanPara.dis6 + m_SPlanPara.vel6 * T7 - 0.5 * y * T7 * T7 + J7 * T7 * T7 * T7 * 1 / 6;
}


double SVelocityPlanner::cal_UByPosition(double J, double A, double f, double Tm, double u0)
{
    double u;
    u = u0 + f * Tm + A * Tm * Tm / 2 + J * Tm * Tm * Tm / 6;
    return u;
}

/**
 * @brief - This function calculates all parameters for S plan
 */
int SVelocityPlanner::cal_SPara(VelocityPlanPara &velPlanPara)
{
    double Ts;
    Ts = m_interpolationCycle;

    initAcceDece();

    checkJerkCondition(); //checks the jerk condition and limit the jerk
    checkAccCondition();  //check T2 and possibly change Acce
    checkDecCondition();  //check T6 and possibly change Dece

    int Rt = checkTotalLenCondition();

    if (Rt == -1)
    {
        m_SPlanPara.startVel = 0;
        m_SPlanPara.endVel = 0;

        initAcceDece();
        checkJerkCondition();
        checkAccCondition();
        checkDecCondition();
    }


    if (Rt == 0)
    {
        initAcceDece();
        checkJerkCondition();
        checkAccCondition();
        checkDecCondition();
    }

    m_Num1 = 1 + static_cast<int>(m_SPlanPara.T1 / Ts);
    m_Num2 = 1 + static_cast<int>(m_SPlanPara.T2 / Ts);
    m_Num3 = 1 + static_cast<int>(m_SPlanPara.T3 / Ts);
    m_Num4 = 1 + static_cast<int>(m_SPlanPara.T4 / Ts);
    m_Num5 = 1 + static_cast<int>(m_SPlanPara.T5 / Ts);
    m_Num6 = 1 + static_cast<int>(m_SPlanPara.T6 / Ts);
    m_Num7 = 1 + static_cast<int>(m_SPlanPara.T7 / Ts);

    m_TotalNum = m_Num1 + m_Num2 + m_Num3 + m_Num4 + m_Num5 + m_Num6 + m_Num7;

    m_Num12 = m_Num1 + m_Num2;
    m_Num13 = m_Num1 + m_Num2 + m_Num3;
    m_Num14 = m_Num1 + m_Num2 + m_Num3 + m_Num4;
    m_Num15 = m_Num1 + m_Num2 + m_Num3 + m_Num4 + m_Num5;
    m_Num16 = m_Num1 + m_Num2 + m_Num3 + m_Num4 + m_Num5 + m_Num6;


    velPlanPara.sPlanPara.T1 = m_Num1*Ts;
    velPlanPara.sPlanPara.T2 = m_Num2*Ts;
    velPlanPara.sPlanPara.T3 = m_Num3*Ts;
    velPlanPara.sPlanPara.T4 = m_Num4*Ts;
    velPlanPara.sPlanPara.T5 = m_Num5*Ts;
    velPlanPara.sPlanPara.T6 = m_Num6*Ts;
    velPlanPara.sPlanPara.T7 = m_Num7*Ts;

    velPlanPara.sPlanPara.totalTime = m_TotalNum*Ts;

    updateADcc();
    updateVelocity();

    velPlanPara.sPlanPara.acc = m_SPlanPara.acc;
    velPlanPara.sPlanPara.dec = m_SPlanPara.dec;

    velPlanPara.sPlanPara.jerk = m_SPlanPara.jerk;
    velPlanPara.sPlanPara.jerk1 = m_SPlanPara.jerk1;
    velPlanPara.sPlanPara.jerk3 = m_SPlanPara.jerk3;
    velPlanPara.sPlanPara.jerk5 = m_SPlanPara.jerk5;
    velPlanPara.sPlanPara.jerk7 = m_SPlanPara.jerk7;

    velPlanPara.sPlanPara.vel1 = m_SPlanPara.vel1;
    velPlanPara.sPlanPara.vel2 = m_SPlanPara.vel2;
    velPlanPara.sPlanPara.vel3 = m_SPlanPara.vel3;
    velPlanPara.sPlanPara.vel4 = m_SPlanPara.vel4;
    velPlanPara.sPlanPara.vel5 = m_SPlanPara.vel5;
    velPlanPara.sPlanPara.vel6 = m_SPlanPara.vel6;
    velPlanPara.sPlanPara.vel7 = m_SPlanPara.vel7;

    velPlanPara.sPlanPara.dis1 = m_SPlanPara.dis1;
    velPlanPara.sPlanPara.dis2 = m_SPlanPara.dis2;
    velPlanPara.sPlanPara.dis3 = m_SPlanPara.dis3;
    velPlanPara.sPlanPara.dis4 = m_SPlanPara.dis4;
    velPlanPara.sPlanPara.dis5 = m_SPlanPara.dis5;
    velPlanPara.sPlanPara.dis6 = m_SPlanPara.dis6;
    velPlanPara.sPlanPara.dis7 = m_SPlanPara.dis7;

    return Rt;
}

/**
 * @brief - This function calculates the parameter for u
 */
double SVelocityPlanner::cal_UForLine(int index)
{
    double CurJ = 0;
    double CurA = 0;
    double Curf = 0;
    double CurTm = 0;
    double CurU0 = 0;
    double CurU = 0;
    double Num = 0;

    if (index <= m_Num1 && index > 0)
    {
        CurJ = m_SPlanPara.jerk1;
        CurA = 0;
        Curf = m_SPlanPara.startVel;
        CurTm = index * m_interpolationCycle;
        CurU0 = 0;
    }

    if (index > m_Num1 && index <= m_Num12)
    {
        Num = (index - m_Num1) * m_interpolationCycle;

        CurJ = 0;
        CurA = m_SPlanPara.acc;
        Curf = m_SPlanPara.vel1;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis1;
    }

    if (index > m_Num12 && index <= m_Num13)
    {
        Num = (index - m_Num12) * m_interpolationCycle;

        CurJ = -m_SPlanPara.jerk3;
        CurA = m_SPlanPara.acc;
        Curf = m_SPlanPara.vel2;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis2;
    }

    if (index > m_Num13 && index <= m_Num14)
    {
        Num = (index - m_Num13) * m_interpolationCycle;

        CurJ = 0;
        CurA = 0;
        Curf = m_SPlanPara.vel3;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis3;
    }

    if (index > m_Num14 && index <= m_Num15)
    {
        Num = (index - m_Num14) * m_interpolationCycle;

        CurJ = -m_SPlanPara.jerk5;
        CurA = 0;
        Curf = m_SPlanPara.vel4;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis4;
    }

    if (index > m_Num15 && index <= m_Num16)
    {
        Num = (index - m_Num15) * m_interpolationCycle;

        CurJ = 0;
        CurA = -m_SPlanPara.dec;
        Curf = m_SPlanPara.vel5;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis5;
    }

    if (index > m_Num16 && index <= m_TotalNum)
    {
        Num = (index - m_Num16) * m_interpolationCycle;

        CurJ = m_SPlanPara.jerk7;
        CurA = -m_SPlanPara.dec;
        Curf = m_SPlanPara.vel6;
        CurTm = Num;
        CurU0 = m_SPlanPara.dis6;
    }

    CurU = cal_UByPosition(CurJ, CurA, Curf, CurTm, CurU0);

    return (CurU / m_SPlanPara.length);
}

double SVelocityPlanner::cal_UForLine(double timeStamp, const SPlanPara &thePara)
{
    double CurJ = 0;
    double CurA = 0;
    double Curf = 0;
    double CurTm = 0;
    double CurU0 = 0;
    double CurU = 0;
    double Num = 0;

    double T12 = thePara.T1 + thePara.T2;
    double T13 = T12 + thePara.T3;
    double T14 = T13 + thePara.T4;
    double T15 = T14 + thePara.T5;
    double T16 = T15 + thePara.T6;

    if (timeStamp <= thePara.T1 && timeStamp > 0)
    {
        CurJ = thePara.jerk1;
        CurA = 0;
        Curf = thePara.startVel;
        CurTm = timeStamp;
        CurU0 = 0;
    }

    if (timeStamp > thePara.T1 && timeStamp <= T12)
    {
        Num = timeStamp - thePara.T1;

        CurJ = 0;
        CurA = thePara.acc;
        Curf = thePara.vel1;
        CurTm = Num;
        CurU0 = thePara.dis1;
    }

    if (timeStamp > T12 && timeStamp <= T13)
    {
        Num = timeStamp - T12;

        CurJ = -thePara.jerk3;
        CurA = thePara.acc;
        Curf = thePara.vel2;
        CurTm = Num;
        CurU0 = thePara.dis2;
    }

    if (timeStamp > T13 && timeStamp <= T14)
    {
        Num = timeStamp - T13;

        CurJ = 0;
        CurA = 0;
        Curf = thePara.vel3;
        CurTm = Num;
        CurU0 = thePara.dis3;
    }

    if (timeStamp > T14 && timeStamp <= T15)
    {
        Num = timeStamp - T14;

        CurJ = -thePara.jerk5;
        CurA = 0;
        Curf = thePara.vel4;
        CurTm = Num;
        CurU0 = thePara.dis4;
    }

    if (timeStamp > T15 && timeStamp <= T16)
    {
        Num = timeStamp - T15;

        CurJ = 0;
        CurA = -thePara.dec;
        Curf = thePara.vel5;
        CurTm = Num;
        CurU0 = thePara.dis5;
    }

    if (timeStamp > T16 && timeStamp <= thePara.totalTime)
    {
        Num = timeStamp - T16;

        CurJ = thePara.jerk7;
        CurA = -thePara.dec;
        Curf = thePara.vel6;
        CurTm = Num;
        CurU0 = thePara.dis6;
    }

    CurU = cal_UByPosition(CurJ, CurA, Curf, CurTm, CurU0);

    return (CurU / thePara.length);
}
