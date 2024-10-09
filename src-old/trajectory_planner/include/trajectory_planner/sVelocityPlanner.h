#ifndef S_VELOCITY_PLANNING_H
#define S_VELOCITY_PLANNING_H

#include "StructDefine.h"

/**
 * Class Name: SVelocityPlanner
 * Description: Used for S velocity plan
 * Reference: https://blog.csdn.net/u010632165/article/details/104951091
*/
class SVelocityPlanner
{
public:
    //initialize the acceleration and deceleration
    void initAcceDece();

    //initialize the velocity plan parameter for joint S planning
    void initJointSPara(VelocityPlanPara &velPara);

    //initialize the velocity plan parameter for cartesian S planning
    void initDecare_LSPara(VelocityPlanPara &velPara);

    //check the jerk condition
    void checkJerkCondition();

    //check the acceleration condition
    void checkAccCondition();

    //check the deceleration condition
    void checkDecCondition();

    //check the total path length condition
    int checkTotalLenCondition();

    //update the acceleration and deceleration according to the calculate duration
    void updateADcc();

    //update the each segment velocity according to the calculate duration
    void updateVelocity();

    //calculate the u parameter for interpolation
    double cal_UByPosition(double J, double A, double f, double Tm, double u0);

    //calculate the all parameters for S plan
    int cal_SPara(VelocityPlanPara &velPlanPara);

    //return the total time stamps number
    int getTotalNumPara();

    //return the path length
    double getLength();

    //calculate the parameter for u, override +2
    double cal_UForLine(int index);
    double cal_UForLine(double timeStamp, const SPlanPara &thePara);

    //constructor override +2
    SVelocityPlanner();
    SVelocityPlanner(const SVelocityPlanner &cp);

    //default destructor
    ~SVelocityPlanner();

private:
    double m_interpolationCycle;

    int m_Num1;
    int m_Num2;
    int m_Num3;
    int m_Num4;
    int m_Num5;
    int m_Num6;
    int m_Num7;

    int m_Num12;
    int m_Num13;
    int m_Num14;
    int m_Num15;
    int m_Num16;

    int m_TotalNum;

    SPlanPara m_SPlanPara;

};

#endif //S_VELOCITY_PLANNING_H
