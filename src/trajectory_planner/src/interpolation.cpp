#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>

#include "trajectory_planner/sVelocityPlanner.h"
#include "trajectory_planner/interpolation.h"

using namespace std;
// using namespace Eigen;

/**
 * @brief - This function generates the interpolation point list according to the specified interpolation type
 * @param[in] - allVelocityPlanParas: 
 */
vector<InterpolationPara> Interpolation::GenerateInterpPoints(std::vector<VelocityPlanPara> &allVelocityPlanParas)
{
    std::vector<InterpolationPara> allInterpolationParas;

    for (size_t i=0; i<allVelocityPlanParas.size(); i++)
    {
        InterpolationPara oneInterpolationPara;

        switch (allVelocityPlanParas[i].interpSpaceType)
        {
        case JOINT:
            JointInterpolation(allVelocityPlanParas[i], oneInterpolationPara.interpolationVector);

            oneInterpolationPara.transformPrecision = 0;
            oneInterpolationPara.interpSpaceType    = allVelocityPlanParas[i].interpSpaceType;
            oneInterpolationPara.remainPointNum     = static_cast<int>(oneInterpolationPara.interpolationVector.size());
            oneInterpolationPara.trajSegmentIndex   = static_cast<int>(i+1);
            oneInterpolationPara.curToLastPointNum  = 0;
            oneInterpolationPara.curToNextPointNum  = 0;

            allInterpolationParas.push_back(oneInterpolationPara);
            break;

        case CARTESIAN:
            Decare_LInterpolation(allVelocityPlanParas[i], oneInterpolationPara.interpolationVector);

            oneInterpolationPara.transformPrecision = allVelocityPlanParas[i].transformPrecision;
            oneInterpolationPara.interpSpaceType    = allVelocityPlanParas[i].interpSpaceType;
            oneInterpolationPara.remainPointNum     = static_cast<int>(oneInterpolationPara.interpolationVector.size());
            oneInterpolationPara.trajSegmentIndex   = static_cast<int>(i+1);
            oneInterpolationPara.curToLastPointNum  = 0;
            oneInterpolationPara.curToNextPointNum  = 0;

            allInterpolationParas.push_back(oneInterpolationPara);
            break;
        }
    }

    return allInterpolationParas;
}

/**
 * @brief - This function conducts the joint interpolation
 * @param[in] - velPlanPara:
 * @param[in] - interpPoints:
 */
int Interpolation::JointInterpolation(VelocityPlanPara &velPlanPara, std::vector<std::vector<double>> &interpPoints)
{
    double cur_U;
    double startPoint;
    double endPoint;

    SVelocityPlanner sVelPlanner;
    sVelPlanner.initJointSPara(velPlanPara);
    sVelPlanner.cal_SPara(velPlanPara);
    velPlanPara.index++;

    if(sVelPlanner.getLength() < 0.001)
    {
        velPlanPara.isFinish = 0;
        return -1;
    }

    for (int i=1; i<=sVelPlanner.getTotalNumPara(); i++)
    {
        cur_U = sVelPlanner.cal_UForLine(i);

        std::vector<double> retValue;
        double ret;
        for (size_t j=0; j<RobotDOF; j++)
        {
            startPoint = velPlanPara.waypoints[0][j];
            endPoint = velPlanPara.waypoints[1][j];

            ret = startPoint + (endPoint - startPoint)*cur_U;
            retValue.push_back(ret);
        }

        interpPoints.push_back(retValue);

        velPlanPara.index++;
    }
    return 0;
}

/**
 * @brief - This function conducts the cartesian interpolation
 */
int Interpolation::Decare_LInterpolation(VelocityPlanPara &velPlanPara, std::vector<std::vector<double>> &interpPoints)
{
    double cur_U;
    double startPoint;
    double endPoint;

    SVelocityPlanner sVelPlanner;
    sVelPlanner.initDecare_LSPara(velPlanPara);
    sVelPlanner.cal_SPara(velPlanPara);
    velPlanPara.index++;

    if(sVelPlanner.getLength() < 0.001)
    {
        velPlanPara.isFinish = 0;
        return -1;
    }

    for (int i=1; i<=sVelPlanner.getTotalNumPara(); i++)
    {
        cur_U = sVelPlanner.cal_UForLine(i);

        std::vector<double> retValue;
        double ret;
        Eigen::VectorXd quaternionS(4);
        Eigen::VectorXd quaternionE(4);
        Eigen::VectorXd quaternionRet;

        for (size_t j=0; j<3; j++)
        {
            startPoint = velPlanPara.waypoints[0][j];
            endPoint = velPlanPara.waypoints[1][j];

            ret = startPoint + (endPoint - startPoint)*cur_U;

            retValue.push_back(ret);
        }

        for (size_t j=0; j<4; j++)
        {
            quaternionS(static_cast<int>(j)) = velPlanPara.waypoints[0][j+3];
            quaternionE(static_cast<int>(j)) = velPlanPara.waypoints[1][j+3];
        }
        // Spherical Linear Interpolation，即球面线性插值
        quaternionRet = slerp(quaternionS, quaternionE, cur_U);

        for (int k=0; k<4; k++)
        {
            retValue.push_back(quaternionRet(k));
        }

        interpPoints.push_back(retValue);
        velPlanPara.index++;
    }

    return 0;
}

/**
 * @brief - This function calculates the u parameter according to the input time for joint interpolation
 */
std::vector<double> Interpolation::CalJointByTime(double time, PlanningParameter thePlanPara)
{
    std::vector<double> retValue;
    double cur_U;
    double startPoint;
    double endPoint;
    double ret;

    SVelocityPlanner theSVelocityPlanner;
    cur_U = theSVelocityPlanner.cal_UForLine(time, thePlanPara.theSingleSPara);

    for (size_t j=0; j<RobotDOF; j++)
    {
        startPoint = thePlanPara.waypoints[0][j];
        endPoint = thePlanPara.waypoints[1][j];

        ret = startPoint + (endPoint - startPoint)*cur_U;
        retValue.push_back(ret);
    }

    return retValue;
}


std::vector<double> Interpolation::CalDecareByTime(double time, PlanningParameter thePlanParaFirst, PlanningParameter thePlanParaSecond)
{
    std::vector<double> retValue;

    SVelocityPlanner theSVelocityPlanner;


    if(time >= thePlanParaFirst.transitFlag && time < thePlanParaFirst.theSingleSPara.totalTime)
    {
        double cur_UM1;
        double cur_UM2;
        std::vector<double> base;
        std::vector<double> pointM1;
        std::vector<double> pointM2;

        double startPointM1, startPointM2;
        double endPointM1, endPointM2;
        double retM1, retM2;

        double timeM1 = time - thePlanParaFirst.transitFlag;

        cur_UM1 = theSVelocityPlanner.cal_UForLine(time, thePlanParaFirst.theSingleSPara);
        cur_UM2 = theSVelocityPlanner.cal_UForLine(timeM1, thePlanParaSecond.theSingleSPara);

        for (size_t i=0; i<PoseDIM; i++)
        {
            base.push_back(thePlanParaSecond.waypoints[0][i]);
        }
        Eigen::Quaterniond quaternionBase;

        quaternionBase.x() = base[3];
        quaternionBase.y() = base[4];
        quaternionBase.z() = base[5];
        quaternionBase.w() = base[6];
        Eigen::Matrix3d matrixBase = quaternion2rotation(quaternionBase);

        //position interpolation
        for (size_t i=0; i<3; i++)
        {
            startPointM1 =  thePlanParaFirst.waypoints[0][i];
            endPointM1 = thePlanParaFirst.waypoints[1][i];
            retM1 = startPointM1 + (endPointM1 - startPointM1)*cur_UM1;
            pointM1.push_back(retM1);

            startPointM2 = thePlanParaSecond.waypoints[0][i];
            endPointM2 = thePlanParaSecond.waypoints[1][i];
            retM2 = startPointM2 + (endPointM2 - startPointM2)*cur_UM2;
            pointM2.push_back(retM2);
        }

        //orientation interpolation
        Eigen::VectorXd quaternionSM1(4), quaternionSM2(4);
        Eigen::VectorXd quaternionEM1(4), quaternionEM2(4);
        Eigen::VectorXd quaternionRetM1, quaternionRetM2;

        for (size_t j=0; j<4; j++)
        {
            quaternionSM1(static_cast<int>(j)) = thePlanParaFirst.waypoints[0][j+3];
            quaternionEM1(static_cast<int>(j)) = thePlanParaFirst.waypoints[1][j+3];

            quaternionSM2(static_cast<int>(j)) = thePlanParaSecond.waypoints[0][j+3];
            quaternionEM2(static_cast<int>(j)) = thePlanParaSecond.waypoints[1][j+3];
        }
        quaternionRetM1 = slerp(quaternionSM1, quaternionEM1, cur_UM1);
        quaternionRetM2 = slerp(quaternionSM2, quaternionEM2, cur_UM2);

        for (int i=0; i<4; i++)
        {
            pointM1.push_back(quaternionRetM1(i));
            pointM2.push_back(quaternionRetM2(i));
        }

        Eigen::Quaterniond quaternionP1;
        Eigen::Quaterniond quaternionP2;
        double tempVal;

        for (size_t k=0; k<3; k++)
        {
            tempVal = pointM1[k] + pointM2[k] - base[k];
            retValue.push_back(tempVal);
        }

        quaternionP1.x() = pointM1[3];
        quaternionP1.y() = pointM1[4];
        quaternionP1.z() = pointM1[5];
        quaternionP1.w() = pointM1[6];
        Eigen::Matrix3d matrixP1 = quaternion2rotation(quaternionP1);

        quaternionP2.x() = pointM2[3];
        quaternionP2.y() = pointM2[4];
        quaternionP2.z() = pointM2[5];
        quaternionP2.w() = pointM2[6];
        Eigen::Matrix3d matrixP2 = quaternion2rotation(quaternionP2);

        Eigen::Matrix3d tempMatrix = (matrixBase.inverse()) * matrixP2;

        Eigen::Matrix3d resultMatrix = matrixP1 * tempMatrix;

        Eigen::Quaterniond resultQuaternion = rotation2quaternion(resultMatrix);

        retValue.push_back(resultQuaternion.x());
        retValue.push_back(resultQuaternion.y());
        retValue.push_back(resultQuaternion.z());
        retValue.push_back(resultQuaternion.w());
    }


    if(time >= thePlanParaFirst.curToLastTime && time < thePlanParaFirst.transitFlag)
    {
        double cur_U;
        double startPoint, endPoint;
        double ret;

        cur_U = theSVelocityPlanner.cal_UForLine(time, thePlanParaFirst.theSingleSPara);

        //position interpolation
        for (size_t i=0; i<3; i++)
        {
            startPoint =  thePlanParaFirst.waypoints[0][i];
            endPoint = thePlanParaFirst.waypoints[1][i];
            ret = startPoint + (endPoint - startPoint)*cur_U;
            retValue.push_back(ret);
        }

        //orientation interpolation
        Eigen::VectorXd quaternionS(4);
        Eigen::VectorXd quaternionE(4);
        Eigen::VectorXd quaternionRet;

        for (size_t j=0; j<4; j++)
        {
            quaternionS(static_cast<int>(j)) = thePlanParaFirst.waypoints[0][j+3];
            quaternionE(static_cast<int>(j)) = thePlanParaFirst.waypoints[1][j+3];
        }

        quaternionRet = slerp(quaternionS, quaternionE, cur_U);

        for (int i=0; i<4; i++)
        {
            retValue.push_back(quaternionRet(i));
        }
    }

    return retValue;
}

std::vector<double> Interpolation::CalDecareByTime(double time, PlanningParameter thePlanParaFirst)
{
    std::vector<double> retValue;
    double cur_U;
    double startPoint, endPoint;
    double ret;

    SVelocityPlanner theSVelocityPlanner;

    cur_U = theSVelocityPlanner.cal_UForLine(time, thePlanParaFirst.theSingleSPara);

    //position interpolation
    for (size_t i=0; i<3; i++)
    {
        startPoint =  thePlanParaFirst.waypoints[0][i];
        endPoint = thePlanParaFirst.waypoints[1][i];
        ret = startPoint + (endPoint - startPoint)*cur_U;
        retValue.push_back(ret);
    }

    //orientation interpolation
    Eigen::VectorXd quaternionS(4);
    Eigen::VectorXd quaternionE(4);
    Eigen::VectorXd quaternionRet;

    for (size_t j=0; j<4; j++)
    {
        quaternionS(static_cast<int>(j)) = thePlanParaFirst.waypoints[0][j+3];
        quaternionE(static_cast<int>(j)) = thePlanParaFirst.waypoints[1][j+3];
    }

    quaternionRet = slerp(quaternionS, quaternionE, cur_U);

    for (int i=0; i<4; i++)
    {
        retValue.push_back(quaternionRet(i));
    }

    return retValue;
}


void Interpolation::CalculateTransitPointNum(std::vector<InterpolationPara> &interPolationPara)
{
    for (size_t i=0; i<interPolationPara.size()-1; i++)
    {
        bool firstFlag = (interPolationPara[i].interpSpaceType == CARTESIAN);

        bool secondFlag = (interPolationPara[i+1].interpSpaceType == CARTESIAN);

        if(firstFlag && secondFlag)
        {
            int firstNo = static_cast<int>(interPolationPara[i].interpolationVector.size());
            int secondNo = static_cast<int>(interPolationPara[i+1].interpolationVector.size());

            if(interPolationPara[i].transformPrecision > 50)
            {
                interPolationPara[i].transformPrecision = 50;
            }

            int transitNo = static_cast<int>(firstNo*interPolationPara[i].transformPrecision/100.0);

            if(transitNo > interPolationPara[i].remainPointNum)
            {
                transitNo = interPolationPara[i].remainPointNum;
            }

            if(transitNo > secondNo)
            {
                transitNo = secondNo;
            }

            interPolationPara[i+1].remainPointNum = secondNo - transitNo;

            interPolationPara[i].curToNextPointNum = transitNo;
            interPolationPara[i+1].curToLastPointNum = transitNo;
        }
    }
}


Eigen::VectorXd Interpolation::quaternion2rpy(Eigen::Quaterniond quaternion)
{
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    // Eigen::VectorXd rpy = rotationMatrix.eulerAngles(0,1,2);
    Eigen::VectorXd rpy = transR2RPY(rotationMatrix,"xyz");

    return rpy;
}


Eigen::VectorXd Interpolation::transR2RPY(Eigen::Matrix3d R,std::string xyz)
{
    Eigen::Vector3d rpy;
    if(xyz == "xyz"){
        if( fabs(R(2,2)) < 1e-12 && fabs(R(1,2)) < 1e-12 ){
            //% singularity
            rpy(0) = 0;  //% roll is zero
            rpy(1) = atan2(R(0,2), R(2,2));  //% pitch
            rpy(2) = atan2(R(1,0), R(1,1));  //% yaw is sum of roll+yaw
        }
        else{
            rpy(0) = atan2(-R(1,2), R(2,2));        //% roll
            //% compute sin/cos of roll angle
            double sr = sin(rpy(0));
            double cr = cos(rpy(0));
            rpy(1) = atan2(R(0,2), cr * R(2,2) - sr * R(1,2));  //% pitch
            rpy(2) = atan2(-R(0,1), R(0,0));        //% yaw
        }
    }
    else if(xyz == "zyx"){
        //            % old ZYX order (as per Paul book)
        if( fabs(R(0,0)) < 1e-12 && fabs(R(1,0)) < 1e-12){
            //% singularity
            rpy(0) = 0;     // roll is zero
            rpy(1) = atan2(-R(2,0), R(0,0));  // pitch
            rpy(2) = atan2(-R(1,2), R(1,1));  // yaw is difference yaw-roll
        }
        else{
            rpy(0) = atan2(R(1,0), R(0,0));
            double sp = sin(rpy(0));
            double cp = cos(rpy(0));
            rpy(1) = atan2(-R(2,0), cp * R(0,0) + sp * R(1,0));
            rpy(2) = atan2(sp * R(0,2) - cp * R(1,2), cp*R(1,1) - sp*R(0,1));
        }
    }
    return rpy;
}


Eigen::Matrix3d Interpolation::quaternion2rotation(Eigen::Quaterniond quaternion)
{
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    return rotationMatrix;
}


Eigen::Quaterniond Interpolation::rotation2quaternion(Eigen::Matrix3d rotationMatrix)
{
    Eigen::Quaterniond quaternion(rotationMatrix);

    return quaternion;
}

/**
 * @brief - This function calculates the spherical linear interpolation
 * refer to: https://en.wikipedia.org/wiki/Slerp
 */
Eigen::VectorXd Interpolation::slerp(Eigen::VectorXd startPos, Eigen::VectorXd endPos, double u)
{
    Eigen::VectorXd result(4);

    double cosa = startPos(0)*endPos(0) + startPos(1)*endPos(1) + startPos(2)*endPos(2) + startPos(3)*endPos(3);

    // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
    // the shorter path. Fix by reversing one quaternion.
    if ( cosa < 0.0 )
    {
        endPos(0) = -endPos(0);
        endPos(1) = -endPos(1);
        endPos(2) = -endPos(2);
        endPos(3) = -endPos(3);
        cosa = -cosa;
    }

    double k0, k1;

    if ( cosa > 0.9999 )
    {
        k0 = 1.0 - u;
        k1 = u;
    }
    else
    {
        double sina = sqrt( 1.0 - cosa*cosa );
        double a = atan2( sina, cosa );
        k0 = sin((1.0 - u)*a) / sina;
        k1 = sin(u*a) / sina;
    }

    result(0) = startPos(0)*k0 + endPos(0)*k1;
    result(1) = startPos(1)*k0 + endPos(1)*k1;
    result(2) = startPos(2)*k0 + endPos(2)*k1;
    result(3) = startPos(3)*k0 + endPos(3)*k1;

    return result;
}

