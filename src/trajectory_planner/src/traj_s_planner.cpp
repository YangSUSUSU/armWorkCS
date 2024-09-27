#include "trajectory_planner/traj_s_planner.h"

using namespace std;
using namespace Eigen;

//#define AnaliticalIK
#define QPsloveIK

Traj_S_Planner::Traj_S_Planner(uint dof, double rate):TrajPlannerBase(dof, rate)
{
    ROS_INFO("Traj_S_Planner constructor called!");
    isCartesian = false;
    veclocity = 0.0;
    plan_method = S_Veclocity;
    cal_duration = 0.0;
}

Traj_S_Planner::~Traj_S_Planner()
{
    ROS_INFO("Traj_S_Planner destructor called!");
}


void Traj_S_Planner::transformPoints(vector<vector<double>> &allPathPoints, vector<SinglePoint> &trajAllPoints, string interpSpaceTypeName)
{
    trajAllPoints.clear();
    if (interpSpaceTypeName == "Cartesian")
    {
        for (uint i=0; i<allPathPoints.size(); ++i)
        {
            SinglePoint onePoint;
            onePoint.dim = PoseDIM;
            onePoint.configuration = allPathPoints[i];

            trajAllPoints.push_back(onePoint);
        }

        isCartesian = true;
    }
    else
    {
        for (uint i=0; i<allPathPoints.size(); ++i)
        {
            SinglePoint onePoint;
            onePoint.dof = RobotDOF;
            onePoint.q = allPathPoints[i];

            trajAllPoints.push_back(onePoint);
        }

        isCartesian = false;
    }
}

vector<VelocityPlanPara> Traj_S_Planner::parseCommand(vector<SinglePoint> &trajAllPoints, double trajVelocity, string interpSpaceTypeName)
{
    vector<VelocityPlanPara> allVelocityPlanParas;
    vector<vector<double>> trajWayPoints;

    for (size_t i=0; i<trajAllPoints.size(); i++)
    {
        if(interpSpaceTypeName == "Joint")
        {
            trajWayPoints.push_back(trajAllPoints[i].q);
        }
        else if (interpSpaceTypeName == "Cartesian")
        {
            trajWayPoints.push_back(trajAllPoints[i].configuration);
        }
    }

    int interpSpaceType = 0;
    int waypointsNumOneSegment = 2; //have more points in other applications, but not in this package
    int trajSegmentsNum = static_cast<int>(trajWayPoints.size() - 1);

    if(interpSpaceTypeName=="Joint")
    {
        interpSpaceType = JOINT;
    }
    else if (interpSpaceTypeName=="Cartesian")
    {
        interpSpaceType = CARTESIAN;
    }

    for (int i=0; i<trajSegmentsNum; i++)
    {
        VelocityPlanPara oneVelocityPlanPara;

        oneVelocityPlanPara.trajSegmentIndex = i+1;
        oneVelocityPlanPara.interpSpaceType  = interpSpaceType;

        oneVelocityPlanPara.sPlanPara.startVel = 0.0;                    //for one segment, start with velocity=0
        oneVelocityPlanPara.sPlanPara.constVel = trajVelocity;
        oneVelocityPlanPara.sPlanPara.endVel   = 0.0;                    //for one segment, end with velocity=0
        oneVelocityPlanPara.transformPrecision = percentPrecision;
        oneVelocityPlanPara.waypointNum        = waypointsNumOneSegment; //actually, this is hard coded to 2 in above section

        if(oneVelocityPlanPara.interpSpaceType==JOINT)
        {
            oneVelocityPlanPara.sPlanPara.acc  = Joint_Acc_Limitation;
            oneVelocityPlanPara.sPlanPara.dec  = Joint_Acc_Limitation;
            oneVelocityPlanPara.sPlanPara.jerk = Joint_Jerk_Limitation;
        }
        else
        {
            oneVelocityPlanPara.sPlanPara.acc  = Cartesian_Acc_Limitation;
            oneVelocityPlanPara.sPlanPara.dec  = Cartesian_Acc_Limitation;
            oneVelocityPlanPara.sPlanPara.jerk = Cartesian_Jerk_Limitation;
        }

        for (int k=0; k<waypointsNumOneSegment; k++)
        {
            oneVelocityPlanPara.waypoints.push_back( trajWayPoints[i+k] );
        }

        oneVelocityPlanPara.isFinish = 1;
        oneVelocityPlanPara.index = 0;

        allVelocityPlanParas.push_back(oneVelocityPlanPara);
    }

    return allVelocityPlanParas;
}

double Traj_S_Planner::initInterpolation(vector<vector<double>> &allPathPoints, double trajVelocity, string interpSpaceTypeName)
{
    double duration = 0.0;
    m_allPlanningParameters.clear();
    cal_duration = 0.0;
    m_timeList.clear();

    vector<SinglePoint> trajAllPoints;
    transformPoints(allPathPoints, trajAllPoints, interpSpaceTypeName);

    std::vector<VelocityPlanPara> allVelocityPlanParas = parseCommand(trajAllPoints, trajVelocity, interpSpaceTypeName);

    //save the waypoint infomation
    for (size_t i=0; i<allVelocityPlanParas.size(); i++)
    {
        VelocityPlanPara oneVelocityPlanPara = allVelocityPlanParas[i];
        PlanningParameter onePlanningParameter;

        onePlanningParameter.waypoints = oneVelocityPlanPara.waypoints;
        m_allPlanningParameters.push_back(onePlanningParameter);
    }

    vector<InterpolationPara> allInterpolationParas = interpolation_Obj.GenerateInterpPoints(allVelocityPlanParas);

    //save the S interpolation parameter
    for (size_t i=0; i<allVelocityPlanParas.size(); i++)
    {
        m_allPlanningParameters[i].theSingleSPara = allVelocityPlanParas[i].sPlanPara;
    }

    interpolation_Obj.CalculateTransitPointNum(allInterpolationParas);

    for (size_t i=0; i<allInterpolationParas.size(); i++)
    {
        if(allInterpolationParas[i].interpSpaceType == CARTESIAN)
        {
            m_allPlanningParameters[i].curToLastTime = (allInterpolationParas[i].curToLastPointNum) * InterpolationCycle;
            m_allPlanningParameters[i].curToNextTime = (allInterpolationParas[i].curToNextPointNum) * InterpolationCycle;
        }
        else if (allInterpolationParas[i].interpSpaceType == JOINT)
        {
            m_allPlanningParameters[i].curToLastTime = 0;
            m_allPlanningParameters[i].curToNextTime = 0;
        }

        m_allPlanningParameters[i].remainTime = (allInterpolationParas[i].remainPointNum - allInterpolationParas[i].curToNextPointNum) * InterpolationCycle;
        m_allPlanningParameters[i].transitFlag = m_allPlanningParameters[i].curToLastTime + m_allPlanningParameters[i].remainTime;

        m_allPlanningParameters[i].interpSpaceType = allInterpolationParas[i].interpSpaceType;

        double tempTime = duration + m_allPlanningParameters[i].remainTime + m_allPlanningParameters[i].curToNextTime;

        duration = tempTime;
        m_timeList.push_back(tempTime);
    }

    cal_duration = duration;

    return duration;
}


bool Traj_S_Planner::calCartesianInterpolation(double time, const Matrix4d &last_T, Matrix4d &T)
{
    double calTime = 0.0;

    if(time < 0 || time > cal_duration)
    {
        T = last_T;
        ROS_WARN("error time [%5.3f]", time);
        return true;
    }

    size_t local = 0;
    if(time <= m_timeList[0])
    {
        local = 0;
        calTime = time + m_allPlanningParameters[local].curToLastTime;
    }
    else
    {
        for (size_t i=1; i<m_timeList.size(); i++)
        {
            if(time > m_timeList[i-1] && time <= m_timeList[i])
            {
                local = i;
                calTime = time - m_timeList[i-1] + m_allPlanningParameters[i].curToLastTime;
                break;
            }
        }
    }

    vector<double> CartesianPosition;
    int flag = 1;
    if(local < m_allPlanningParameters.size()-1)
    {
        flag =2;
        cout<<"calTime"<<calTime<<endl;
        // cout<<"The CartesianPosition = "<<endl;
        // copy(m_allPlanningParameters[local].cbegin(),m_allPlanningParameters[local].cend(),ostream_iterator<PlanningParameter>(cout," "));
        // cout<<endl;
        // cout<<"calTime"<<calTime<<endl;

        CartesianPosition = interpolation_Obj.CalDecareByTime(calTime, m_allPlanningParameters[local], m_allPlanningParameters[local+1]);
        cout<<"The CartesianPosition = "<<endl;
        copy(CartesianPosition.cbegin(),CartesianPosition.cend(),ostream_iterator<double>(cout," "));
        cout<<endl;
    }

    if(local == m_allPlanningParameters.size()-1) //locate in last segment
    {
        flag =3;
        CartesianPosition = interpolation_Obj.CalDecareByTime(calTime, m_allPlanningParameters[local]);
    }
    ROS_INFO("Flag = %d",flag);
    T = Quat2T_matrix(CartesianPosition);

    return true;
}
