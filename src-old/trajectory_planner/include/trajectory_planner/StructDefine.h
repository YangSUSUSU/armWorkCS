#ifndef STRUCT_DEFINE_H
#define STRUCT_DEFINE_H

#include <vector>
#include <math.h>

const int RobotDOF = 7;
const int PoseDIM  = 7;  //xyz & quaternion

//control cycle 1000Hz
const double InterpolationCycle = 0.005;

//joint acceleration parameter rad/s^2
const double Joint_Acc_Limitation = M_PI/6; //30 degree

//joint jerk parameter rad/s^3
const double Joint_Jerk_Limitation = Joint_Acc_Limitation * 3;

//cartesian acceleration parameter m/s^2
const double Cartesian_Acc_Limitation = 3;

//cartesian jerk parameter m/s^3
const double Cartesian_Jerk_Limitation = Cartesian_Acc_Limitation * 3;

//transit precision
const int percentPrecision = 20;

//the joint rotation direction
const int dir[RobotDOF] = {1, 1, 1, 1, 1, 1,1};

//interpolation type
typedef enum
{
    JOINT = 1,        //joint interpolation
    CARTESIAN = 2,    //cartesian linear interpolation
}EnumCommand;

typedef struct
{
    int dof;                              //dof of robot
    int dim;                              //dimension of configuration space (xyz xyzw)
    std::vector<double> q;                //joint space position of single waypoint
    std::vector<double> configuration;    //Cartesian space pose infomation of single joint

}SinglePoint;

//parameter for S plan
typedef struct
{
  double length;

  double startVel;
  double constVel;
  double endVel;

  double T1;
  double T2;
  double T3;
  double T4;
  double T5;
  double T6;
  double T7;

  double totalTime;

  double vel1;
  double vel2;
  double vel3;
  double vel4;
  double vel5;
  double vel6;
  double vel7;

  double dis1;
  double dis2;
  double dis3;
  double dis4;
  double dis5;
  double dis6;
  double dis7;

  double acc;
  double dec;

  double jerk1;
  double jerk3;
  double jerk5;
  double jerk7;

  double jerk;

}SPlanPara; //each segment has this

//parameter for velocity plan
typedef struct
{
  int trajSegmentIndex; //in which segment of the whole trajectory
  int interpSpaceType;  //JOINT or CARTESIAN
  int waypointNum;      //currently, hard coded as 2 in this project
  std::vector<std::vector<double>> waypoints;

  int index;                 //all fine interpolation points as Numxx
  double transformPrecision; //just 'percentPrecision'

  SPlanPara sPlanPara;  //after calculating SPlanPara, update this filed of 'VelocityPlanPara', then 'PlanningParameter'
  int isFinish;         //will be set after finishing waypoints loading for one segment

}VelocityPlanPara; //one layer before 'SPlanPara', after 'SinglePoint'

//parameter for interpolation
typedef struct
{
  int interpSpaceType;
  double transformPrecision; //for neighbering segments, select the min()
  int remainPointNum;        //original stamps num - transition Numbers
  int curToLastPointNum;     //transtion num connecting to last segment
  int curToNextPointNum;     //transtion num connecting to next segment

  std::vector<std::vector<double>> interpolationVector; //S curve results, all points
  int trajSegmentIndex;

}InterpolationPara; //after finishing 'VelocityPlanPara', get 'InterpolationPara' for one segment: GenerateInterpPoints(allVelocityPlanParas)


//parameter for planning
typedef struct
{
    std::vector<std::vector<double>> waypoints;

    double remainTime;     //all time  - transition time
    double curToLastTime;  
    double curToNextTime;

    double transitFlag; //when start to transit

    int interpSpaceType;

    SPlanPara theSingleSPara;

}PlanningParameter;           //single segment parameter

#endif //STRUCT_DEFINE_H
