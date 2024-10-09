#include "trajectory_planner/traj_planner_base.h"

using namespace std;
using namespace Eigen;

TrajPlannerBase::TrajPlannerBase(uint dof, double rate):DOF(dof), control_frequency(rate)
{
    ROS_INFO("TrajPlannerBase constructor called!");
}

TrajPlannerBase::~TrajPlannerBase()
{
    ROS_INFO("TrajPlannerBase destructor called!");
}

vector<double> TrajPlannerBase::T_matrix2Quat(Matrix4d T_matrix)
{
    vector<double> waypoint(7,0);

    Matrix3d rotMatrix = T_matrix.block<3,3>(0,0);
    Quaterniond quat(rotMatrix);

    waypoint[0] = T_matrix(0,3);
    waypoint[1] = T_matrix(1,3);
    waypoint[2] = T_matrix(2,3);
    waypoint[3] = quat.x();
    waypoint[4] = quat.y();
    waypoint[5] = quat.z();
    waypoint[6] = quat.w();

    return waypoint;
}

Matrix4d TrajPlannerBase::Quat2T_matrix(vector<double> waypoint)
{
    Vector3d posValue;
    Quaterniond quat;
    if (waypoint.size() != 7) {
        // 错误处理
        cout<<"The CartesianPosition = "<<endl;
        copy(waypoint.cbegin(),waypoint.cend(),ostream_iterator<double>(cout," "));
        cout<<endl;
        throw std::invalid_argument("Quaternion size must be 7");
    }
    for (int i=0; i<3; i++)
    {
        posValue(i) = waypoint[i];
    }

    quat.x() = waypoint[3];
    quat.y() = waypoint[4];
    quat.z() = waypoint[5];
    quat.w() = waypoint[6];

    Matrix4d T = MatrixXd::Identity(4,4);
    T.block<3,3>(0,0) = quat.toRotationMatrix();
    T.block<3,1>(0,3) = posValue;

    return T;
}

Quaterniond TrajPlannerBase::Euler2Quat(VectorXd rpy)
{
    Matrix3d rotationMatrix = Euler2Rotation(rpy);
    Quaterniond quaternion(rotationMatrix);

    return quaternion;
}

VectorXd TrajPlannerBase::Quat2Euler(Quaterniond quaternion)
{
    Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    VectorXd rpy = rotationMatrix.eulerAngles(0,1,2);

    return rpy;
}

Matrix3d TrajPlannerBase::Euler2Rotation(VectorXd rpy)
{
    AngleAxisd roll(AngleAxisd(rpy[0],Vector3d::UnitX()));
    AngleAxisd pitch(AngleAxisd(rpy[1],Vector3d::UnitY()));
    AngleAxisd yaw(AngleAxisd(rpy[2],Vector3d::UnitZ()));

    Matrix3d rotationMatrix = roll.matrix() * pitch.matrix() * yaw.matrix();

    return rotationMatrix;
}

VectorXd TrajPlannerBase::Rotation2Euler(Matrix3d rotationMatrix)
{
    VectorXd rpy = rotationMatrix.eulerAngles(0,1,2);
    return rpy;
}
