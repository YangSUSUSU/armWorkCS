#ifndef TEST_GAZEBO_H  
#define TEST_GAZEBO_H  
//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <map>
#include <string>
#include <iostream>
// dyn
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"



class RosInterface 
{
public:
     RosInterface();
    ~RosInterface();

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void printJointStates() const;
    void setJointTorque(const std::string& joint_name, double effort);

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    std::map<std::string, ros::Publisher> torque_publishers_;
    std::map<std::string, double> joint_positions_;
    std::map<std::string, double> joint_velocities_;
    std::map<std::string, double> joint_efforts_;
};

class DynamicsCalculator 
{
public:
    DynamicsCalculator(const std::string& urdf_filename);
    ~DynamicsCalculator();

    Eigen::VectorXd computeJointTorques(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a);
    void forwardKinematics(const Eigen::VectorXd& q);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
};


#endif // TEST_GAZEBO_H