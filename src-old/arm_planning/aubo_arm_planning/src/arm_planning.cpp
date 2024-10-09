/**
 * @file main.cpp
 * @brief  Aubo trajectory_planning
 * @author He Yan
 * @date 2024-06-18
 * @version alpha
 */

#include "aubo_arm_planning/arm_planning.h"

#include <string>
#include <time.h>
#include <sys/time.h>

using namespace Eigen;
using namespace std;

#define simulation
// #define real
#define LLM
#define unveiling_ceremony
// #define hug_emblem
// #define left_hello
#define right_hello

int hand_move = 0;
int handReset = 0;
uint32_t left_grasp_seq,right_grasp_seq;
vector<double> left_grasp_pose(7);
vector<double> right_grasp_pose(7);
bool first_time = true;
Eigen::VectorXd joint_test_last(7);

DualArm::DualArm(ros::NodeHandle& nh)
{
    left_arm_param_.n_joints=7;
    right_arm_param_.n_joints=7;
    left_arm_param_.resize();
    right_arm_param_.resize();
    frequency = 200;
    
    nh.getParam("left_chain_start", left_arm_param_.base_link);
    nh.getParam("left_chain_end", left_arm_param_.eef_link);
    nh.getParam("right_chain_start", right_arm_param_.base_link);
    nh.getParam("right_chain_end", right_arm_param_.eef_link);

    left_arm_param_.name = "left";
    left_arm_param_.which_arm = 2;

    left_arm_param_.joints_min ={-3.1415926,0        ,-2.268928,-1.779,-2.268928,-0.6454,-0.6454};
    left_arm_param_.joints_max ={0.7853981 ,1.7453292,2.2689280,0        , 2.268928, 0.6454, 0.6454};

    right_arm_param_.name = "right";
    right_arm_param_.which_arm = 2;

    right_arm_param_.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
    right_arm_param_.joints_max ={3.1415926  ,0         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};

    left_arm_param_.joint_names = {"1", "2", "3", "4",
                                   "5", "6", "7"};
    right_arm_param_.joint_names = {"1", "2", "3", "4",
                                    "5", "6", "7"};
    
    grasp_pose_subscriber   = nh.subscribe("/hand_pose_req",10,&DualArm::GraspPose_CallBack,this);

    left_joints_subscriber  = nh.subscribe("/human_arm_state_left",10,&DualArm::LeftJointStatesCallback,this);
    right_joints_subscriber = nh.subscribe("/human_arm_state_right",10,&DualArm::RightJointStatesCallback,this);
    
    left_joints_publisher  = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_left", 10);
    right_joints_publisher = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_right", 10);
    arrive_pose_publisher  = nh.advertise<llm_msgs::pose_action_status>("/pose_action_status", 10);
    joint_state_pub        = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    test_trajectory_pub_ = nh.advertise<geometry_msgs::Point>("cartesian_trajectory", 10); // 初始化话题发布者

    // int thread_num = 4;
    // spinner_ = std::make_unique<ros::AsyncSpinner>(thread_num);
    // spinner_->start();
}
// DualArm::~DualArm()
// {
//     ROS_INFO("Stop the spinner and clean up");
//     spinner_->stop();
// }
bool DualArm::Judge_arrived(vector<double> cur_pose,vector<double> end_grasp_pose)
{
    double dx = cur_pose[0] - end_grasp_pose[0];
    double dy = cur_pose[1] - end_grasp_pose[1];
    double dz = cur_pose[2] - end_grasp_pose[2];
    double dxx = cur_pose[3] - end_grasp_pose[3];
    double dxy = cur_pose[4] - end_grasp_pose[4];
    double dxz = cur_pose[5] - end_grasp_pose[5];
    double dxw = cur_pose[6] - end_grasp_pose[6];

    double dis = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,2)+pow(dxx,2) + pow(dxy,2) + pow(dxz,2)+ pow(dxw,2));
    cout << "dis = " << dis <<endl;
    if(dis > 0.02) //position error
        return false;

    // Matrix4d T_cur = TrajPlannerBase::Quat2T_matrix(cur_pose);
    // Matrix4d T_grasp = TrajPlannerBase::Quat2T_matrix(end_grasp_pose);

    // Matrix3d pose_err_rotation = (T_cur.block<3,3>(0,0)).inverse()*(T_grasp.block<3,3>(0,0));
    // VectorXd pose_err_rpy = TrajPlannerBase::Rotation2Euler(pose_err_rotation);

    // for (uint i=0; i<3; ++i) {
    //     if(abs(pose_err_rpy(i))>0.01)
    //     {
    //         cout<< "abs(pose_err_rpy(i)) = " << abs(pose_err_rpy(i)) <<endl;
    //         return false;
    //     }
    // }
    return true;
}

void DualArm::GraspPose_CallBack(const llm_msgs::hand_pose_req::ConstPtr& grasp_pos)
{
    // ROS_INFO("grasp_pos->hand_move_enable =%d",grasp_pos->hand_move_enable);

    if (grasp_pos->hand_move_enable==1)
    {
        handReset = grasp_pos -> hand_reset;
        struct timeval timeNow;
        gettimeofday(&timeNow, nullptr);

        // // 转换为tm结构体
        // struct tm* timeinfo;
        // timeinfo = localtime(&timeNow.tv_sec);
        // 打印时分
        // std::cout << "Current time: " << std::to_string(double(timeNow.tv_sec + (timeNow.tv_usec / 1e6))) << std::endl;
	    // std::cout << "Header time:" << std::to_string(double(grasp_pos->header.stamp.toSec())) << std::endl;
        // ROS_INFO("grasp_pos->hand_move_enable  = %d" ,grasp_pos->hand_move_enable);
        if (grasp_pos->hand_side == 0)
        {
            left_grasp_pose[0] = grasp_pos->pose_req.position.x;
            left_grasp_pose[1] = grasp_pos->pose_req.position.y;
            left_grasp_pose[2] = grasp_pos->pose_req.position.z;
            left_grasp_pose[3] = grasp_pos->pose_req.orientation.x;
            left_grasp_pose[4] = grasp_pos->pose_req.orientation.y;
            left_grasp_pose[5] = grasp_pos->pose_req.orientation.z;
            left_grasp_pose[6] = grasp_pos->pose_req.orientation.w;
            hand_move = grasp_pos->hand_move_enable;
            left_grasp_seq = grasp_pos->header.seq;
            left_arm_param_.which_arm = grasp_pos->hand_side ;
            ROS_INFO("left_grasp_seq :%d",left_grasp_seq);
            ROS_INFO("Receive Left arm grasp pose = %f %f %f %f %f %f %f" , left_grasp_pose[0],left_grasp_pose[1],left_grasp_pose[2],left_grasp_pose[3],left_grasp_pose[4],left_grasp_pose[5],left_grasp_pose[6]);
            Cartesian_Plan(left_grasp_pose,left_arm_param_,left_grasp_seq);

        }
        if (grasp_pos->hand_side == 1)
        {
            right_grasp_pose[0] = grasp_pos->pose_req.position.x;
            right_grasp_pose[1] = grasp_pos->pose_req.position.y;
            right_grasp_pose[2] = grasp_pos->pose_req.position.z;
            right_grasp_pose[3] = grasp_pos->pose_req.orientation.x;
            right_grasp_pose[4] = grasp_pos->pose_req.orientation.y;
            right_grasp_pose[5] = grasp_pos->pose_req.orientation.z;
            right_grasp_pose[6] = grasp_pos->pose_req.orientation.w;
            hand_move = grasp_pos->hand_move_enable;
            right_grasp_seq = grasp_pos->header.seq;
    
            right_arm_param_.which_arm = grasp_pos->hand_side;
            ROS_INFO("right_grasp_seq :%d",right_grasp_seq);
            ROS_INFO("Receive Right arm grasp pose = %f %f %f %f %f %f %f" , right_grasp_pose[0],right_grasp_pose[1],right_grasp_pose[2],right_grasp_pose[3],right_grasp_pose[4],right_grasp_pose[5],right_grasp_pose[6]);
            
            Cartesian_Plan(right_grasp_pose,right_arm_param_,right_grasp_seq);

        }
    }
     
}

void DualArm::LeftJoints_pub(const vector<double> pos, const VectorXd vel, ros::Publisher pub) {

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(7); 
    joint_state.position.resize(7);
    joint_state.velocity.resize(7);
    joint_state.effort.resize(7);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"1","2","3","4","5","6","7"};
    for (int i = 0; i < 7; i++) {
        joint_state.position[i] = pos[i];
        // joint_state.velocity[i] = vel(i);
    }
    // while (pub.getNumSubscribers()<1)
    // {
    //     sleep(0.1);
    // }
    
    pub.publish(joint_state);
}

void DualArm::RightJoints_pub(const vector<double> pos, const VectorXd vel, ros::Publisher pub) {

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(7); 
    joint_state.position.resize(7);
    joint_state.velocity.resize(7);
    joint_state.effort.resize(7);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = {"1","2","3","4","5","6","7"};
    for (int i = 0; i < 7; i++) {
        joint_state.position[i] = pos[i];
        // joint_state.velocity[i] = vel(i);
    }
    // while (pub.getNumSubscribers()<1)
    // {
    //     sleep(0.1);
    // }
    pub.publish(joint_state);
}

void DualArm::LeftJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i = 0; i < 7; i++)
    {
        left_arm_param_.joint_pos[i] = msg->position[i];
    }
}
        
void DualArm::RightJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{   

    for (int i = 0; i < 7; i++)
    {
        right_arm_param_.joint_pos[i] = msg->position[i];
        
    }
    
    double abs_vel = 0.;
    for (size_t i = 0; i < 7; i ++ )
    {
        abs_vel += msg->velocity[i] * msg->velocity[i];
    }

    static bool end_time_obtained = false;
    if ((abs_vel > 0.00001) && (!end_time_obtained))
    {
        end_time_obtained = true;
        struct timeval cur_time;
        gettimeofday(&cur_time, NULL);
        // std::cout << "end_time is " << std::to_string(double(cur_time.tv_sec + cur_time.tv_usec / 1e6)) << std::endl;
    }

}

// Function to calculate minimum jerk trajectory in Cartesian space
std::vector<Eigen::Matrix4d> DualArm::minimumJerkInterpolationCartesian(const Eigen::Matrix4d& start, const Eigen::Matrix4d& end, double duration, int num_samples)
{
    std::vector<Eigen::Matrix4d> trajectory;
    trajectory.reserve(num_samples);

    for (int i = 0; i <= num_samples; ++i) {
        double t = i * duration / num_samples;
        double tau = t / duration;

        // Minimum jerk polynomial coefficients
        double h1 = 10 * pow(tau, 3) - 15 * pow(tau, 4) + 6 * pow(tau, 5);
        double h2 = -15 * pow(tau, 3) + 30 * pow(tau, 4) - 10 * pow(tau, 5);
        double h3 = 6 * pow(tau, 3) - 10 * pow(tau, 4) + 3 * pow(tau, 5);

        // Interpolate position
        Eigen::Vector3d start_pos = start.block<3, 1>(0, 3);
        Eigen::Vector3d end_pos = end.block<3, 1>(0, 3);
        Eigen::Vector3d interp_pos = start_pos * (1 - h1) + end_pos * h1;

        // Interpolate orientation (assuming quaternions for simplicity)
        Eigen::Quaterniond start_quat(start.block<3, 3>(0, 0));
        Eigen::Quaterniond end_quat(end.block<3, 3>(0, 0));
        Eigen::Quaterniond interp_quat = start_quat.slerp(h1, end_quat);

        // Construct the interpolated transformation matrix
        Eigen::Matrix4d interp_pose = Eigen::Matrix4d::Identity();
        interp_pose.block<3, 3>(0, 0) = interp_quat.toRotationMatrix();
        interp_pose.block<3, 1>(0, 3) = interp_pos;
        if(i==num_samples-1)
        {
        geometry_msgs::Point trajectory_point;
        trajectory_point.x=interp_pose(0,3);
        trajectory_point.y=interp_pose(1,3);
        trajectory_point.z=interp_pose(2,3);
        test_trajectory_pub_.publish(trajectory_point);
        trajectory.push_back(interp_pose);
        }
    }

    return trajectory;
}

void DualArm::Cartesian_Plan(vector<double> &goal_pose , ArmParam &arm_param_,uint32_t id)
{
    if(hand_move ==1)
    {
        // ros::Duration(1.0).sleep();

        trajSPlannerObj = new Traj_S_Planner(7, frequency);
        hand_move = 0;
        if ((arm_param_.name == "left") )
        {
            if ((arm_param_.which_arm == 0))
            {
                ROS_INFO("Enter the Cartesian Plan,The left arm move ");
            }
            else
            {
                ROS_INFO("The left arm not move " );
                exit(0);
            }
            
        }

        if ((arm_param_.name == "right") )
        {
            if ((arm_param_.which_arm == 1))
            {
                ROS_INFO("Enter the Cartesian Plan,The right arm move " );
            }
            else
            {
                ROS_INFO("The right arm not move " );
                exit(0);
            }
            
        }
    
        // ROS_INFO("Grasp pose is %f %f %f %f %f %f %f", goal_pose[0],goal_pose[1],goal_pose[2],goal_pose[3],goal_pose[4],goal_pose[5],goal_pose[6]);
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        arm_kinematics::ArmKinematicsSolver kin_solver(urdf_path, arm_param_.base_link, arm_param_.eef_link, arm_param_.joints_min, arm_param_.joints_max);
        ROS_INFO("ArmKinematicsSolver init success!");
        ROS_INFO("URDF Path=======: %s", urdf_path.c_str());
        ROS_INFO("Base Link=======: %s", (arm_param_.base_link).c_str());
        ROS_INFO("End Effector Link==: %s", (arm_param_.eef_link).c_str());
        // if(Arrrived_GoalPose(kin_solver,arm_param_,goal_pose,id))
        // {
        //     ROS_INFO("Already in the goal_pose!!!Don't need Plan");
        //     return ;
        // }else{
        //     ROS_INFO("Not in the goal_pose!!!Start Plan");
        // }
        arm_kinematics::ArmQpIkSolver qpik_solver;

        
        Eigen::VectorXd q_ik(7);

        //cartesian trajectory planning
        vector<vector<double>> trajCartesianAllPoints;
        string commandName = "Cartesian";
        double cartesianVelocity = 0.45;
        MatrixXd InitPoseT;

        #ifdef simulation
        Eigen::VectorXd joint_cur(7);
        if(arm_param_.name == "left")
        //==============================1
        {
            if (first_time)
            {
                /* code */
                 joint_cur << -0.7, 0.60, 0, -0.78, 0.0, -0.40, 0.0;
                 first_time=false;
            }
            else
            {
                joint_cur=joint_test_last;

            }

            //std::cout << "===Left joint current===: " << joint_cur.transpose() << std::endl;
           
        }else{
            joint_cur << 0.7, -0.60, 0, 0.78, 0.0, 0.40, 0.0;
        }
        // vector<double> joint_start = {-0.187,0.222,-1.4729,-0.93926,2.52457,0.3973,-0.23314};
        
        vector<double> joint_start(joint_cur.data(),joint_cur.data()+joint_cur.size());
        // vector<double> grasp_pose={0.0502, 0.4592, -0.054598 ,0.0441899, 0.992888, 0.0526805, 0.0971909};
        #endif

        #ifdef real
        // ros::Duration(1.0).sleep();
        VectorXd joint_cur(7);
        joint_cur<< arm_param_.joint_pos[0],arm_param_.joint_pos[1],arm_param_.joint_pos[2],arm_param_.joint_pos[3],arm_param_.joint_pos[4],arm_param_.joint_pos[5],arm_param_.joint_pos[6]; 
        vector<double> joint_start(arm_param_.joint_pos);

        #endif

        
        kin_solver.getFkSolution(InitPoseT, joint_start, error_string);

        vector<double> init_pose = TrajPlannerBase::T_matrix2Quat(InitPoseT);
        // cout<<"joint_start"<<endl;
        // copy(joint_start.cbegin(),joint_start.cend(),ostream_iterator<double>(cout," "));
        // cout<<endl;
        // vector<double> middle_pose(goal_pose);
        // middle_pose[2] += 0.15;
        // vector<double> start_up_pose(start_pose);
        // start_up_pose[2] += 0.1;
        // trajCartesianAllPoints.push_back(start_pose);
        // trajCartesianAllPoints.push_back(start_up_pose);
        
        // trajCartesianAllPoints.push_back(goal_pose);

        // ROS_INFO("Enter the initInterpolation");
        // double duration = trajSPlannerObj->initInterpolation(trajCartesianAllPoints, cartesianVelocity, commandName);
        // ROS_INFO("Finish the initInterpolation");

        #ifdef simulation
        // 定义 JointState 消息
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name.resize(7);
        joint_state_msg.position.resize(7);

        if(arm_param_.name == "left")
        {
            // 设置 JointState 消息中的关节名称和初始位置
            joint_state_msg.name[0] = "left_joint1";
            joint_state_msg.name[1] = "left_joint2";
            joint_state_msg.name[2] = "left_joint3";
            joint_state_msg.name[3] = "left_joint4";
            joint_state_msg.name[4] = "left_joint5";
            joint_state_msg.name[5] = "left_joint6";
            joint_state_msg.name[6] = "left_joint7";
        }else{
            // 设置 JointState 消息中的关节名称和初始位置
            joint_state_msg.name[0] = "right_joint1";
            joint_state_msg.name[1] = "right_joint2";
            joint_state_msg.name[2] = "right_joint3";
            joint_state_msg.name[3] = "right_joint4";
            joint_state_msg.name[4] = "right_joint5";
            joint_state_msg.name[5] = "right_joint6";
            joint_state_msg.name[6] = "right_joint7"; 
        }
        #endif
        
        // double t = 0;
        VectorXd vel(7);
        vel.setZero(7);
        ros::Rate loop_rate(frequency);
        // Matrix4d last_T = InitPoseT;
        // struct timeval begin_time;
        // gettimeofday(&begin_time, NULL);

        // Define duration and number of samples
        Eigen::Matrix4d start_pose = InitPoseT;
        Eigen::Matrix4d end_pose = Eigen::Matrix4d::Identity();
        end_pose = TrajPlannerBase::Quat2T_matrix(goal_pose);
        std::vector<Eigen::Matrix4d> cartesian_trajectory;
        // Convert init_pose and goal_pose to Eigen::Quaterniond
        Eigen::Quaterniond quat_init(init_pose[3], init_pose[4], init_pose[5], init_pose[6]);
        Eigen::Quaterniond quat_goal(goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]);

        // Calculate the dot product of the two quaternions
        double dot_product = fabs(quat_init.dot(quat_goal));
        if(!handReset)
        {
            double duration1; // 2.5 seconds for the first segment
            double duration2;// 2.5 seconds for the second segment
            int num_samples1;
            int num_samples2;
            if(dot_product < 0.95)
            {
                // Define durations and number of samples for each segment
                duration1 = 2; // 2.5 seconds for the first segment
                duration2 = 1;// 2.5 seconds for the second segment
                num_samples1 = 350;
                num_samples2 = 175;
            }else{
                // Define durations and number of samples for each segment
                duration1 = 1; // 2.5 seconds for the first segment
                duration2 = 1; // 2.5 seconds for the second segment
                num_samples1 = 150;
                num_samples2 = 150;
            }
            
            vector<double> middle_pose_temp(goal_pose);
            middle_pose_temp[2] += 0.15;
            Eigen::Matrix4d middle_pose = TrajPlannerBase::Quat2T_matrix(middle_pose_temp);
            // Get the minimum jerk trajectory for each segment
            std::vector<Eigen::Matrix4d> trajectory1 = minimumJerkInterpolationCartesian(start_pose, middle_pose, duration1, num_samples1);
            std::vector<Eigen::Matrix4d> trajectory2 = minimumJerkInterpolationCartesian(middle_pose, end_pose, duration2, num_samples2);

            // Combine the two segments
            cartesian_trajectory.insert(cartesian_trajectory.end(), trajectory1.begin(), trajectory1.end());
            cartesian_trajectory.insert(cartesian_trajectory.end(), trajectory2.begin(), trajectory2.end());
        }
        else{
            int num_samples;
            double duration;
            if(dot_product < 0.95)
            {
                duration = 2.0; // 5 seconds
                num_samples = 350;
            }else{
                duration = 1; // 5 seconds
                num_samples = 175;
            }
            
            // Get the minimum jerk trajectory in Cartesian space
            cartesian_trajectory = minimumJerkInterpolationCartesian(start_pose, end_pose, duration, num_samples);
        }
        
        int i=0;
        while (ros::ok() && i < cartesian_trajectory.size())
        {
            
            // std::cout << "activate_time is " << std::to_string(double(cur_time.tv_sec + cur_time.tv_usec / 1e6)) << std::endl;

            // Matrix4d T;
            // if(!trajSPlannerObj->calCartesianInterpolation(t, last_T, T))
            // {
            //     ROS_ERROR("Cartesian Interpolation failed!");
            //     break;
            // }
            // last_T = T ;

            // ROS_INFO("Enter the IKqpSolution");
            
            q_ik = qpik_solver.IKqpSolution(&kin_solver,frequency,cartesian_trajectory[i], joint_cur, error_string,arm_param_.joints_min,arm_param_.joints_max);
            i++;
            // ROS_INFO("Finish the IKqpSolution");
            //=====2
            joint_cur = q_ik;
            joint_test_last =joint_cur;
            #ifdef real

            vector<double> newJoint;
            newJoint.resize(7);
            for (int i = 0; i < 7; i++)
            {
                newJoint[i] = q_ik(i);
            }
            // cout<<"The newJoint joints = "<<endl;
            // copy(newJoint.cbegin(),newJoint.cend(),ostream_iterator<double>(cout," "));
            // cout<<endl;

            if(arm_param_.name == "left")
            {
                
                LeftJoints_pub(newJoint, vel, left_joints_publisher);
                // ROS_INFO("LeftJoints_pub finish");
                
                // struct timeval cur_time;
                // gettimeofday(&cur_time, NULL);
                // double dTimeDiffUs = (cur_time.tv_sec - begin_time.tv_sec)*1000000 + (cur_time.tv_usec - begin_time.tv_usec);
                // ROS_INFO("LeftJoints Control[%f]us: %f,%f,%f,%f,%f,%f,%f", dTimeDiffUs,newJoint[0],newJoint[1],newJoint[2],newJoint[3],newJoint[4],newJoint[5],newJoint[6]);
            }else{
                // struct timeval cur_time;
                // gettimeofday(&cur_time, NULL);
                // std::cout << "activate_time is " << std::to_string(double(cur_time.tv_sec + cur_time.tv_usec / 1e6)) << std::endl;
                RightJoints_pub(newJoint, vel, right_joints_publisher);
                // ROS_INFO("RightJoints_pub finish");
                // struct timeval cur_time;
                // gettimeofday(&cur_time, NULL);
                // double dTimeDiffUs = (cur_time.tv_sec - begin_time.tv_sec)*1000000 + (cur_time.tv_usec - begin_time.tv_usec);
                // ROS_INFO("RightJoints Control[%f]us: %f,%f,%f,%f,%f,%f,%f", dTimeDiffUs,newJoint[0],newJoint[1],newJoint[2],newJoint[3],newJoint[4],newJoint[5],newJoint[6]);
                // ROS_INFO("Publish RightJoints to Control: %f,%f,%f,%f,%f,%f,%f",newJoint[0],newJoint[1],newJoint[2],newJoint[3],newJoint[4],newJoint[5],newJoint[6]);
            }
            #endif
            
            // t += 1.0/frequency;
            #ifdef simulation
            for (int i = 0; i < 7; i++)
            {
                joint_state_msg.position[i] = q_ik(i); 

            }
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_pub.publish(joint_state_msg);
            #endif
            loop_rate.sleep();
            // ros::SpinOnce();
        }
        // struct timeval end_time;
        // gettimeofday(&end_time, NULL);
        // double dTimeDiffs = (end_time.tv_sec - begin_time.tv_sec);
        // ROS_INFO("Publish Control time = [%f]s", dTimeDiffs);
        ROS_INFO("Before delete trajSPlannerObj");
        delete trajSPlannerObj;
        ROS_INFO("After delete trajSPlannerObj");

        #ifdef LLM
        sleep(0.5);
        if(Arrrived_GoalPose(kin_solver,arm_param_,goal_pose,id))
        {
            ROS_INFO("Cartesian Trajectory Planning finished!");
        }else{
            ROS_INFO("Cartesian Trajectory Planning failed!");
        }
        // to do
        // llm_msgs::pose_action_status grasp_status;
        // grasp_status.hand_move_success = 1;
        // arrive_pose_publisher.publish(grasp_status);

        // if(grasp_status.hand_move_success == 1)
        // {
        //     ROS_INFO("Cartesian Trajectory Planning finished!");
        // }else{
        //     ROS_INFO("Cartesian Trajectory Planning failed!");
        // }
        // std::cout << "hand_move_success_time is " << std::to_string(double(cur_time.tv_sec + cur_time.tv_usec / 1e6)) << std::endl;
        #endif

    }
    
}

// 0.7958445725629495, 0.2782065906429096, -2.160228444539954, -2.130078052154062, 0.7684380883113876, -0.7372445889897331, -0.13617722696856294, 
// -0.8562834156056438, -0.3744101957551576, 1.829003642915747, 2.146712156322697, -0.9412218493068566, 0.4309527275966223, 0.5659602942144225
void DualArm::LeftJoints_Plan(vector<double> &goal_joints)
{
    ros::NodeHandle nh;
    vector<double>  second_joints = {-0.785281197362372, 0.024114177985562278, -1.7238070754345196, -1.8227296614932949, 3.054337908898873, 0.4747325392828785, 0.0236539837491965877};
    vector<double>  third_joints = {-0.9193741279353954, 0.026535950154436737, -1.3767688493631962, -1.7861998265057832, 3.0543417438508427, 0.42266539639007017, 0.023544687618059737};
    #ifdef real
    boost::shared_ptr<sensor_msgs::JointState const> JointStates;
    JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_left", ros::Duration(5));

    vector<double> start_joints(7);
    for (int i = 0; i < 7; i++)
    {
        start_joints[i] = JointStates->position[i];
    }
    
    cout<<"The start_joints = "<<endl;
    copy(start_joints.cbegin(),start_joints.cend(),ostream_iterator<double>(cout," "));
    cout<<endl;
    #endif
    #ifdef simulation
    vector<double> start_joints ={-0.8312699413831836, 0.025663498581326775, -2.2961314724426827, -1.7861480546541921, 3.0541845108200847, 0.422680736197949, 0.024875415951550537};
    #endif
    vector<double> first_joints ={-0.8312699413831836, 0.025663498581326775, -2.2961314724426827, -1.7861480546541921, 3.0541845108200847, 0.422680736197949, 0.024875415951550537};

    vector<double> timeStamp;
    vector<vector<double>> trajCartesianAllPoints;
    trajCartesianAllPoints.push_back(start_joints);
    timeStamp.push_back(0);
    timeStamp.push_back(2);
    // trajCartesianAllPoints.push_back(first_joints);
    trajCartesianAllPoints.push_back(goal_joints);

    // timeStamp.push_back(2);
    // trajCartesianAllPoints.push_back(third_joints);
    // timeStamp.push_back(3);
    // trajCartesianAllPoints.push_back(first_joints);
    // timeStamp.push_back(4);
    // trajCartesianAllPoints.push_back(third_joints);
    // timeStamp.push_back(5);
    

    double t_cp = 0;
    vector<double> pub_ArmJoints(7);
    VectorXd vel(7);
    vel.setZero(7);
    ros::Rate loop_rate(frequency);
    Traj_CP_Planner *trajCPPlannerObj = new Traj_CP_Planner(7, frequency);

    trajCPPlannerObj->CPInit(trajCartesianAllPoints, timeStamp);
    #ifdef simulation
    // 定义 JointState 消息
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(7);
    joint_state_msg.position.resize(7);

    
    // 设置 JointState 消息中的关节名称和初始位置
    joint_state_msg.name[0] = "left_joint1";
    joint_state_msg.name[1] = "left_joint2";
    joint_state_msg.name[2] = "left_joint3";
    joint_state_msg.name[3] = "left_joint4";
    joint_state_msg.name[4] = "left_joint5";
    joint_state_msg.name[5] = "left_joint6";
    joint_state_msg.name[6] = "left_joint7";

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    #endif

    while (ros::ok() && t_cp <= timeStamp.back())
    {
        for (int i = 0; i < 7; ++i)
        {
            pub_ArmJoints[i] = trajCPPlannerObj->calCPPosition(t_cp, &trajCPPlannerObj->allJoint._CP_param[i]);
        }

        #ifdef real
        LeftJoints_pub(pub_ArmJoints, vel, left_joints_publisher);
    
        #endif
        #ifdef simulation
        for (int i = 0; i < 7; i++)
        {
            joint_state_msg.position[i] = pub_ArmJoints[i]; 

        }
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_state_msg);
        #endif
        t_cp += 1.0 / frequency;

        #ifdef debug

        #endif
        loop_rate.sleep();
    }
    delete trajCPPlannerObj;
}

void DualArm::RightJoints_Plan(vector<double> &goal_joints)
{
    ros::NodeHandle nh;
    vector<double>  second_joints = {-0.785281197362372, 0.024114177985562278, -1.7238070754345196, -1.8227296614932949, 3.054337908898873, 0.4747325392828785, 0.0236539837491965877};
    vector<double>  third_joints = {-0.9193741279353954, 0.026535950154436737, -1.3767688493631962, -1.7861998265057832, 3.0543417438508427, 0.42266539639007017, 0.023544687618059737};
    #ifdef real
    boost::shared_ptr<sensor_msgs::JointState const> JointStates;
    JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_right", ros::Duration(5));

    vector<double> start_joints(7);
    for (int i = 0; i < 7; i++)
    {
        start_joints[i] = JointStates->position[i];
    }
    
    cout<<"The start_joints = "<<endl;
    copy(start_joints.cbegin(),start_joints.cend(),ostream_iterator<double>(cout," "));
    cout<<endl;
    #endif
    #ifdef simulation
    vector<double> start_joints ={0.8312699413831836, -0.025663498581326775, 2.2961314724426827, 1.7861480546541921, -3.0541845108200847, -0.422680736197949, -0.024875415951550537};
    #endif
    vector<double> first_joints ={-0.8312699413831836, 0.025663498581326775, -2.2961314724426827, -1.7861480546541921, 3.0541845108200847, 0.422680736197949, 0.024875415951550537};

    vector<double> timeStamp;
    vector<vector<double>> trajCartesianAllPoints;
    trajCartesianAllPoints.push_back(start_joints);
    timeStamp.push_back(0);
    timeStamp.push_back(1);
    trajCartesianAllPoints.push_back(goal_joints);

    double t_cp = 0;
    vector<double> pub_ArmJoints(7);
    VectorXd vel(7);
    vel.setZero(7);
    ros::Rate loop_rate(frequency);
    Traj_CP_Planner *trajCPPlannerObj = new Traj_CP_Planner(7, frequency);

    trajCPPlannerObj->CPInit(trajCartesianAllPoints, timeStamp);
    #ifdef simulation
    // 定义 JointState 消息
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(7);
    joint_state_msg.position.resize(7);

    joint_state_msg.name[0] = "right_joint1";
    joint_state_msg.name[1] = "right_joint2";
    joint_state_msg.name[2] = "right_joint3";
    joint_state_msg.name[3] = "right_joint4";
    joint_state_msg.name[4] = "right_joint5";
    joint_state_msg.name[5] = "right_joint6";
    joint_state_msg.name[6] = "right_joint7";

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    #endif

    while (ros::ok() && t_cp <= timeStamp.back())
    {
        for (int i = 0; i < 7; ++i)
        {
            pub_ArmJoints[i] = trajCPPlannerObj->calCPPosition(t_cp, &trajCPPlannerObj->allJoint._CP_param[i]);
        }

        #ifdef real
        RightJoints_pub(pub_ArmJoints, vel, right_joints_publisher);
        #endif
        #ifdef simulation
        for (int i = 0; i < 7; i++)
        {
            joint_state_msg.position[i] = pub_ArmJoints[i]; 

        }
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_state_msg);
        #endif
        t_cp += 1.0 / frequency;

        #ifdef debug

        #endif
        loop_rate.sleep();
    }
    delete trajCPPlannerObj;
}

void DualArm::Wave_Arm(vector<double> &goal_joints)
{
    ros::NodeHandle nh;
    #ifdef left_hello
    vector<double>  second_joints = {-0.785281197362372, 0.024114177985562278, -1.7238070754345196, -1.8227296614932949, 3.054337908898873, 0.4747325392828785, 0.0236539837491965877};
    vector<double>  third_joints = {-0.9193741279353954, 0.026535950154436737, -1.3767688493631962, -1.7861998265057832, 3.0543417438508427, 0.42266539639007017, 0.023544687618059737};
    #endif

    #ifdef right_hello
    vector<double>  second_joints = {0.785281197362372, -0.024114177985562278, 1.7238070754345196, 1.8227296614932949, -3.054337908898873, -0.4747325392828785, -0.0236539837491965877};
    vector<double>  third_joints = {0.9193741279353954, -0.026535950154436737, 1.3767688493631962, 1.7861998265057832, -3.0543417438508427, -0.42266539639007017, -0.023544687618059737};
    #endif
    
    #ifdef real

    #ifdef left_hello
    boost::shared_ptr<sensor_msgs::JointState const> JointStates;
    JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_left", ros::Duration(5));
    #endif

    #ifdef right_hello
    boost::shared_ptr<sensor_msgs::JointState const> JointStates;
    JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_right", ros::Duration(5));
    #endif

    vector<double> start_joints(7);
    for (int i = 0; i < 7; i++)
    {
        start_joints[i] = JointStates->position[i];
    }
    
    cout<<"The start_joints = "<<endl;
    copy(start_joints.cbegin(),start_joints.cend(),ostream_iterator<double>(cout," "));
    cout<<endl;
    #endif
    #ifdef simulation
    vector<double> start_joints ={0,0,0,0,0,0,0};
    #endif
    // vector<double> first_joints ={-0.8312699413831836, 0.025663498581326775, -2.2961314724426827, -1.7861480546541921, 3.0541845108200847, 0.422680736197949, 0.024875415951550537};

    vector<double> timeStamp;
    vector<vector<double>> trajCartesianAllPoints;
    trajCartesianAllPoints.push_back(start_joints);
    timeStamp.push_back(0);
    timeStamp.push_back(2);
    trajCartesianAllPoints.push_back(goal_joints);
    timeStamp.push_back(3);
    trajCartesianAllPoints.push_back(third_joints);
    timeStamp.push_back(4);
    trajCartesianAllPoints.push_back(goal_joints);
    timeStamp.push_back(5);
    trajCartesianAllPoints.push_back(third_joints);
    timeStamp.push_back(6);
    trajCartesianAllPoints.push_back(start_joints);
    timeStamp.push_back(7);
    

    double t_cp = 0;
    vector<double> pub_ArmJoints(7);
    VectorXd vel(7);
    vel.setZero(7);
    ros::Rate loop_rate(frequency);
    Traj_CP_Planner *trajCPPlannerObj = new Traj_CP_Planner(7, frequency);

    trajCPPlannerObj->CPInit(trajCartesianAllPoints, timeStamp);
    #ifdef simulation
    // 定义 JointState 消息
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(7);
    joint_state_msg.position.resize(7);

    #ifdef left_hello
    // 设置 JointState 消息中的关节名称和初始位置
    joint_state_msg.name[0] = "left_joint1";
    joint_state_msg.name[1] = "left_joint2";
    joint_state_msg.name[2] = "left_joint3";
    joint_state_msg.name[3] = "left_joint4";
    joint_state_msg.name[4] = "left_joint5";
    joint_state_msg.name[5] = "left_joint6";
    joint_state_msg.name[6] = "left_joint7";
    #endif

    #ifdef right_hello
    // 设置 JointState 消息中的关节名称和初始位置
    joint_state_msg.name[0] = "right_joint1";
    joint_state_msg.name[1] = "right_joint2";
    joint_state_msg.name[2] = "right_joint3";
    joint_state_msg.name[3] = "right_joint4";
    joint_state_msg.name[4] = "right_joint5";
    joint_state_msg.name[5] = "right_joint6";
    joint_state_msg.name[6] = "right_joint7";
    #endif

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    #endif

    while (ros::ok() && t_cp <= timeStamp.back())
    {
        for (int i = 0; i < 7; ++i)
        {
            pub_ArmJoints[i] = trajCPPlannerObj->calCPPosition(t_cp, &trajCPPlannerObj->allJoint._CP_param[i]);
        }

        #ifdef real

        #ifdef left_hello
        LeftJoints_pub(pub_ArmJoints, vel, left_joints_publisher);
        #endif

        #ifdef right_hello
        LeftJoints_pub(pub_ArmJoints, vel, right_joints_publisher);
        #endif

        #endif
        #ifdef simulation
        for (int i = 0; i < 7; i++)
        {
            joint_state_msg.position[i] = pub_ArmJoints[i]; 

        }
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_state_msg);
        #endif
        t_cp += 1.0 / frequency;

        #ifdef debug

        #endif
        loop_rate.sleep();
    }
    delete trajCPPlannerObj;
}

bool DualArm::Arrrived_GoalPose(arm_kinematics::ArmKinematicsSolver &kin_solver,ArmParam &arm_param_,std::vector<double> &goal_pose,uint32_t id)
{
    // boost::shared_ptr<sensor_msgs::JointState const> JointStates;
    // if(arm_param_.name == "left")
    // {
    //     JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_left", ros::Duration(5));
    // }else{
    //     JointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/human_arm_state_right", ros::Duration(5));
    // }
    // vector<double> cur_joints(7);
    // for (int i = 0; i < 7; i++)
    // {
    //     cur_joints[i] = JointStates->position[i];
    // }
    // cout<<"The current joints = "<<endl;
    // copy(cur_joints.cbegin(),cur_joints.cend(),ostream_iterator<double>(cout," "));
    // cout<<endl;
    // Eigen::MatrixXd P_cur(4,4);
    // kin_solver.getFkSolution(P_cur,cur_joints, error_string);
    // vector<double> pos_cur = TrajPlannerBase::T_matrix2Quat(P_cur);

    // cout<<"The current pose = "<<endl;
    // copy(pos_cur.cbegin(),pos_cur.cend(),ostream_iterator<double>(cout," "));
    // cout<<endl;
    // cout<<"The goal pose = "<<endl;
    // copy(goal_pose.cbegin(),goal_pose.cend(),ostream_iterator<double>(cout," "));
    // cout<<endl;
    // if(Judge_arrived(pos_cur,goal_pose))
    // {
    llm_msgs::pose_action_status grasp_status;
    
    grasp_status.hand_move_success = 1;
    grasp_status.resp_frame_id = id;
    grasp_status.header.stamp = ros::Time::now();
    ROS_INFO("resp_frame_id = %d",grasp_status.resp_frame_id);
    arrive_pose_publisher.publish(grasp_status);
    ROS_INFO("Arrive the target pose");
    return true;
    // }else
    // {
    //     ROS_INFO("Not arrive the target pose");
    //     return false;
    // }
}

void Say_Hello(DualArm &dual_arm)
{
    // vector<double>  left_joints = {-0.8312699413831836, 0.025663498581326775, -2.2961314724426827, -1.7861480546541921, 3.0541845108200847, 0.422680736197949, 0.024875415951550537};
    // dual_arm.Wave_Arm(left_joints);
    vector<double>  right_joints = {0.8312699413831836, -0.025663498581326775, 2.2961314724426827, 1.7861480546541921, -3.0541845108200847, -0.422680736197949, -0.024875415951550537};
    dual_arm.Wave_Arm(right_joints);
}

void Hug_Brand(DualArm &dual_arm)
{
    #ifdef unveiling_ceremony
    // 创新中心牌
    vector<double>  left_joints = {0.8562834156056438, 0.3744101957551576, -1.829003642915747, -2.146712156322697, 0.9412218493068566, -0.3309527275966223, -0.1659602942144225};
    vector<double>  right_joints = {-0.8562834156056438, -0.3744101957551576, 1.829003642915747, 2.146712156322697, -0.9412218493068566, 0.3309527275966223, 0.1659602942144225};
    #endif

    #ifdef hug_emblem
    // 会徽
    vector<double>  left_joints = {0.8562834156056438, 0.0744101957551576, -1.829003642915747, -2.146712156322697, 0.9412218493068566, -0.5309527275966223, -0.0659602942144225};
    vector<double>  right_joints = {-0.8562834156056438, -0.0744101957551576, 1.829003642915747, 2.146712156322697, -0.9412218493068566, 0.5309527275966223, 0.0659602942144225};
    #endif
    dual_arm.LeftJoints_Plan(left_joints);
    dual_arm.RightJoints_Plan(right_joints);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"trajectory_planning");
    ros::NodeHandle nh("~");
    DualArm dual_arm(nh);
    //打招呼
    // thread joint_thread(Say_Hello, ref(dual_arm));
    // joint_thread.detach();
    // //抱创新中心牌、会徽
    // thread hug_thread(Hug_Brand, ref(dual_arm));
    // hug_thread.detach();
    ros::spin();
    // ros::waitForShutdown();

    return 0;
}

