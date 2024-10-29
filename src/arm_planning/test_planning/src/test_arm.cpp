#include "test_arm.h"  
#include <iostream>  
#include <cstdlib>  
#include <ctime>  
#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>


#define SIM  

SmoothingFilter::SmoothingFilter(size_t windowSize) : windowSize(windowSize), index(0) 
{  
    values.resize(windowSize, {0.0, 0.0, 0.0});  
}  
  
std::array<double, 3> SmoothingFilter::filter(const std::array<double, 3>& input_force) 
{  
    values[index] = input_force;  
  
    std::array<double, 3> smoothed_value = {0.0, 0.0, 0.0};  
    for (const auto& val : values) 
    {  
        smoothed_value[0] += val[0];  
        smoothed_value[1] += val[1];  
        smoothed_value[2] += val[2];  
    }  
  
    smoothed_value[0] /= windowSize;  
    smoothed_value[1] /= windowSize;  
    smoothed_value[2] /= windowSize;  
  
    index = (index + 1) % windowSize;  
    return smoothed_value;  
}  
  
ArmController::ArmController(ros::NodeHandle& nh)   : filter_left(3), filter_right(3)   
    {  
        left_arm_pub_ = nh.advertise<llm_msgs::hand_pose_req>("/left_arm_pose_req", 10);
        right_arm_pub_ = nh.advertise<llm_msgs::hand_pose_req>("/right_arm_pose_req", 10);

        left_sub = nh.subscribe("desired_pose_left", 10, &ArmController::positionCallback_left, this);
        right_sub = nh.subscribe("desired_pose_right", 10, &ArmController::positionCallback_right, this);
        
        #ifdef SIM
            // joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);
            // joint_state_sub = nh.subscribe("/joint_states", 10, &ArmController::jointStateCallback, this);
            joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_1111states", 10);


        #else

            // 左右关节发布指令
            left_joints_publisher  = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_left", 10);
            right_joints_publisher = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_right", 10);

            joint_state_sub_left_teleoperation = nh.subscribe("/human_arm_cmd_left" , 10, &ArmController::jointStateCallback_left_teleoperation, this);
            joint_state_sub_right_teleoperation = nh.subscribe("/human_arm_cmd_right" , 10, &ArmController::jointStateCallback_right_teleoperation, this);

            joint_state_sub_left = nh.subscribe("/human_arm_state_left" , 10, &ArmController::jointStateCallback_left, this);
            joint_state_sub_right = nh.subscribe("/human_arm_state_right" , 10, &ArmController::jointStateCallback_right, this);
            
        #endif
        // joint_state_sub = nh.subscribe("/human_arm_state_left", 10, &ArmController::jointStateCallback, this);

        first();
        ros::Rate r(198);
        /////////////////////////
        while (ros::ok()) {
            // command();
            // publishGraspPoses();
            // right_jacob();
            // left_jacob();
            test_jacob();
            ros::spinOnce();    
            r.sleep();
        }
    }  
  
    void ArmController::Joints_pub(std::vector<double> pos, ros::Publisher pub) 
    {

        sensor_msgs::JointState joint_state;
        joint_state.name.resize(7); 
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"1","2","3","4","5","6","7"};
        for (int i = 0; i < 7; i++)
        {
            joint_state.position[i] = pos[i];
            // joint_state.velocity[i] = vel(i);
        }
        pub.publish(joint_state);
    }

     void ArmController::left_jacob()
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        std::vector<double> temp(7);


        for (size_t i = 0; i < temp.size(); ++i) 
        {
            temp[i] = joint_positions_L[i]; // 或者任何其他值
        }


        std::string error_string_left;

        Eigen::MatrixXd InitPoseT_left;
        // std::cout<<"==1=="<<std::endl;
        arm_kinematics::ArmKinematicsSolver kin_solver_leftSim(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);
        //step 1  正解
        kin_solver_leftSim.getFkSolution(InitPoseT_left, temp, error_string_left);
        Eigen::MatrixXd jaco;
        std::string error_string;
        // std::cout<<"==1=="<<std::endl;

        //step 2  获取雅可比
        kin_solver_leftSim.getJacobian(jaco,temp,error_string);
        // std::cout<<"==jacobian=="<<jaco<<std::endl;
        
        Eigen::VectorXd v_end(6); // 末端速度 (vx, vy, vz, wx, wy, wz)
        
        //测试例子
        double radius = 0.2; // 圆的半径
        double angular_velocity = 1.0; // 角速度
        double time = ros::Time::now().toSec(); // 当前时间
        //往复测试
        double xx = radius * cos(angular_velocity * time);

        double yy = radius * sin(angular_velocity * time);
        v_end<<0,0.1*xx,0,0,0,0;

        //左侧导纳角速度
        Eigen::Quaterniond q; 
        q.x()=qx_left;
        q.y()=qy_left;
        q.z()=qz_left;
        q.w()=qw_left;
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();  
        Eigen::AngleAxisd angle_axis(rotationMatrix);
        Eigen::Vector3d temp_voir;
        temp_voir = angle_axis.angle() * angle_axis.axis();
        // std::cout<<"======="<<  temp_voir.transpose()<<std::endl;
        // std::cout<<"==3=="<<std::endl;
        //step 3 末端转换
        Eigen::Vector3d tempPos(xx_left, yy_left, zz_left);
        Eigen::Vector3d tempOri(temp_voir(0), temp_voir(1), temp_voir(2));
        tempPos=InitPoseT_left.block<3,3>(0,0)*tempPos;               
        tempOri=InitPoseT_left.block<3,3>(0,0)*tempOri;        
        Eigen::VectorXd combined(6);  
        // 0.1*tempPos
        combined << 0.1*tempPos,tempOri;

        //导纳输出6维速度时 第一针信号可能是空 
        bool hasNaN = false;  
        for (int i = 0; i < 6; ++i) 
        {   
            if (std::isnan(combined(i)) )
            {  
                hasNaN = true;  
                break;  
            }  
        }  
    
        if (hasNaN) 
        {  
            combined << 0,0,0,
                        0,0,0;   
        }  
       

        // std::cout<<"===L===="<< combined.transpose()<<std::endl;
        // combined=combined;
        //  std::cout<<"==4=="<<std::endl;
        //step 4 计算关节速度
        Eigen::MatrixXd J_regularized = jaco.completeOrthogonalDecomposition().pseudoInverse();
        //step 5 正则化参数
        double lambda = 0.005;  
        double some_threshold = 0.075;
        double some_factor = 15;
        // 计算每个关节到其边界的距离，并据此调整lambda  
        for (unsigned int i = 0; i < 7; i++) 
        {  
            double distance_to_min = abs(temp[i] - left_min[i]);  
            double distance_to_max = abs(left_max[i] - temp[i]);  
            double distance_to_boundary = std::min(distance_to_min, distance_to_max);  
        
            // 如果关节靠近边界，则增加lambda以减少运动  
            if (distance_to_boundary < some_threshold) 
            {  
                lambda *= some_factor; // some_factor > 1，例如1.5或2  
            }  
        }  
        Eigen::MatrixXd regularized_term = (jaco * jaco.transpose() + lambda * Eigen::MatrixXd::Identity(jaco.rows(), jaco.rows())).inverse();
        Eigen::MatrixXd J_pseudo_inverse = jaco.transpose() * regularized_term;
        Eigen::VectorXd q_dot = J_pseudo_inverse * combined;
        double maxQv = q_dot.maxCoeff();  // 获取q_dot中的最大值  
        double minQv = q_dot.minCoeff();  // 获取q_dot中的最小值  
        
        //限制 1  --------最终角速度不能超过 1.85rad/s  
        // 设定角速度的最大允许值  
        double max_allowed_speed = 0.005 * 1.85;  
        
        // 检查角速度是否超出限制，并进行相应调整  
        if (maxQv > max_allowed_speed || minQv < -max_allowed_speed) 
        {  
            double scale_factor = max_allowed_speed / std::max(std::abs(maxQv), std::abs(minQv));  
            q_dot *= scale_factor;  
        }  
        Eigen::VectorXd dq(7);
        for (int i = 0; i < 7; i++)
        {
            dq(i)=desired_joint_R[i];
            /* code */
        }
        double q_dot_norm;
        q_dot_norm = q_dot.norm();
        double desired_norm;
        desired_norm=dq.norm();
        double weight_q_dot = q_dot_norm / (q_dot_norm + desired_norm);  
        double weight_desired = desired_norm / (q_dot_norm + desired_norm); 
        for ( int i = 0; i < 7; i++) 
        {
            temp[i] = joint_positions_L[i]+0.9*weight_q_dot*q_dot(i)+0.1*weight_desired*desired_joint_L[i];
            // 检查新角度是否超出边界
              
            if (temp[i] > left_max[i]-0.005) 
             {  
                temp[i] = left_max[i]-0.005; // 设置为最大值  

             }
             else if (temp[i] < left_min[i]+0.005) 
             {  
                temp[i] = left_min[i]+0.005; // 设置为最小值  
             }  
            //  temp[0]=temp[0]+0.002*sin(time);
             // 更新角度，确保在边界内  
        }
                Eigen::VectorXd testr(7);
        for (int i = 0; i < 7; i++)
        {
            testr(i)=q_dot(i)+desired_joint_L[i];
        }
        // std::cout<<"===l===="<<testr.transpose()<<std::endl;
        // std::cout<<"====="<<temp[0]<<";"<<temp[1]<<";"<<temp[2]<<";"<<temp[3]<<";"<<temp[4]<<";"<<temp[5]<<";"<<temp[6]<<";"<<std::endl;
 
        //step 6 发布
        Joints_pub(temp,left_joints_publisher);
        
    }

    void ArmController::right_jacob()
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        std::vector<double> temp(7);


        for (size_t i = 0; i < temp.size(); ++i) 
        {
            temp[i] = joint_positions_R[i];
        }

        std::string error_string_right;
        Eigen::MatrixXd InitPoseT_right;
        arm_kinematics::ArmKinematicsSolver kin_solver_rightSim(urdf_path, arm_param_right.base_link, arm_param_right.eef_link, arm_param_right.joints_min, arm_param_right.joints_max);
        kin_solver_rightSim.getFkSolution(InitPoseT_right, temp, error_string_right);
        Eigen::MatrixXd jaco;
        std::string error_string;
        kin_solver_rightSim.getJacobian(jaco,temp,error_string);    
        Eigen::VectorXd v_end(6);
        double radius = 0.2; 
        double angular_velocity = 1.0; 
        double time = ros::Time::now().toSec(); 
        double xx = radius * cos(angular_velocity * time);

        double yy = radius * sin(angular_velocity * time);
        v_end<<0,0.1*xx,0,0,0,0;
        Eigen::Quaterniond q; 
        q.x()=qx_right;
        q.y()=qy_right;
        q.z()=qz_right;
        q.w()=qw_right; 
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();  
        Eigen::AngleAxisd angle_axis(rotationMatrix);
        Eigen::Vector3d temp_voir;
        temp_voir = angle_axis.angle() * angle_axis.axis();
        Eigen::Vector3d tempPos(xx_right, yy_right, zz_right);
        Eigen::Vector3d tempOri(temp_voir(0), temp_voir(1), temp_voir(2));
        tempPos=InitPoseT_right.block<3,3>(0,0)*tempPos;               
        tempOri=InitPoseT_right.block<3,3>(0,0)*tempOri;
        Eigen::VectorXd combined(6);  
        combined << 0.1*tempPos,tempOri; 
        bool hasNaN_input = false;  
        for (int i = 0; i < 6; ++i) 
        {   
            if (std::isnan(combined(i)) )
            {  
                hasNaN_input = true;  
                break;  
            }  
        }  
    
        if (hasNaN_input) 
        {  
            combined << 0,0,0,
                        0,0,0;   
        }  
        Eigen::MatrixXd J_regularized = jaco.completeOrthogonalDecomposition().pseudoInverse();
        double lambda = 0.005;  
        double some_threshold = 0.15;
        double some_factor = 15;
        for (unsigned int i = 0; i < 7; i++) 
        {  
            double distance_to_min = abs(temp[i] - right_min[i]);  
            double distance_to_max = abs(right_max[i] - temp[i]);  
            double distance_to_boundary = std::min(distance_to_min, distance_to_max);  
            if (distance_to_boundary < some_threshold) 
            {  
                lambda *= some_factor; 
            }  
        }  
        Eigen::MatrixXd regularized_term = (jaco * jaco.transpose() + lambda * Eigen::MatrixXd::Identity(jaco.rows(), jaco.rows())).inverse();
        Eigen::MatrixXd J_pseudo_inverse = jaco.transpose() * regularized_term;
        Eigen::VectorXd q_dot = J_pseudo_inverse * combined;


        bool hasNaN_output = false;  
        for (int i = 0; i < 6; ++i) 
        {   
            if (std::isnan(q_dot(i)) )
            {  
                hasNaN_output = true;  
                break;  
            }  
        }  
    
        if (hasNaN_output) 
        {  
            q_dot << 0,0,0,
                        0,0,0,0;   
        }  


        double maxQv = q_dot.maxCoeff(); 
        double minQv = q_dot.minCoeff(); 

        double max_allowed_speed = 0.005 * 1.75;  

        if (maxQv > max_allowed_speed || minQv < -max_allowed_speed) 
        {  
            double scale_factor = max_allowed_speed / std::max(std::abs(maxQv), std::abs(minQv));  
            q_dot *= scale_factor;  
        }  
                
        Eigen::VectorXd dq(7);
        for (int i = 0; i < 7; i++)
        {
            dq(i)=desired_joint_R[i];
        }
        double q_dot_norm;
        q_dot_norm = q_dot.norm();
        double desired_norm;
        desired_norm=dq.norm();
        double weight_q_dot = q_dot_norm / (q_dot_norm + desired_norm);  
        double weight_desired = desired_norm / (q_dot_norm + desired_norm); 
        for (unsigned int i = 0; i < 7; i++) 
        { 
            temp[i] = joint_positions_R[i]+0.03*(desired_joint_R[i]-joint_positions_R[i])+2*q_dot(i);
            if (temp[i] > right_max[i]-0.005) 
             {  
                temp[i] = right_max[i]-0.005;

             }
             else if (temp[i] < right_min[i]+0.005) 
             {  
                temp[i] = right_min[i]-0.005;  
             }  

        }
        Eigen::VectorXd testr(7);
        for (int i = 0; i < 7; i++)
        {
            testr(i)=q_dot(i)+desired_joint_R[i];
        }
        Joints_pub(temp,right_joints_publisher);
        
    }
    
    Eigen::VectorXd computeNullSpaceOscillation(const Eigen::MatrixXd& jaco, 
                                                const Eigen::VectorXd& null_velocity_amplitude, 
                                                double time, 
                                                double frequency) 
    {
        
        // 生成零空间中的往复运动速度
        Eigen::VectorXd null_velocity = null_velocity_amplitude * std::sin(M_PI * frequency * time);

        // 计算雅可比矩阵的伪逆
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jaco);
        Eigen::MatrixXd jaco_pseudo_inv = cod.pseudoInverse();

        // 计算零空间投影矩阵
        Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(jaco.cols(), jaco.cols());
        Eigen::MatrixXd null_space_projection = identity - jaco_pseudo_inv * jaco;

        // 计算零空间中的角速度
        Eigen::VectorXd joint_velocity = null_space_projection * null_velocity;
        // std::cout<<(joint_velocity.normalized()).transpose()<<std::endl;
        return joint_velocity;
    }
    void ArmController::test_jacob()
    {
        std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        urdf_path += "/models/AUBO_HRM.urdf";
        std::vector<double> temp(7);
        Eigen::VectorXd tempV = Eigen::VectorXd::Zero(7);
        for (size_t i = 0; i < temp.size(); ++i) 
        {

            temp[i] = joint_positions_L[i]; // 或者任何其他值
            tempV(i)= temp[i];

        }

        std::string error_string_left;

        Eigen::MatrixXd InitPoseT_left;

        arm_kinematics::ArmKinematicsSolver kin_solver_leftSim(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);

        kin_solver_leftSim.getFkSolution(InitPoseT_left, temp, error_string_left);
        Eigen::MatrixXd jaco;
        std::string error_string;

        kin_solver_leftSim.getJacobian(jaco,temp,error_string);
        Eigen::VectorXd null_velocity_amplitude; // 设定零空间振荡的幅度
        // double time = ros::Time::now(); // 当前时间
        double time = ros::Time::now().toSec(); 

        double frequency = 0.5; // 设定振荡频率

        // 示例：将每个关节的幅度设定为0.1（或根据需求调整）
        null_velocity_amplitude = Eigen::VectorXd::Constant(jaco.cols(), 0.15);

        // 调用函数计算关节速度
        Eigen::VectorXd joint_velocity = computeNullSpaceOscillation(jaco, null_velocity_amplitude, time, frequency);

        


        // std::cout<<"==jacobian=="<<jaco<<std::endl;
        
        Eigen::VectorXd v_end(6); // 末端速度 (vx, vy, vz, wx, wy, wz)


        double radius = 0.28; // 圆的半径
        double angular_velocity = 2.0; // 角速度
        //往复测试
        double xx = radius * cos(angular_velocity * time);

        double yy = radius * sin(angular_velocity * time);

        Eigen::Quaterniond q(qx_left, qy_left, qz_left, qw_left);  
        // 将四元数转换为旋转矩阵  
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();  
        Eigen::AngleAxisd angle_axis(rotationMatrix);
        Eigen::Vector3d temp_voir;
        temp_voir = angle_axis.angle() * angle_axis.axis(); // 返回旋转矢量
        // v_end<<0.005*xx,0.005*yy,0,0,0.0005*yy,0;
        v_end<<0.005*yy,0.005*xx,0,0,0.0005*xx,0;

        Eigen::Vector3d tempPos(v_end(0), v_end(1), v_end(2));
        Eigen::Vector3d tempOri(v_end(3), v_end(4), v_end(5));


        tempPos=InitPoseT_left.block<3,3>(0,0)*tempPos;
        tempOri=InitPoseT_left.block<3,3>(0,0)*tempOri;
        Eigen::VectorXd combined(6);  
        combined << tempPos, tempOri; 
        // 计算关节速度

        Eigen::MatrixXd J_regularized = jaco.completeOrthogonalDecomposition().pseudoInverse();
        // 正则化参数
        double lambda = 0.001;
        // 正则化方法
        Eigen::MatrixXd regularized_term = (jaco * jaco.transpose() + lambda * Eigen::MatrixXd::Identity(jaco.rows(), jaco.rows())).inverse();
        Eigen::MatrixXd J_pseudo_inverse = jaco.transpose() * regularized_term;

        // Eigen::MatrixXd J_pseudo_inverse = jaco.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd q_dot = J_pseudo_inverse * combined;

        Eigen::VectorXd left_min(7);
        left_min << -3.1415926, -0.2, -2.268928, -1.779, -2.268928, -0.6454, -0.6454;

        Eigen::VectorXd left_max(7);
        left_max << 0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454;

        
        auto qpresult =  optimizeNullSpaceIncrement(jaco,
                                            tempV, 
                                            q_dot, 
                                            left_min, 
                                            left_max, 
                                            q_dot_last, 
                                            10, 
                                            10, 
                                            10);
        // for (unsigned int i = 0; i < 7; i++) 
        // {
        //     joint_positions_L[i] = joint_positions_L[i]+q_dot(i);
        // }
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name.resize(7);
        joint_state_msg.position.resize(7);
        joint_state_msg.velocity.resize(7);
        
        // // Set joint names
        joint_state_msg.name = 
        {
            "left_joint1", "left_joint2", "left_joint3",
            "left_joint4", "left_joint5", "left_joint6", "left_joint7"
        };
         joint_state_msg.header.stamp = ros::Time::now();
        for (int i = 0; i < 7; i++)
        {
            joint_positions_L[i] = joint_positions_L[i]+q_dot(i)+qpresult(i);// 或者任何其他值
            // joint_positions_L[i] = joint_positions_L[i]+joint_velocity[i]+q_dot(i);// 或者任何其他值
            joint_state_msg.position[i]=joint_positions_L[i];
        }
        q_dot_last = q_dot + qpresult;
        // joint_state_msg.header.stamp = ros::Time::now();
        
        joint_pub.publish(joint_state_msg);
    }
    void ArmController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        // std::string urdf_path = ros::package::getPath("arm_kinematics_solver");
        // urdf_path += "/models/AUBO_HRM.urdf";
        // joint_positions_L = msg->position; // 保存 joint position

        // std::string error_string_left;

        // Eigen::MatrixXd InitPoseT_left;

        // arm_kinematics::ArmKinematicsSolver kin_solver_leftSim(urdf_path, arm_param_left.base_link, arm_param_left.eef_link, arm_param_left.joints_min, arm_param_left.joints_max);

        // kin_solver_leftSim.getFkSolution(InitPoseT_left, joint_positions_L, error_string_left);
        // Eigen::MatrixXd jaco;
        // std::string error_string;
        // kin_solver_leftSim.getJacobian(jaco,joint_positions_L,error_string);
        //         std::cout<<"==jacobian=="<<jaco<<std::endl;
        
        //         Eigen::VectorXd v_end(6); // 末端速度 (vx, vy, vz, wx, wy, wz)
        // v_end<<0.0001,0,0,0,0,0;
        // // 计算关节速度
        // Eigen::MatrixXd J_pseudo_inverse = jaco.completeOrthogonalDecomposition().pseudoInverse();
        // Eigen::VectorXd q_dot = J_pseudo_inverse * v_end;
        // for (unsigned int i = 0; i < 7; i++) {
        //     joint_positions_L[i] = joint_positions_L[i]+q_dot(i);
        // }



        // // L_grasp_pose[0] = InitPoseT_left(0, 3);
        // // L_grasp_pose[1] = InitPoseT_left(1, 3);
        // // L_grasp_pose[2] = InitPoseT_left(2, 3);


        // auto joint_positions_R = msg->position; // 保存 joint position

        // std::string error_string_right;

        // Eigen::MatrixXd InitPoseT_right;
        // arm_kinematics::ArmKinematicsSolver kin_solver_rightSim(urdf_path, arm_param_right.base_link, arm_param_right.eef_link, arm_param_right.joints_min, arm_param_right.joints_max);

        // kin_solver_rightSim.getFkSolution(InitPoseT_right, joint_positions_R, error_string_right);

    }
        void printMatrix(const std::string& name, const Eigen::MatrixXd& matrix) {
        std::cout << name << " matrix:" << std::endl << matrix << std::endl;
    }

    void printVector(const std::string& name, const Eigen::VectorXd& vector) {
        std::cout << name << ":" << std::endl << vector.transpose() << std::endl;
    }


Eigen::VectorXd ArmController::optimizeNullSpaceIncrement(
    Eigen::MatrixXd& jaco, 
    Eigen::VectorXd& joint_angles, 
    Eigen::VectorXd& delta_q_desired, 
    Eigen::VectorXd& joint_limits_min, 
    Eigen::VectorXd& joint_limits_max, 
    Eigen::VectorXd& prev_delta_q, 
    double weight_limit_avoidance, 
    double weight_singularity_avoidance, 
    double weight_smoothness) 
{
    // 打印输入参数
    // printMatrix("Jacobian", jaco);
    // printVector("Joint Angles", joint_angles);
    // printVector("Desired Delta Q", delta_q_desired);
    // printVector("Joint Limits Min", joint_limits_min);
    // printVector("Joint Limits Max", joint_limits_max);
    // printVector("Previous Delta Q", prev_delta_q);
    // std::cout << "Weight Limit Avoidance: " << weight_limit_avoidance << std::endl;
    // std::cout << "Weight Singularity Avoidance: " << weight_singularity_avoidance << std::endl;
    // std::cout << "Weight Smoothness: " << weight_smoothness << std::endl;

    int num_joints = joint_angles.size();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_joints, num_joints);

    // 计算雅可比矩阵的广义逆和零空间投影矩阵
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jaco);
    Eigen::MatrixXd jaco_pseudo_inv = cod.pseudoInverse();
    Eigen::MatrixXd null_space_projection = Eigen::MatrixXd::Identity(num_joints, num_joints) - jaco_pseudo_inv * jaco;

    // 构建 H 矩阵增加到什么位置
    H += weight_singularity_avoidance * null_space_projection.transpose() * null_space_projection;
    H += weight_smoothness * null_space_projection.transpose() * null_space_projection;

    for (int i = 0; i < num_joints; ++i) {
        H(i, i) += weight_limit_avoidance;
    }

    // Eigen::VectorXd  = Eigen::VectorXd::Zero(num_joints,0.001);
    Eigen::VectorXd f = Eigen::VectorXd::Constant(num_joints, 0.000001); // 小的非零值

    Eigen::VectorXd lower_bound = joint_limits_min - joint_angles - delta_q_desired;
    Eigen::VectorXd upper_bound = joint_limits_max - joint_angles - delta_q_desired;

    // 使用 OSQPEigen 进行优化求解
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setRelativeTolerance(1e-6);

    solver.data()->setNumberOfVariables(num_joints);
    solver.data()->setNumberOfConstraints(num_joints);

    // 设置 Hessian 矩阵
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    if (!solver.data()->setHessianMatrix(H_sparse)) {
        std::cerr << "Error setting Hessian matrix" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    if (!solver.data()->setGradient(f)) {
        std::cerr << "Error setting gradient vector" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    // 设置约束矩阵
    Eigen::SparseMatrix<double> A_sparse = Eigen::MatrixXd::Identity(num_joints, num_joints).sparseView();
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        std::cerr << "Error setting constraint matrix" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    if (!solver.data()->setLowerBound(lower_bound)) {
        std::cerr << "Error setting lower bounds" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    if (!solver.data()->setUpperBound(upper_bound)) {
        std::cerr << "Error setting upper bounds" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    if (!solver.initSolver()) {
        std::cerr << "Solver initialization failed" << std::endl;
        return Eigen::VectorXd::Zero(num_joints);
    }

    auto result = solver.solve();
    // if (result != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Solver failed to find a solution. Error: " << result << std::endl;
        // return Eigen::VectorXd::Zero(num_joints);
    // }

    Eigen::VectorXd delta_q_null = solver.getSolution();
    std::cout << "qp==========" << delta_q_null.transpose()<<std::endl;
    // 将解投影到零空间，确保不影响笛卡尔空间的末端位置
    delta_q_null = null_space_projection * delta_q_null;

    return delta_q_null;
}

    void ArmController::jointStateCallback_left(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        joint_positions_L = msg->position; // 保存 joint position   
    }
    void ArmController::jointStateCallback_right_teleoperation(const sensor_msgs::JointState::ConstPtr& msg) 
    {      
        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            // desired_joint_R[i]=temp_dq[i]-joint_positions_R[i];
            desired_joint_R[i]=temp_dq[i];

            /* code */
        }
        
  
    }
    void ArmController::jointStateCallback_left_teleoperation(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            desired_joint_L[i]=temp_dq[i]-joint_positions_L[i];
            /* code */
        }
   
    }
    void ArmController::jointStateCallback_right(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        joint_positions_R = msg->position;
        
    }

    void ArmController::positionCallback_left(const geometry_msgs::Pose::ConstPtr& msg) {
        // 将接收到的位置数据传递给滤波器
        std::array<double, 3> input_position = {msg->position.x, msg->position.y, msg->position.z};
        auto filtered_position = filter_left.filter(input_position);

        // 更新 xx, yy, zz 为滤波后的值
        xx_left = filtered_position[0];
        yy_left = filtered_position[1];
        zz_left = filtered_position[2];
        // xx_left = msg->position.x;
        // yy_left = msg->position.y;
        // zz_left = msg->position.z;
        qx_left = msg->orientation.x;
        qy_left = msg->orientation.y;
        qz_left = msg->orientation.z;
        qw_left = msg->orientation.w;
    }
    void ArmController::positionCallback_right(const geometry_msgs::Pose::ConstPtr& msg) {
        // 将接收到的位置数据传递给滤波器
        std::array<double, 3> input_position = {msg->position.x, msg->position.y, msg->position.z};
        auto filtered_position = filter_right.filter(input_position);
        xx_right = filtered_position[0];
        yy_right = filtered_position[1];
        zz_right = filtered_position[2];
        qx_right = msg->orientation.x;
        qy_right = msg->orientation.y;
        qz_right = msg->orientation.z;
        qw_right = msg->orientation.w;
    }
   
    Eigen::Matrix4d ArmController::Quat2T_matrix(const std::vector<double>& quat) {
        Eigen::Vector3d posValue;
        Eigen::Quaterniond quattt;

        for (int i = 0; i < 3; i++) {
            posValue(i) = quat[i];
        }

        quattt.x() = quat[3];
        quattt.y() = quat[4];
        quattt.z() = quat[5];
        quattt.w() = quat[6];

        Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
        T.block<3, 3>(0, 0) = quattt.toRotationMatrix();
        T.block<3, 1>(0, 3) = posValue;

        return T;
    }
    void ArmController::first() 
    {
        ROS_INFO("=======first========");

        
        arm_param_left.name = "left";
        arm_param_left.base_link = "base_link";
        arm_param_left.eef_link = "left_flange";
        arm_param_left.joints_min ={-3.1415926, -0.2, -2.268928, -1.779, -2.268928, -0.6454, -0.6454};
        arm_param_left.joints_max ={0.7853981, 1.7453292, 2.2689280, 0, 2.268928, 0.6454, 0.6454};

        arm_param_right.name = "right";
        arm_param_right.base_link = "base_link";
        arm_param_right.eef_link = "right_flange";
        arm_param_right.joints_min ={-0.7853981 ,-1.7453292,-2.2689280,0        , -2.268928, -0.6454, -0.6454};
        arm_param_right.joints_max ={3.1415926  ,0.2         , 2.268928 ,1.779 ,  2.268928,  0.6454,  0.6454};



        // L_grasp_pose = {0.352496-0.035, 0.162388 + 0.015, 0.746278 - 0.01, 0.56662673, 0.48369656, -0.04194205, 0.66574217};
        // R_grasp_pose = {0.354662, -0.109894, 0.14731, 0.7415988, -0.01539735, 0.21900877, -0.63390008};
        // L_grasp_pose = { 0.505169,0.323111,0.249817,0.0872157,0.631405,-0.11521,0.761871 };
        // R_grasp_pose = { 0.50705,-0.317131,0. ,0.702849,0.108244,0.695218,-0.104683};
        // L_grasp_pose = { 0.405169,0.323111,0.249817,0.0872157,0.631405,-0.11521,0.76187};
                                                   
        L_grasp_pose = { 0.505169,0.323111,0.249817,0.303926,0.884509,-0.0545777,0.349706 };
        R_grasp_pose = { 0.50705,-0.317131,0.249316,0.702849,0.108244,0.695218,-0.104683};
        left_grasp_pose.hand_move_enable = 1;
        left_grasp_pose.hand_side = 0;
        left_grasp_pose.hand_reset = 1;
        left_grasp_pose.pose_req.position.x = L_grasp_pose[0];
        left_grasp_pose.pose_req.position.y = L_grasp_pose[1];
        left_grasp_pose.pose_req.position.z = L_grasp_pose[2];
        left_grasp_pose.pose_req.orientation.x = L_grasp_pose[3];
        left_grasp_pose.pose_req.orientation.y = L_grasp_pose[4];
        left_grasp_pose.pose_req.orientation.z = L_grasp_pose[5];
        left_grasp_pose.pose_req.orientation.w = L_grasp_pose[6];

        right_grasp_pose.hand_move_enable = 1;
        right_grasp_pose.hand_side = 1;
        right_grasp_pose.hand_reset = 1;        
        right_grasp_pose.pose_req.position.x = R_grasp_pose[0]; 
        right_grasp_pose.pose_req.position.y = R_grasp_pose[1];
        right_grasp_pose.pose_req.position.z = R_grasp_pose[2];
        right_grasp_pose.pose_req.orientation.x = R_grasp_pose[3];
        right_grasp_pose.pose_req.orientation.y = R_grasp_pose[4];
        right_grasp_pose.pose_req.orientation.z = R_grasp_pose[5];
        right_grasp_pose.pose_req.orientation.w = R_grasp_pose[6];
        
        // sleep(1);
        // left_arm_pub_.publish(left_grasp_pose);

        // right_arm_pub_.publish(right_grasp_pose); // 如果需要右手抓握位置，可以取消注释
        // sleep(1);

        // // 更新左抓握位置
        // updateGraspPoses(left_grasp_pose, L_grasp_pose);
        // // 更新右抓握位置
        // updateGraspPoses(right_grasp_pose, R_grasp_pose);
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.name.resize(7);
        joint_state_msg.position.resize(7);
        joint_state_msg.velocity.resize(7);
        
        // // Set joint names
        joint_state_msg.name = 
        {
            "left_joint1", "left_joint2", "left_joint3",
            "left_joint4", "left_joint5", "left_joint6", "left_joint7"
        };
        for (int i = 0; i < 7; i++)
        {
            joint_state_msg.position[i]=(left_max[i]+left_min[i])/2;
            joint_positions_L[i]=0.5*(left_max[i]+left_min[i]);
        }
                 joint_state_msg.header.stamp = ros::Time::now();

        
        // joint_state_msg.header.stamp = ros::Time::now();
        
        joint_pub.publish(joint_state_msg);

    }
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test_arm");
    ros::NodeHandle nh;

    ArmController arm_controller(nh);

    return 0;
}

    