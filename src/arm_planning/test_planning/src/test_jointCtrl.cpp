#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <Eigen/Dense> 

class JointInterpolator {
public:

    //关节状态
    Eigen::VectorXd qr_r;
    Eigen::VectorXd qr_l;

    Eigen::VectorXd qa_r;
    Eigen::VectorXd qa_l;

    ros::Subscriber sub_qr_r;
    ros::Subscriber sub_qr_l;

    ros::Subscriber sub_qa_r;
    ros::Subscriber sub_qa_l;

    ros::Publisher cmd_pub_r;
    ros::Publisher cmd_pub_l;
    bool geTqrR = false;
    bool geTqrL = false;

    bool init_qaR = false;
    bool init_qaL = false;

    int step_L = 0;
    int step_R = 0;
    // Eigen::VectorXd

    JointInterpolator(ros::NodeHandle& nh)
    { 
        qr_r = Eigen::VectorXd::Zero(7);
        qr_l = Eigen::VectorXd::Zero(7);
        qa_r = Eigen::VectorXd::Zero(7);
        qa_l = Eigen::VectorXd::Zero(7);

        // sub_qa_r = nh.subscribe("/human_arm_state_right", 10, &JointInterpolator::qaCB_r, this);
        // sub_qa_l = nh.subscribe("/human_arm_state_left" , 10, &JointInterpolator::qaCB_l, this);
        sub_qa_r = nh.subscribe("/joint_states", 10, &JointInterpolator::qaCB_r, this);
        sub_qa_l = nh.subscribe("/joint_states" , 10, &JointInterpolator::qaCB_l, this);
        sub_qr_r = nh.subscribe("/human_arm_ctrl_right_nikoo", 10, &JointInterpolator::qrCB_r, this);
        sub_qr_l = nh.subscribe( "/human_arm_ctrl_left_nikoo" , 10, &JointInterpolator::qrCB_l, this);

        // cmd_pub_l  = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_left", 10);
        // cmd_pub_r  = nh.advertise<sensor_msgs::JointState>("/human_arm_ctrl_right", 10);
        cmd_pub_l  = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        cmd_pub_r  = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }
    
    void Joints_pub(Eigen::VectorXd& cmd, ros::Publisher pub) 
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
            joint_state.position[i] = cmd(i);
        }
        pub.publish(joint_state);
    }
    //======================右臂 期望回调
    void qrCB_r(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        std::cout<<"==========ok========="<<std::endl;
        geTqrR = true;

        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            qr_r(i) = temp_dq[i];
        }
    }
    //======================左臂 期望回调
    void qrCB_l(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            qr_l(i) = temp_dq[i];
        }
    }

    //======================右臂 当前回调
    void qaCB_r(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        
        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            qa_r(i) = temp_dq[10+i];
        }
        std::cout<<"======"<<qa_r.transpose()<<std::endl;
    }
    //======================左臂 当前回调
    void qaCB_l(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        auto temp_dq = msg->position;
        for (int i = 0; i < 7; i++)
        {
            qa_l(i) = temp_dq[3+i];
        }
    }


private:
    
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_interpolator");
    ros::NodeHandle nh;

    JointInterpolator interpolator(nh);
    ros::Rate loop_rate(200);

    while (ros::ok()) 
    {

        if (interpolator.geTqrR)
        {
            if (interpolator.step_R<800)
            {
                Eigen::VectorXd cmdR = interpolator.qa_r + 0.00125 * (interpolator.qr_r - interpolator.qa_r);
                std::cout<<cmdR.transpose()<<std::endl;
                sensor_msgs::JointState joint_state;
                joint_state.name.resize(17); 
                joint_state.position.resize(17);
                joint_state.velocity.resize(17);
                joint_state.effort.resize(17);
                joint_state.header.stamp = ros::Time::now();
                joint_state.name = 
                {
                    "waist_Z_joint",
                    "waist_roll_joint",
                    // "waist_pitch_joint",
                    "waist_yaw_joint",
                    "left_joint1",
                    "left_joint2",
                    "left_joint3",
                    "left_joint4",
                    "left_joint5",
                    "left_joint6",
                    "left_joint7",
                    "right_joint1",
                    "right_joint2",
                    "right_joint3",
                    "right_joint4",
                    "right_joint5",
                    "right_joint6",
                    "right_joint7"};
                    for (int i = 0; i < 7; ++i) 
                    {
                        joint_state.position[i+10] = cmdR(i);
                    }
                interpolator.cmd_pub_r.publish(joint_state);

                // interpolator.Joints_pub(cmdR,interpolator.cmd_pub_r); 
            }
            else
            {
                interpolator.geTqrR = false;
            }
            interpolator.step_R++;
        }
        
        if (interpolator.geTqrL)
        {
            if (interpolator.step_L<800)
            {
                Eigen::VectorXd cmdL = interpolator.qa_l + 0.00125 * (interpolator.qr_l - interpolator.qa_l);
                interpolator.Joints_pub(cmdL,interpolator.cmd_pub_l); 
            }
            else
            {
                interpolator.geTqrL = false;
            }
            interpolator.step_L++;
        }


        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
