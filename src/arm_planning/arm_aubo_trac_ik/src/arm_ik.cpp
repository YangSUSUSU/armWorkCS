// #include "ros/ros.h"
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>
// #include <llm_msgs/hand_pose_req.h>
// #include <sensor_msgs/JointState.h>

// using namespace KDL;

// class ArmKinematics {
// public:
//     ArmKinematics(ros::NodeHandle& nh) {
//         nh.param("chain_start", chain_start, std::string("base_link"));
//         nh.param("chain_end", chain_end, std::string("left_flange"));
//         nh.param("urdf_param", urdf_param, std::string("/robot_description"));
//         nh.param("timeout", timeout, 0.004);
        
//         ik_solver = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, 1e-6, TRAC_IK::Distance);
        
//         if (!ik_solver->getKDLChain(chain)) {
//             ROS_ERROR("No valid KDL chain found");
//             ros::shutdown();
//         }

//         ik_solver->getKDLLimits(ll, ul);
//         fk_solver = new KDL::ChainFkSolverPos_recursive(chain);

//         pose_sub = nh.subscribe("/left_arm_pose1_req", 10, &ArmKinematics::poseCallback, this);
//         joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

//         // Initialize joint_seed to the midpoint between joint limits
//         joint_seed.resize(chain.getNrOfJoints());
//         for (unsigned int i = 0; i < joint_seed.rows(); ++i) {
//             joint_seed(i) = (ll(i) + ul(i)) / 2.0;
//         }
//     }

//     void poseCallback(const llm_msgs::hand_pose_req::ConstPtr& msg) {
//         KDL::Vector position(msg->pose_req.position.x, msg->pose_req.position.y, msg->pose_req.position.z);
//         KDL::Rotation rotation = KDL::Rotation::Quaternion(
//             msg->pose_req.orientation.x,
//             msg->pose_req.orientation.y,
//             msg->pose_req.orientation.z,
//             msg->pose_req.orientation.w
//         );
//         KDL::Frame target_pose(rotation, position);

//         KDL::JntArray result(joint_seed); // Use the current joint_seed

//         int rc = ik_solver->CartToJnt(joint_seed, target_pose, result);

//         if (rc >= 0) {
//             // Update joint_seed to the current result
//             joint_seed = result;

//             sensor_msgs::JointState joint_state_msg;
//             joint_state_msg.name.resize(7);
//             joint_state_msg.position.resize(7);
//             joint_state_msg.velocity.resize(7);
            
//             // Set joint names
//             joint_state_msg.name = {
//                 "left_joint1", "left_joint2", "left_joint3",
//                 "left_joint4", "left_joint5", "left_joint6", "left_joint7"
//             };

//             for (unsigned int i = 0; i < result.rows(); i++) {
//                 joint_state_msg.position[i] = result(i);
//             }
//             joint_state_msg.header.stamp = ros::Time::now();

//             joint_pub.publish(joint_state_msg);
//             ROS_INFO("Published joint angles");
//         } else {
//             ROS_WARN("IK failed for the given pose");
//         }
//     }

// private:
//     TRAC_IK::TRAC_IK* ik_solver;
//     KDL::Chain chain;
//     KDL::ChainFkSolverPos_recursive* fk_solver;
//     KDL::JntArray ll, ul; // Joint limits
//     KDL::JntArray joint_seed; // Joint seed
//     std::string chain_start, chain_end, urdf_param;
//     double timeout;

//     ros::Subscriber pose_sub;
//     ros::Publisher joint_pub;
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "arm_ik");
//     ros::NodeHandle nh("~");
//     ArmKinematics arm_kinematics(nh);
//     ros::spin();
//     return 0;
// }
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class JointStateForwarder {
public:
    JointStateForwarder(ros::NodeHandle& nh) {
        // 订阅 joint_states111
        joint_state_sub = nh.subscribe("/joint_1111states", 10, &JointStateForwarder::jointStateCallback, this);

        // 发布到 joint_states
        joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

private:
    ros::Subscriber joint_state_sub;
    ros::Publisher joint_pub;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 发布接收到的 joint_states111 到 joint_states
        joint_pub.publish(*msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_state_forwarder");
    ros::NodeHandle nh;

    JointStateForwarder forwarder(nh);

    ros::spin(); // 保持节点运行并处理回调
    return 0;
}
