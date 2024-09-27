#include <ros/ros.h>
#include "llm_msgs/set_angle.h"
#include "llm_msgs/set_angleRequest.h"
#include "llm_msgs/set_angleResponse.h"

class GripperClient
{
public:
    GripperClient()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle();

        // Initialize service clients
        client_set_left_gripper_ = nh_.serviceClient<llm_msgs::set_angle>("/inspire_hand/set_angle/left_hand");
        client_set_right_gripper_ = nh_.serviceClient<llm_msgs::set_angle>("/inspire_hand/set_angle/right_hand");

        // Wait for the services to be available
        ros::service::waitForService("/inspire_hand/set_angle/left_hand");
        ros::service::waitForService("/inspire_hand/set_angle/right_hand");

        // Load angles from parameter server
        loadAnglesFromParams();
    }

    void setGripperAngles()
    {
        llm_msgs::set_angle srv_set_angle_left;
        llm_msgs::set_angle srv_set_angle_right;

        // Set left gripper angles
        srv_set_angle_left.request.angle0Ratio = left_gripper_[0];
        srv_set_angle_left.request.angle1Ratio = left_gripper_[1];
        srv_set_angle_left.request.angle2Ratio = left_gripper_[2];
        srv_set_angle_left.request.angle3Ratio = left_gripper_[3];
        srv_set_angle_left.request.angle4Ratio = left_gripper_[4];
        srv_set_angle_left.request.angle5Ratio = left_gripper_[5];

        // Set right gripper angles
        srv_set_angle_right.request.angle0Ratio = right_gripper_[0];
        srv_set_angle_right.request.angle1Ratio = right_gripper_[1];
        srv_set_angle_right.request.angle2Ratio = right_gripper_[2];
        srv_set_angle_right.request.angle3Ratio = right_gripper_[3];
        srv_set_angle_right.request.angle4Ratio = right_gripper_[4];
        srv_set_angle_right.request.angle5Ratio = right_gripper_[5];

        // Call services
        if (client_set_left_gripper_.call(srv_set_angle_left) && client_set_right_gripper_.call(srv_set_angle_right))
        {
            if (srv_set_angle_left.response.angle_accepted && srv_set_angle_right.response.angle_accepted)
            {
                ROS_INFO("Angles accepted!!");
            }
            else
            {
                ROS_INFO("Angles not accepted!!!");
            }
            ROS_INFO("Set angles are OK!!");
        }
        else
        {
            ROS_INFO("Set angles failed!!");
        }
    }

private:
    void loadAnglesFromParams()
    {
        nh_.getParam("left_gripper", left_gripper_);
        nh_.getParam("right_gripper", right_gripper_);
    }

    ros::NodeHandle nh_;
    ros::ServiceClient client_set_left_gripper_;
    ros::ServiceClient client_set_right_gripper_;
    std::vector<double> left_gripper_;
    std::vector<double> right_gripper_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_client");

    GripperClient gripper_client;

    gripper_client.setGripperAngles();

    return 0;
}
