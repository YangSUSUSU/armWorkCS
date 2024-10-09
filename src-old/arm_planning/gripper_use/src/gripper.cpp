#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "llm_msgs/set_angle.h"
#include "llm_msgs/set_angleRequest.h"
#include "llm_msgs/set_angleResponse.h"
#include "llm_msgs/set_speed.h"
#include "llm_msgs/set_speedRequest.h"
#include "llm_msgs/set_speedResponse.h"
#include "llm_msgs/set_force.h"
#include "llm_msgs/set_forceRequest.h"
#include "llm_msgs/set_forceResponse.h"

#include "llm_msgs/get_angle_act.h"
#include "llm_msgs/get_angle_actResponse.h"
#include "llm_msgs/hand_pose_req.h"
#include "llm_msgs/pose_action_status.h"

class GripperClient
{
public:
    GripperClient()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle();

	// Initialize topic subscriber
	gripper_cmd_sub_ = nh_.subscribe("/hand_pose_req", 10, &GripperClient::GripperCmdCallback, this);
	gripper_status_pub_ = nh_.advertise<llm_msgs::pose_action_status>("/pose_action_status", 10);

        // Initialize service clients
        client_set_left_gripper_ = nh_.serviceClient<llm_msgs::set_angle>("/inspire_hand/set_angle/left_hand");
        client_set_right_gripper_ = nh_.serviceClient<llm_msgs::set_angle>("/inspire_hand/set_angle/right_hand");
        client_set_left_gripper_spd_ = nh_.serviceClient<llm_msgs::set_speed>("/inspire_hand/set_speed/left_hand");
        client_set_right_gripper_spd_ = nh_.serviceClient<llm_msgs::set_speed>("/inspire_hand/set_speed/right_hand");
        client_set_left_gripper_force_ = nh_.serviceClient<llm_msgs::set_force>("/inspire_hand/set_force/left_hand");
        client_set_right_gripper_force_ = nh_.serviceClient<llm_msgs::set_force>("/inspire_hand/set_force/right_hand");
      
      	client_get_left_gripper_ = nh_.serviceClient<llm_msgs::get_angle_act>("/inspire_hand/get_angle/left_hand");
        client_get_right_gripper_ = nh_.serviceClient<llm_msgs::get_angle_act>("/inspire_hand/get_angle/right_hand");

       // Wait for the services to be available
        ros::service::waitForService("/inspire_hand/set_angle/left_hand");
        ros::service::waitForService("/inspire_hand/set_angle/right_hand");
	ros::service::waitForService("/inspire_hand/get_angle/left_hand");
        ros::service::waitForService("/inspire_hand/get_angle/right_hand");

        // Load angles from parameter server
        // loadAnglesFromParams();
    }

    void GripperCmdCallback(const llm_msgs::hand_pose_req::ConstPtr& msg)
    {
	std::cout << "grasp msg recv " << std::endl;
    	if (msg->gripper_move_enable == 1)
	{
		if (msg->gripper_side == 0)
		{
			left_gripper_cmd_[0] = msg->little_finger;
			left_gripper_cmd_[1] = msg->ring_finger;
			left_gripper_cmd_[2] = msg->middle_finger;
			left_gripper_cmd_[3] = msg->index_finger;
			left_gripper_cmd_[4] = msg->thumb_bending;
			left_gripper_cmd_[5] = msg->thumb_rotating;
			
			recv_new_left_cmd_ = true;
			left_cmd_type_ = msg->finger_status;
			left_cmd_recv_time_ = ros::Time::now();
		}
		else if (msg->gripper_side == 1)
		{
			right_gripper_cmd_[0] = msg->little_finger;
			right_gripper_cmd_[1] = msg->ring_finger;
			right_gripper_cmd_[2] = msg->middle_finger;
			right_gripper_cmd_[3] = msg->index_finger;
			right_gripper_cmd_[4] = msg->thumb_bending;
			right_gripper_cmd_[5] = msg->thumb_rotating;			
		
			recv_new_right_cmd_ = true;
			right_cmd_type_ = msg->finger_status;
			right_cmd_recv_time_ = ros::Time::now();
		}
		else if (msg->gripper_side == 2)
		{
			std::cout << "Double hand not support" << std::endl;
		}
	}	

    }

    
    void LeftGripperExecuteSuccess()
    {
	std::cout << "In LeftGripperExecuteSuccess" << std::endl;

    	if (left_cmd_processed_)
	{
		left_cmd_processed_ = false;
	
		ros::Time entry_time = ros::Time::now();
		llm_msgs::get_angle_act srv_get_angle_left;		
		
		bool success = true;
		llm_msgs::pose_action_status status_msg;
		ros::Rate loop_rate(10);
		bool get_finger_mag_first = false;
	        double prev_middle_finger_mag, prev_index_finger_mag;

		bool msg_has_sent = false; 
		while (ros::Time::now() - entry_time < ros::Duration(2.))
		{
			if (client_get_left_gripper_.call(srv_get_angle_left))
			{

				double cur_middle_finger_mag = srv_get_angle_left.response.curangleRatio[2];
				double cur_index_finger_mag = srv_get_angle_left.response.curangleRatio[3];
				if (!get_finger_mag_first)
				{
					get_finger_mag_first = true;
					prev_middle_finger_mag = srv_get_angle_left.response.curangleRatio[2];
					prev_index_finger_mag = srv_get_angle_left.response.curangleRatio[3];
				}

				for (int i = 0; i < 6; i ++ )
				{
					std::cout << srv_get_angle_left.response.curangleRatio[i] << ' ' ;

				}
				std::cout << std::endl;
				
				std::cout << std::fabs(srv_get_angle_left.response.curangleRatio[2] - left_gripper_cmd_[2]) << "   " << std::fabs(srv_get_angle_left.response.curangleRatio[3] - left_gripper_cmd_[3]) << std::endl;


				// detecting logic for closing fingers
				// 当前逻辑的漏洞：当夹取面包等物体时较软，指令与状态间差值较小<0.01，会误触发以下逻辑
				if (left_cmd_type_ && (std::fabs(srv_get_angle_left.response.curangleRatio[2] - left_gripper_cmd_[2]) < 0.01) && (std::fabs(srv_get_angle_left.response.curangleRatio[3] - left_gripper_cmd_[3]) < 0.01))
				{
					std::cout << "1" << std::endl;
					success = false;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 0;
					gripper_status_pub_.publish(status_msg);
					break;
				}
				if (left_cmd_type_ && (std::fabs(cur_middle_finger_mag - prev_middle_finger_mag) < 0.005) && (std::fabs(cur_index_finger_mag - prev_index_finger_mag) < 0.005) && (cur_middle_finger_mag - prev_middle_finger_mag > 0.05) && (cur_index_finger_mag - prev_middle_finger_mag >0.05))
				{

					std::cout << "2" << std::endl;
					success = true;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 1;
					gripper_status_pub_.publish(status_msg);
					break;
				}

				// detecting logic for opening fingers
				if (!left_cmd_type_ && (std::fabs(srv_get_angle_left.response.curangleRatio[2] - left_gripper_cmd_[2]) < 0.01) && (std::fabs(srv_get_angle_left.response.curangleRatio[3] - left_gripper_cmd_[3]) < 0.01))
				{

					std::cout << "3" << std::endl;
					success = true;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 1;
					gripper_status_pub_.publish(status_msg);
					break;
				}
                                
	        		prev_middle_finger_mag = cur_middle_finger_mag;
	        		prev_index_finger_mag = cur_index_finger_mag;


			}	


			loop_rate.sleep();		
		}

		std::cout << "left finger move status: " << success << std::endl;
	}

	std::cout << "Out LeftGripperExecuteSuccess" << std::endl;	
    }



    bool LeftGripperCheckRequired()
    {
    	return left_cmd_processed_;
    }    


    void RightGripperExecuteSuccess()
    {
	std::cout << "In RightGripperExecuteSuccess" << std::endl;

    	if (right_cmd_processed_)
	{
		right_cmd_processed_ = false;
	
		ros::Time entry_time = ros::Time::now();
		llm_msgs::get_angle_act srv_get_angle_right;		
		
		bool success = true;
		llm_msgs::pose_action_status status_msg;
		ros::Rate loop_rate(10);
		bool get_finger_mag_first = false;
	        double prev_middle_finger_mag, prev_index_finger_mag;

		bool msg_has_sent = false; 
		while (ros::Time::now() - entry_time < ros::Duration(2.))
		{
			if (client_get_right_gripper_.call(srv_get_angle_right))
			{

				double cur_middle_finger_mag = srv_get_angle_right.response.curangleRatio[2];
				double cur_index_finger_mag = srv_get_angle_right.response.curangleRatio[3];
				if (!get_finger_mag_first)
				{
					get_finger_mag_first = true;
					prev_middle_finger_mag = srv_get_angle_right.response.curangleRatio[2];
					prev_index_finger_mag = srv_get_angle_right.response.curangleRatio[3];
				}

				for (int i = 0; i < 6; i ++ )
				{
					std::cout << srv_get_angle_right.response.curangleRatio[i] << ' ' ;

				}
				std::cout << std::endl;
				
				std::cout << std::fabs(srv_get_angle_right.response.curangleRatio[2] - right_gripper_cmd_[2]) << "   " << std::fabs(srv_get_angle_right.response.curangleRatio[3] - right_gripper_cmd_[3]) << std::endl;


				// detecting logic for closing fingers
				// 当前逻辑的漏洞：当夹取面包等物体时较软，指令与状态间差值较小<0.01，会误触发以下逻辑
				if (right_cmd_type_ && (std::fabs(srv_get_angle_right.response.curangleRatio[2] - right_gripper_cmd_[2]) < 0.01) && (std::fabs(srv_get_angle_right.response.curangleRatio[3] - right_gripper_cmd_[3]) < 0.01))
				{
					std::cout << "1" << std::endl;
					success = false;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 0;
					gripper_status_pub_.publish(status_msg);
					break;
				}
				if (right_cmd_type_ && (std::fabs(cur_middle_finger_mag - prev_middle_finger_mag) < 0.005) && (std::fabs(cur_index_finger_mag - prev_index_finger_mag) < 0.005) && (cur_middle_finger_mag - prev_middle_finger_mag > 0.05) && (cur_index_finger_mag - prev_middle_finger_mag >0.05))
				{

					std::cout << "2" << std::endl;
					success = true;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 1;
					gripper_status_pub_.publish(status_msg);
					break;
				}

				// detecting logic for opening fingers
				if (!right_cmd_type_ && (std::fabs(srv_get_angle_right.response.curangleRatio[2] - right_gripper_cmd_[2]) < 0.01) && (std::fabs(srv_get_angle_right.response.curangleRatio[3] - right_gripper_cmd_[3]) < 0.01))
				{

					std::cout << "3" << std::endl;
					success = true;
					status_msg.header.stamp = ros::Time::now();
					status_msg.gripper_move_success = 1;
					gripper_status_pub_.publish(status_msg);
					break;
				}
                                
	        		prev_middle_finger_mag = cur_middle_finger_mag;
	        		prev_index_finger_mag = cur_index_finger_mag;


			}	


			loop_rate.sleep();		
		}

		std::cout << "right finger move status: " << success << std::endl;
	}

	std::cout << "Out RightGripperExecuteSuccess" << std::endl;	
    }



    
    bool RightGripperCheckRequired()
    {
    	return right_cmd_processed_;
    }    



    void SetGripperAngles() // hand_side: 0-left, 1-right, 2-double
    {
	// cur_time = ros::Time::now();

	if (recv_new_left_cmd_)
	{
		llm_msgs::set_angle srv_set_angle_left;
		llm_msgs::set_speed srv_set_speed_left;
		llm_msgs::set_force srv_set_force_left;
	
		// Set left gripper angles
        	srv_set_angle_left.request.angle0Ratio = left_gripper_cmd_[0];
        	srv_set_angle_left.request.angle1Ratio = left_gripper_cmd_[1];
        	srv_set_angle_left.request.angle2Ratio = left_gripper_cmd_[2];
        	srv_set_angle_left.request.angle3Ratio = left_gripper_cmd_[3];
        	srv_set_angle_left.request.angle4Ratio = left_gripper_cmd_[4];
        	srv_set_angle_left.request.angle5Ratio = left_gripper_cmd_[5];
	 
		srv_set_force_left.request.force0Ratio = 0.999;
		srv_set_force_left.request.force1Ratio = 0.999;
		srv_set_force_left.request.force2Ratio = 0.999;
		srv_set_force_left.request.force3Ratio = 0.999;
		srv_set_force_left.request.force4Ratio = 0.999;
		srv_set_force_left.request.force5Ratio = 0.999;
		
		if (left_cmd_type_ == 0)
		{
		    srv_set_speed_left.request.speed0Ratio = 0.9;
		    srv_set_speed_left.request.speed1Ratio = 0.9;
		    srv_set_speed_left.request.speed2Ratio = 0.9;
		    srv_set_speed_left.request.speed3Ratio = 0.9;
		    srv_set_speed_left.request.speed4Ratio = 0.9;
		    srv_set_speed_left.request.speed5Ratio = 0.9;
		}
		else
		{
		    srv_set_speed_left.request.speed0Ratio = 0.3;
		    srv_set_speed_left.request.speed1Ratio = 0.3;
		    srv_set_speed_left.request.speed2Ratio = 0.3;
		    srv_set_speed_left.request.speed3Ratio = 0.3;
		    srv_set_speed_left.request.speed4Ratio = 0.3;
		    srv_set_speed_left.request.speed5Ratio = 0.3;
 		}
	 
		// Call services
		int cnt = 3;
		while (cnt -- )
		{
			if (client_set_left_gripper_.call(srv_set_angle_left) && client_set_left_gripper_spd_.call(srv_set_speed_left) && client_set_left_gripper_force_.call(srv_set_force_left))
			{
			    if (srv_set_angle_left.response.angle_accepted)
			    {
				ROS_INFO("Left angle cmd accepted!!");
			    }
			    else
			    {
				ROS_INFO("Left angle not accepted!!!");
			    }
			    ROS_INFO("Set left angle are OK!!");
			}
			else
			{
			    ROS_INFO("Set left angle failed!!");
			}
		}

		recv_new_left_cmd_ = false;
		left_cmd_processed_ = true;	
	}
    

	if (recv_new_right_cmd_)
	{
        	llm_msgs::set_angle srv_set_angle_right;
		llm_msgs::set_speed srv_set_speed_right;
		llm_msgs::set_force srv_set_force_right;
	
		// Set right gripper angles
        	srv_set_angle_right.request.angle0Ratio = right_gripper_cmd_[0];
        	srv_set_angle_right.request.angle1Ratio = right_gripper_cmd_[1];
        	srv_set_angle_right.request.angle2Ratio = right_gripper_cmd_[2];
        	srv_set_angle_right.request.angle3Ratio = right_gripper_cmd_[3];
        	srv_set_angle_right.request.angle4Ratio = right_gripper_cmd_[4];
        	srv_set_angle_right.request.angle5Ratio = right_gripper_cmd_[5];
	 
		srv_set_force_right.request.force0Ratio = 0.999;
		srv_set_force_right.request.force1Ratio = 0.999;
		srv_set_force_right.request.force2Ratio = 0.999;
		srv_set_force_right.request.force3Ratio = 0.999;
		srv_set_force_right.request.force4Ratio = 0.999;
		srv_set_force_right.request.force5Ratio = 0.999;
		
		if (right_cmd_type_ == 0)
		{
		    srv_set_speed_right.request.speed0Ratio = 0.9;
		    srv_set_speed_right.request.speed1Ratio = 0.9;
		    srv_set_speed_right.request.speed2Ratio = 0.9;
		    srv_set_speed_right.request.speed3Ratio = 0.9;
		    srv_set_speed_right.request.speed4Ratio = 0.9;
		    srv_set_speed_right.request.speed5Ratio = 0.9;
		}
		else
		{
		    srv_set_speed_right.request.speed0Ratio = 0.3;
		    srv_set_speed_right.request.speed1Ratio = 0.3;
		    srv_set_speed_right.request.speed2Ratio = 0.3;
		    srv_set_speed_right.request.speed3Ratio = 0.3;
		    srv_set_speed_right.request.speed4Ratio = 0.3;
		    srv_set_speed_right.request.speed5Ratio = 0.3;
 		}
	 
	 
		// Call services
		int cnt = 3;
		while (cnt -- ){
			if (client_set_right_gripper_.call(srv_set_angle_right) && client_set_right_gripper_spd_.call(srv_set_speed_right) && client_set_right_gripper_force_.call(srv_set_force_right))
			{
			    if (srv_set_angle_right.response.angle_accepted)
			    {
				ROS_INFO("Right angle cmd accepted!!");
			    }
			    else
			    {
				ROS_INFO("Right angle cmd not accepted!!!");
			    }
			    ROS_INFO("Set right angle are OK!!");
			}
			else
			{
			    ROS_INFO("Set right angle failed!!");
			}
		}

		recv_new_right_cmd_ = false;
		right_cmd_processed_ = true;
	}
    }

private:
    /*
    void loadAnglesFromParams()
    {
        nh_.getParam("left_gripper_cmd_", left_gripper_cmd_);
        nh_.getParam("right_gripper_cmd_", right_gripper_cmd_);
    }
    */

    ros::NodeHandle nh_;
    ros::Subscriber gripper_cmd_sub_;
    ros::Publisher  gripper_status_pub_;
    ros::ServiceClient client_set_left_gripper_;
    ros::ServiceClient client_set_left_gripper_spd_;
    ros::ServiceClient client_set_left_gripper_force_;
    
    ros::ServiceClient client_set_right_gripper_spd_;
    ros::ServiceClient client_set_right_gripper_force_;
    ros::ServiceClient client_set_right_gripper_;


    ros::ServiceClient client_get_left_gripper_;
    ros::ServiceClient client_get_right_gripper_;

    bool recv_new_left_cmd_, recv_new_right_cmd_;
    bool left_cmd_processed_, right_cmd_processed_;
    bool left_cmd_type_, right_cmd_type_; // 0-打开， 1-闭合 
    ros::Time left_cmd_recv_time_, right_cmd_recv_time_;

    std::array<double, 7> left_gripper_cmd_;
    std::array<double, 7> right_gripper_cmd_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_client");

    GripperClient gripper_client;

    ros::Rate loop_rate(300);

    while (ros::ok())
    {
    	gripper_client.SetGripperAngles();
 	
	if (gripper_client.LeftGripperCheckRequired())
	{
		//std::thread left_pose_status_check([&gripper_client]() { gripper_client.LeftGripperExecuteSuccess(); });
		//left_pose_status_check.detach();
	}	


	if (gripper_client.RightGripperCheckRequired())
	{
		//std::thread right_pose_status_check([&gripper_client]() { gripper_client.RightGripperExecuteSuccess(); });
		//right_pose_status_check.detach();
	}	

	loop_rate.sleep();

	ros::spinOnce();
	// std::cout << "loop ... " << std::endl;
    }

    return 0;
}
