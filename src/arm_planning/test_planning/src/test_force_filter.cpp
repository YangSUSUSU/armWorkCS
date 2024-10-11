#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <array>
#include <vector>
// #define SIM
class SmoothingFilter {
public:
    SmoothingFilter(size_t windowSize) : windowSize(windowSize), index(0) {
        values.resize(windowSize, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); // 初始化数组
    }

    std::array<double, 6> filter(const std::array<double, 6>& input_force) {
        values[index] = input_force;

        std::array<double, 6> smoothed_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        for (const auto& val : values) {
            smoothed_value[0] += val[0];
            smoothed_value[1] += val[1];
            smoothed_value[2] += val[2];
            smoothed_value[3] += val[3];
            smoothed_value[4] += val[4];
            smoothed_value[5] += val[5];
        }

        for (size_t i = 0; i < 6; ++i) {
            smoothed_value[i] /= windowSize; // 平均
        }

        index = (index + 1) % windowSize; // 更新索引
        return smoothed_value;
    }

private:
    size_t windowSize; // 窗口大小
    std::vector<std::array<double, 6>> values; // 存储最近的值
    int index; // 当前索引
};

class ForceFilterNode {
public:
    ForceFilterNode(ros::NodeHandle& nh)
        : filter_left_(30), filter_right_(30) {  // 为两个传感器设置滤波器窗口大小

        #ifdef SIMULATION_MODE
                sub_left_ = nh.subscribe("noisy_force_left", 10, &ForceFilterNode::leftForceCallback, this);
                sub_right_ = nh.subscribe("noisy_force_right", 10, &ForceFilterNode::rightForceCallback, this);
        #else
                sub_left_ = nh.subscribe("human_arm_6dof_left", 10, &ForceFilterNode::leftForceCallback, this);
                sub_right_ = nh.subscribe("human_arm_6dof_right", 10, &ForceFilterNode::rightForceCallback, this);
        #endif

        pub_left_ = nh.advertise<geometry_msgs::WrenchStamped>("filtered_force_left", 10);
        pub_right_ = nh.advertise<geometry_msgs::WrenchStamped>("filtered_force_right", 10);
    }

    void leftForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        std::array<double, 6> input_force = {
            msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
            msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z
        };

        auto filtered_force = filter_left_.filter(input_force);

        geometry_msgs::WrenchStamped output_msg;
        output_msg.header.frame_id = "left_flange"; // 设置坐标系
        output_msg.header.stamp = ros::Time::now(); // 设置时间戳    
        output_msg.wrench.force.x = filtered_force[0];
        output_msg.wrench.force.y = filtered_force[1];
        output_msg.wrench.force.z = filtered_force[2];
        output_msg.wrench.torque.x = 0.001 * filtered_force[3];
        output_msg.wrench.torque.y = 0.001 * filtered_force[4];
        output_msg.wrench.torque.z = 0.001 * filtered_force[5];
        pub_left_.publish(output_msg);
    }

    void rightForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        std::array<double, 6> input_force = {
            msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
            msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z
        };

        auto filtered_force = filter_right_.filter(input_force);

        geometry_msgs::WrenchStamped output_msg;
        output_msg.header.frame_id = "right_flange"; // 设置坐标系
        output_msg.header.stamp = ros::Time::now(); // 设置时间戳    
        output_msg.wrench.force.x = filtered_force[0];
        output_msg.wrench.force.y = filtered_force[1];
        output_msg.wrench.force.z = filtered_force[2];
        output_msg.wrench.torque.x = 0.001 * filtered_force[3];
        output_msg.wrench.torque.y = 0.001 * filtered_force[4];
        output_msg.wrench.torque.z = 0.001 * filtered_force[5];
        pub_right_.publish(output_msg);
    }

private:
    ros::Subscriber sub_left_;
    ros::Subscriber sub_right_;
    ros::Publisher pub_left_;
    ros::Publisher pub_right_;
    SmoothingFilter filter_left_;  // 左侧滤波器
    SmoothingFilter filter_right_; // 右侧滤波器
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_filter");
    ros::NodeHandle nh;
    ForceFilterNode filter_node(nh);
    ros::spin();
    return 0;
}
