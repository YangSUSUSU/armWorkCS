#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <array>
#include <vector>

class SmoothingFilter {
public:
    SmoothingFilter(size_t windowSize) : windowSize(windowSize), index(0) {
        values.resize(windowSize, {0.0,0.0,0.0,0.0, 0.0, 0.0}); // 初始化数组
    }

    std::array<double, 6> filter(const std::array<double, 6>& input_force) {
        values[index] = input_force;

        std::array<double, 6> smoothed_value = {0.0, 0.0, 0.0 , 0, 0, 0};
        for (const auto& val : values) {
            smoothed_value[0] += val[0];
            smoothed_value[1] += val[1];
            smoothed_value[2] += val[2];
            smoothed_value[3] += val[3];
            smoothed_value[4] += val[4];
            smoothed_value[5] += val[5];
        }

        smoothed_value[0] /= windowSize;
        smoothed_value[1] /= windowSize;
        smoothed_value[2] /= windowSize;
        smoothed_value[3] /= windowSize;
        smoothed_value[4] /= windowSize;
        smoothed_value[5] /= windowSize;
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
        : filter_(30) {  // 设置窗口大小为5
        // sub_ = nh.subscribe("human_arm_6dof_left", 10, &ForceFilterNode::forceCallback, this);
        sub_ = nh.subscribe("noisy_force", 10, &ForceFilterNode::forceCallback, this);
        pub_ = nh.advertise<geometry_msgs::WrenchStamped>("filtered_force", 10);
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        std::array<double, 6> input_force = 
        {
            msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
            msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z
        };

        auto filtered_force = filter_.filter(input_force);

        geometry_msgs::WrenchStamped output_msg;
        output_msg.header.frame_id = "left_flange"; // 设置坐标系
        output_msg.header.stamp = ros::Time::now(); // 设置时间戳    
        output_msg.wrench.force.x = filtered_force[0];
        output_msg.wrench.force.y = filtered_force[1];
        output_msg.wrench.force.z = filtered_force[2];
        output_msg.wrench.torque.x = 0.001*filtered_force[3];
        output_msg.wrench.torque.y = 0.001*filtered_force[4];
        output_msg.wrench.torque.z = 0.001*filtered_force[5];
        pub_.publish(output_msg);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    SmoothingFilter filter_; // 使用平滑滤波器
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_filter");
    ros::NodeHandle nh;
    ForceFilterNode filter_node(nh);
    ros::spin();
    return 0;
}
