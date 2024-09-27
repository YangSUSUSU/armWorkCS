// #include <ros/ros.h>
// #include <geometry_msgs/Wrench.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <array>

// class LowPassFilter {
// public:
//     LowPassFilter(double alpha) : alpha_(alpha) {
//         filtered_force_.fill(0.0);  // 初始化数组为零
//     }

//     std::array<double, 3> filter(const std::array<double, 3>& input_force) {
//         for (size_t i = 0; i < 3; ++i) 
//         {
//         double clamped_force = input_force[i];
//         // 限制输入力的绝对值不超过 8
//         if (std::abs(clamped_force) > 20) 
//         {
//             clamped_force = (clamped_force > 0) ? 20 : -20;
//         }
//             filtered_force_[i] = alpha_ * clamped_force + (1.0 - alpha_) * filtered_force_[i];
//         }
//         return filtered_force_;
//     }

// private:
//     double alpha_;
//     std::array<double, 3> filtered_force_;  // 使用 std::array
// };

// class ForceFilterNode {
// public:
//     ForceFilterNode(ros::NodeHandle& nh) 
//         : filter_(0.5) {  // 设置滤波系数
//         sub_ = nh.subscribe("human_arm_6dof_left", 10, &ForceFilterNode::forceCallback, this);
//         //sub_ = nh.subscribe("noisy_force", 10, &ForceFilterNode::forceCallback, this);

//         pub_ = nh.advertise<geometry_msgs::WrenchStamped>("filtered_force", 10);
//     }
//     // 订阅真实传感器数据
//     // ros::Subscriber sub = nh.subscribe<geometry_msgs::WrenchStamped>(
//     //     "/human_arm_6dof_left", 10,
//     //     boost::bind(forceSensorCallback, _1, boost::ref(controller), dt));
//     void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
//         std::array<double, 3> input_force = 
//         {
//             msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z
//         };

//         auto filtered_force = filter_.filter(input_force);

//         geometry_msgs::WrenchStamped output_msg;
//         output_msg.header.frame_id = "left_flange"; // 设置坐标系
//         output_msg.header.stamp = ros::Time::now(); // 设置时间戳    
//         output_msg.wrench.force.x = filtered_force[0];
//         output_msg.wrench.force.y = filtered_force[1];
//         output_msg.wrench.force.z = filtered_force[2];
//         // std::cout<<filtered_force[0]<<", "<<filtered_force[1]<<", "<<filtered_force[2]<<std::endl;  // 打印滤波后的数据
//         pub_.publish(output_msg);
//     }

// private:
//     ros::Subscriber sub_;
//     ros::Publisher pub_;
//     LowPassFilter filter_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "force_filter");
//     ros::NodeHandle nh;
//     // ros::Rate rate(200);  // 发布频率
//     // while (ros::ok()) {
//     //     generator.publishForce();
//     //     ros::spinOnce();
//     //     rate.sleep();
//     // }

//     ForceFilterNode filter_node(nh);
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <array>
#include <vector>

class SmoothingFilter {
public:
    SmoothingFilter(size_t windowSize) : windowSize(windowSize), index(0) {
        values.resize(windowSize, {0.0, 0.0, 0.0}); // 初始化数组
    }

    std::array<double, 3> filter(const std::array<double, 3>& input_force) {
        values[index] = input_force;

        std::array<double, 3> smoothed_value = {0.0, 0.0, 0.0};
        for (const auto& val : values) {
            smoothed_value[0] += val[0];
            smoothed_value[1] += val[1];
            smoothed_value[2] += val[2];
        }

        smoothed_value[0] /= windowSize;
        smoothed_value[1] /= windowSize;
        smoothed_value[2] /= windowSize;

        index = (index + 1) % windowSize; // 更新索引
        return smoothed_value;
    }

private:
    size_t windowSize; // 窗口大小
    std::vector<std::array<double, 3>> values; // 存储最近的值
    int index; // 当前索引
};

class ForceFilterNode {
public:
    ForceFilterNode(ros::NodeHandle& nh) 
        : filter_(200) {  // 设置窗口大小为5
        // sub_ = nh.subscribe("human_arm_6dof_left", 10, &ForceFilterNode::forceCallback, this);
        sub_ = nh.subscribe("noisy_force", 10, &ForceFilterNode::forceCallback, this);
        pub_ = nh.advertise<geometry_msgs::WrenchStamped>("filtered_force", 10);
    }

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        std::array<double, 3> input_force = 
        {
            msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z
        };

        auto filtered_force = filter_.filter(input_force);

        geometry_msgs::WrenchStamped output_msg;
        output_msg.header.frame_id = "left_flange"; // 设置坐标系
        output_msg.header.stamp = ros::Time::now(); // 设置时间戳    
        output_msg.wrench.force.x = filtered_force[0];
        output_msg.wrench.force.y = filtered_force[1];
        output_msg.wrench.force.z = filtered_force[2];
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

