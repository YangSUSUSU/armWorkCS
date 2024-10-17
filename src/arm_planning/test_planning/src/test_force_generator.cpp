#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cstdlib>
#include <ctime>

class ForcePublisher {
public:
    ForcePublisher(ros::NodeHandle& nh) 
    {
        pub_left_ = nh.advertise<geometry_msgs::WrenchStamped>("noisy_force_left", 10);  // 左侧传感器
        pub_right_ = nh.advertise<geometry_msgs::WrenchStamped>("noisy_force_right", 10);  // 右侧传感器
        srand(static_cast<unsigned int>(time(0)));  // 随机数种子
    }

    void publishForce() {
        geometry_msgs::WrenchStamped left_force_;
        geometry_msgs::WrenchStamped right_force_;
        
        double time = ros::Time::now().toSec(); // 获取当前时间

        // 左侧传感器数据
        left_force_.header.frame_id = "left_flange"; // 设置左传感器坐标系
        left_force_.header.stamp = ros::Time::now(); // 设置时间戳    
        left_force_.wrench.force.y = 0.0 + 0.1 * sin(0.5 * time) + 0.0 * (rand() % 10 - 5);  // 模拟数据
        int signs = 0;
        if(sin(0.2 * time + M_PI / 4)>0)
        {signs = 1;}
        else
        {signs = -1;}
        left_force_.wrench.force.x = 0.0 + 20 * signs + 0.0 * (rand() % 10 - 5); 
        left_force_.wrench.force.z = 0.0 + 0.1 * sin(0.5 * time + M_PI / 2) + 0.0 * (rand() % 10 - 5);
        left_force_.wrench.torque.x = 0.0;
        left_force_.wrench.torque.y = 10 * sin(0.25 * time) + 0.0 * (rand() % 10 - 5);
        left_force_.wrench.torque.z = 0.0;
        pub_left_.publish(left_force_);

        // 右侧传感器数据
        right_force_.header.frame_id = "right_flange"; // 设置右传感器坐标系
        right_force_.header.stamp = ros::Time::now(); // 设置时间戳    
        right_force_.wrench.force.x = 0.0 + 0.1 * sin(0.6 * time) + 0.0 * (rand() % 10 - 5);  // 模拟数据
        right_force_.wrench.force.y = 0.0 + 20 * signs + 0.0 * (rand() % 10 - 5); 
        right_force_.wrench.force.z = 0.0 + 0.1 * sin(0.6 * time + M_PI / 2) + 0.0 * (rand() % 10 - 5);
        right_force_.wrench.torque.x = 0.0;
        right_force_.wrench.torque.y = 5 * sin(0.5 * time + M_PI / 2) + 0.0 * (rand() % 10 - 5);
        right_force_.wrench.torque.z = 0.0;
        pub_right_.publish(right_force_);
    }

private:
    ros::Publisher pub_left_;
    ros::Publisher pub_right_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_generator");
    ros::NodeHandle nh;
    ForcePublisher generator(nh);
    ros::Rate rate(200);  // 发布频率

    while (ros::ok()) {
        generator.publishForce();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
