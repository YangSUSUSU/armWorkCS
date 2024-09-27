#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cstdlib>
#include <ctime>

class ForcePublisher {
public:
    ForcePublisher(ros::NodeHandle& nh) {
        pub_ = nh.advertise<geometry_msgs::WrenchStamped>("noisy_force", 100);
        srand(static_cast<unsigned int>(time(0)));  // 随机数种子
    }

    void publishForce() {
        geometry_msgs::WrenchStamped force_;
       
        double time = ros::Time::now().toSec(); // 获取当前时间
        force_.header.frame_id = "left_flange"; // 设置坐标系
        force_.header.stamp = ros::Time::now(); // 设置时间戳    
        force_.wrench.force.x = 0.0 + 15.0 *sin(0.2*time) + 1.0*(rand() % 10 - 5) / 100.0;  // 使用正弦函数并加噪声
        force_.wrench.force.y = 0.0 + 15.0 *sin(0.2*time + M_PI / 4) + 1.0*(rand() % 10 - 5) / 10.0; // 不同相位
        force_.wrench.force.z = 0.0 + 15.0 * sin(0.2*time + M_PI / 2) + 1.0*(rand() % 10 - 5) / 10.0; // 不同相位

        pub_.publish(force_);
    }

private:
    ros::Publisher pub_;
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
