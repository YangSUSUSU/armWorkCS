#include <iostream>
#include <cmath>

const double PI = 3.14159265358979323846;

// 圆心和半径
const double C[3] = {0.0, 0.0, 0.0};  // 圆心
const double r = 1.0;                  // 半径
const double phi = PI / 2;             // P2 相对于 P1 的角度

void calculatePositions(int frame) {
    double max_angle = PI / 4;  // 最大运动弧度

    // 控制 P1 在圆环上往返，限制运动范围
    double theta;
    if (frame < 50) {  // 前半段
        theta = (frame / 10.0) * (max_angle / 5.0);  // 从 0 到 max_angle
    } else {  // 后半段
        theta = ((100 - frame) / 10.0) * (max_angle / 5.0);  // 从 max_angle 回到 0
    }

    // 计算 P1 的位置
    double P1[3];
    P1[0] = C[0] + r * (cos(theta));  // X 坐标
    P1[1] = C[1] + r * (sin(theta));  // Y 坐标
    P1[2] = C[2];                     // Z 坐标保持不变

    // 计算 P2 的位置，保持与 P1 的固定角度
    double P2[3];
    P2[0] = C[0] + r * (cos(theta + phi));  // X 坐标
    P2[1] = C[1] + r * (sin(theta + phi));  // Y 坐标
    P2[2] = C[2];                           // Z 坐标保持不变

    // 计算 P1 和 P2 之间的距离
    double distance = sqrt(pow(P2[0] - P1[0], 2) + pow(P2[1] - P1[1], 2) + pow(P2[2] - P1[2], 2));

    // 输出 P1 和 P2 的位置以及它们之间的距离
    // std::cout << "Frame: " << frame << "\n";
    // std::cout << "P1: (" << P1[0] << ", " << P1[1] << ", " << P1[2] << ")\n";
    // std::cout << "P2: (" << P2[0] << ", " << P2[1] << ", " << P2[2] << ")\n";
    std::cout << "Distance between P1 and P2: " << distance << "\n\n";
}

int main() {
    // 模拟 100 帧
    for (int frame = 0; frame < 100; ++frame) {
        calculatePositions(frame);
    }
    return 0;
}
