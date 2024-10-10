import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 定义参数
C = np.array([0, 0, 0])  # 圆心
r = 1.0                  # 半径
N = np.array([0, 0, 1])  # 法向量
phi = np.pi/1.5          # P2 相对于 P1 的角度

# 计算单位向量 U 和 V
A = np.array([1, 0, 0])  # 随机选择的向量
U = (A - np.dot(A, N) * N)
U = U / np.linalg.norm(U)  # 单位化
V = np.cross(N, U)         # 计算 V

# 创建图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('P1 Moves Back and Forth on a Circular Ring')

# 初始化 P1 和 P2
P1_line, = ax.plot([], [], [], 'bo', label='P1')  # P1
P2_line, = ax.plot([], [], [], 'ro', label='P2')  # P2

# 绘制圆环
theta = np.linspace(0, 2 * np.pi, 100)
circle_x = C[0] + r * np.cos(theta) * U[0] + r * np.sin(theta) * V[0]
circle_y = C[1] + r * np.cos(theta) * U[1] + r * np.sin(theta) * V[1]
circle_z = C[2] + r * np.cos(theta) * U[2] + r * np.sin(theta) * V[2]
ax.plot(circle_x, circle_y, circle_z, label='Circle', color='g', alpha=0.5)

# 更新函数
def update(frame):
    # 控制 P1 在圆环上往返，限制运动范围
    max_angle = np.pi / 1.5  # 设置最大运动弧度
    if frame < 50:  # 前半段
        theta = frame / 10.0 * (max_angle / 2.0)  # 将运动范围限制在 0 到 max_angle
    else:  # 后半段
        theta = (100 - frame) / 10.0 * (max_angle / 2.0)  # 回程

    P1 = C + r * (np.cos(theta) * U + np.sin(theta) * V)
    
    # 根据 P1 的位置计算 P2 的位置，保持与 P1 的固定角度
    P2 = C + r * (np.cos(theta + phi) * U + np.sin(theta + phi) * V)

    # 更新 P1 和 P2 的位置
    P1_line.set_data(P1[0], P1[1])
    P1_line.set_3d_properties(P1[2])

    P2_line.set_data(P2[0], P2[1])
    P2_line.set_3d_properties(P2[2])

    return P1_line, P2_line

# 创建动画
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), blit=True, interval=100)

# 添加图例
ax.legend()
plt.show()
