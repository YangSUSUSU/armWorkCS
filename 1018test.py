import rosbag
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Pose

# 设置要读取的 .bag 文件路径
bag_file = 'sintest.bag'

# 初始化列表来存储时间和位置数据
time_data = []
x_data = []
y_data = []
z_data = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, time in bag.read_messages(topics=['/test_armTcp']):
        pose: Pose = msg
        x_data.append(pose.position.x)
        y_data.append(pose.position.y)
        z_data.append(pose.position.z)
        time_data.append(time.to_sec())
# 绘制线条
plt.figure(figsize=(12, 8))

# 绘制 x vs time
plt.subplot(3, 1, 1)
plt.plot(time_data, x_data, label='X Position', color='r')
plt.xlabel('Time (s)')
plt.ylabel('X Position')
plt.title('X Position vs Time')
plt.grid()
plt.legend()

# 绘制 y vs time
plt.subplot(3, 1, 2)
plt.plot(time_data, y_data, label='Y Position', color='g')
plt.xlabel('Time (s)')
plt.ylabel('Y Position')
plt.title('Y Position vs Time')
plt.grid()
plt.legend()

# 绘制 z vs time
plt.subplot(3, 1, 3)
plt.plot(time_data, z_data, label='Z Position', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Z Position')
plt.title('Z Position vs Time')
plt.grid()
plt.legend()

# 显示图形
plt.tight_layout()
plt.show()
