#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState

def extract_joint_data(bag_file, joint_names):
    """
    从 rosbag 文件中提取多个关节的 position 和 effort 数据。
    """
    joint_data = {name: {'timestamps': [], 'positions': [], 'efforts': []} for name in joint_names}
    
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/test_dyn_Error']):
            print("Available joint names in message: ", msg.name)  # 打印调试信息
            
            for joint_name in joint_names:
                if joint_name in msg.name:
                    index = msg.name.index(joint_name)
                    joint_data[joint_name]['timestamps'].append(t.to_sec())
                    joint_data[joint_name]['positions'].append(msg.position[index])
                    joint_data[joint_name]['efforts'].append(msg.effort[index])
    
    return joint_data

def plot_joint_data(joint_data):
    """
    绘制多个关节的 position 和 effort 数据。
    """
    # 绘制 Position 数据
    plt.figure(figsize=(10, 6))
    for joint_name, data in joint_data.items():
        if data['timestamps'] and data['positions']:
            plt.plot(data['timestamps'], data['positions'], label=f'Joint "{joint_name}" Position')
        else:
            print(f"未找到关节 '{joint_name}' 的 position 数据，跳过绘制！")
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title('Joint Positions Over Time')
    plt.legend()
    plt.grid()
    plt.show()

    # 绘制 Effort 数据
    plt.figure(figsize=(10, 6))
    for joint_name, data in joint_data.items():
        if data['timestamps'] and data['efforts']:
            plt.plot(data['timestamps'], data['efforts'], label=f'Joint "{joint_name}" Effort')
        else:
            print(f"未找到关节 '{joint_name}' 的 effort 数据，跳过绘制！")
    plt.xlabel('Time (s)')
    plt.ylabel('Effort (Nm)')
    plt.title('Joint Efforts Over Time')
    plt.legend()
    plt.grid()
    plt.show()

def print_joint_effort(joint_data):
    """
    打印每个关节的 effort 数据。
    """
    print("\nJoint Effort Data:")
    for joint_name, data in joint_data.items():
        if data['efforts']:
            print(f"Joint '{joint_name}': {data['efforts']}")
        else:
            print(f"Joint '{joint_name}' has no effort data.")

if __name__ == '__main__':
    # 配置 bag 文件路径和关节名称
    bag_file = '2024-11-22-15-50-58.bag'  # 替换为实际的 rosbag 文件路径
    joint_names = ['1', '2', '3', '4', '5', '6', '7']  # 所有目标关节名称

    # 提取数据
    joint_data = extract_joint_data(bag_file, joint_names)

    # 打印 Effort 数据
    print_joint_effort(joint_data)

    # 绘制数据
    plot_joint_data(joint_data)
