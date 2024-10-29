# import numpy as np
# import matplotlib.pyplot as plt
# import mujoco_py
# import os
# from mujoco_py import MjViewer

# # 设置模型文件的路径
# model_path = os.path.join(os.path.expanduser('~/.mujoco/mujoco210/bin/models'), 'AUBO_xml.xml')

# # 加载模型
# model = mujoco_py.load_model_from_path(model_path)

# # 创建模拟
# sim = mujoco_py.MjSim(model)

# # 关节 2、4 和 6 对应的索引（根据模型的实际情况调整）
# joint_indices = [0, 3-1, 6-1 ,8-1,10]

# # 设置仿真步长
# time_step = 0.01  # 每步模拟的时间间隔
# simulation_time = 0.0  # 初始化仿真时间

# # 创建可视化窗口
# viewer = MjViewer(sim)

# # 设置采样次数
# num_samples = 10000  # 可以根据需要增加采样点的数量

# # 获取关节范围
# joint_ranges = model.jnt_range

# # 存储末端执行器位姿
# tcp_positions = []

# # 末端执行器的 body 名称 (替换为你的模型中的末端执行器名称)
# tcp_name = "left_link7"  # 替换为你的末端名称

# # 创建 Matplotlib 绘图窗口
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # 主循环，用于生成随机关节配置并显示 GUI 和工作空间
# for _ in range(num_samples):
#     random_qpos = np.zeros(model.nq)  # 初始化关节角度数组
    
#     # 为每个关节生成随机角度配置
#     for i in range(model.njnt):
#         if i in joint_indices:
#             # 对关节 2、4 和 6 应用正弦变化
#             random_qpos[i] = 0.5 * np.sin(simulation_time)  # 振幅为0.5，可以根据需要调整
#         else:
#             # 在关节范围内生成随机值
#             joint_min, joint_max = joint_ranges[i]
#             random_qpos[i] = 0
    
#     # 设置关节角度
#     sim.data.qpos[:model.njnt] = random_qpos
    
#     # 更新仿真状态
#     sim.forward()
    
#     # 获取末端执行器的位姿（位置信息）
#     tcp_position = sim.data.get_body_xpos(tcp_name)
#     tcp_positions.append(tcp_position)
    
#     # 更新 GUI 显示
#     viewer.render()
    
#     # 更新仿真时间
#     simulation_time += time_step

import gym
from gym import spaces
import numpy as np
import mujoco_py
import os

class CustomMujocoEnv(gym.Env):
    def __init__(self):
        super(CustomMujocoEnv, self).__init__()

        # 加载 MuJoCo 模型
        model_path = os.path.join(os.path.expanduser('~/.mujoco/mujoco210/bin/models'), 'AUBO_xml.xml')
        self.model = mujoco_py.load_model_from_path(model_path)
        self.sim = mujoco_py.MjSim(self.model)

        # 确保模型有 actuator
        if self.model.nu == 0:
            raise ValueError("模型中没有定义 actuator")

        # 定义动作空间
        # self.action_space = spaces.Box(low=0, high=0.2, shape=(self.model.nu,), dtype=np.float32)
        # 获取关节的范围
        self.joint_ranges = self.model.jnt_range
        print(self.joint_ranges)
        # 定义动作空间，范围基于关节的最大值和最小值
        self.action_space = spaces.Box(low=self.joint_ranges[:, 0], high=self.joint_ranges[:, 1], shape=(self.model.nu,), dtype=np.float32)


        # 定义观察空间
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.sim.data.qpos.size + self.sim.data.qvel.size,), dtype=np.float32)

        # 环境的其他参数
        self.goal_position = np.array([0.5, 0.3, 0.3])  # 目标点
        self.max_steps = 200
        self.current_step = 0
        self.viewer = mujoco_py.MjViewer(self.sim)

    def reset(self):
        self.sim.reset()
        self.current_step = 0
        # initial_state = np.random.uniform(-0.01, 0.01, size=self.sim.data.qpos.shape)
        # initial_state = np.random.uniform(low=self.model.jnt_range[:, 0], high=self.model.jnt_range[:, 1], size=self.sim.data.qpos.shape)
        initial_state = np.random.uniform(low=self.joint_ranges[:, 0], high=self.joint_ranges[:, 1], size=self.sim.data.qpos.shape)


        self.sim.data.qpos[:] = initial_state
        self.sim.forward()
        return np.concatenate([self.sim.data.qpos, self.sim.data.qvel])
    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)  # 限制在范围内
        self.sim.data.ctrl[:] = action
        self.sim.step()
        self.current_step += 1
        observation = np.concatenate([self.sim.data.qpos, self.sim.data.qvel])
        end_effector_pos = self.sim.data.site_xpos[self.sim.model.site_name2id("left_link7_site")]
        distance_to_goal = np.linalg.norm(end_effector_pos - self.goal_position)
        reward = -distance_to_goal
        done = distance_to_goal < 0.05 or self.current_step >= self.max_steps
        return observation, reward, done, {}


    def render(self, mode='human'):
        self.viewer.render()

    def close(self):
        pass

# 测试环境
env = CustomMujocoEnv()
observation = env.reset()

for _ in range(1000000):
    action = env.action_space.sample()  # 随机动作
    observation, reward, done, info = env.step(action)
    print(reward)
    env.render()
    if done:
        env.reset()

env.close()
