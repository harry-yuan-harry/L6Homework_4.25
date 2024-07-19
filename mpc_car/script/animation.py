#!/usr/bin/env python3
# -*- coding: utf-8 -*
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 读取csv文件
df = pd.read_csv('nmpc_solvetime_data.csv')

# 获取第一个时间戳
first_timestamp = df.iloc[0, 0]

# 将第一列数据全部减去第一个时间戳数据
df.iloc[:, 0] = df.iloc[:, 0] - first_timestamp

# 创建figure和axes对象
fig, ax = plt.subplots()

# 在初始化函数中创建两个线条
line_data, = ax.plot([], [], 'k-')  # 'k-'表示黑色实线
line, = ax.plot([], [], 'g-')

# 初始化函数
def init():
    ax.clear()
    ax.set_xlabel('Timestamp', fontsize=20)
    ax.set_ylabel('nmpc_slove_time (ms)', fontsize=20)
    ax.set_ylim(0, 10)  # 设置纵坐标范围
    ax.set_xlim(0, 45)  # 设置横坐标范围
    ax.tick_params(axis='both', labelsize=20)  # 设置刻度标签的字体大小
    ax.text(0.95, 0.95, 'Proposed NMPC', fontsize=16, va='top', ha='right',color='red', transform=ax.transAxes, 
        bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=1'))
    line_data.set_data([], [])
    line.set_data([], [])
    return line_data, line,

# 更新函数
def update(i):
    data = df.iloc[:int(i+1)]  # 选择数据
    if data.empty:
        return line_data, line,
    line_data.set_data(data.iloc[:, 0].values, data.iloc[:, 1].values)  # 更新线条的数据

    # 更新垂直线的位置
    line.set_data([data.iloc[-1, 0], data.iloc[-1, 0]], [0, df.iloc[:, 1].max()])
    return line_data, line,

# 创建动画
ani = animation.FuncAnimation(fig, update, frames=len(df), init_func=init, interval=100, blit=True, repeat_delay=100)

# 更新图像到最后一帧
update(len(df)-1)

# 保存动画为GIF文件
ani.save('animation.gif', writer='pillow')

plt.show()