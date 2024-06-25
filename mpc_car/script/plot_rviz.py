#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from nav_msgs.msg import Path
import csv
import matplotlib.pyplot as plt

# 全局变量，用于存储接收到的数据
spline_data = []
predict_data = []
actual_data = []

# 回调函数，处理接收到的数据
def spline_callback(msg):
    global spline_data
    for pose in msg.poses:
        spline_data.append([msg.header.stamp.to_sec(),pose.pose.position.x, pose.pose.position.y])

def predict_callback(msg):
    global predict_data
    for pose in msg.poses:
        predict_data.append([msg.header.stamp.to_sec(),pose.pose.position.x, pose.pose.position.y])

def actual_callback(msg):
    global actual_data
    for pose in msg.poses:
        actual_data.append([msg.header.stamp.to_sec(),pose.pose.position.x, pose.pose.position.y])

# 初始化ROS节点
rospy.init_node('trajectory_plot')

# 订阅主题
rospy.Subscriber('mpc_car/reference_path', Path, spline_callback)
rospy.Subscriber('mpc_car/traj_delay', Path, predict_callback)
rospy.Subscriber('mpc_car/traj', Path, actual_callback)

rospy.spin()

# 保存数据到csv文件
with open('spline_data.csv', 'w') as f:
    writer = csv.writer(f)
    for row in spline_data:
        writer.writerow(row)

with open('predict_data.csv', 'w') as f:
    writer = csv.writer(f)
    for row in predict_data:
        writer.writerow(row)

with open('actual_data.csv', 'w') as f:
    writer = csv.writer(f)
    for row in actual_data:
        writer.writerow(row)

# 绘制图形
# 创建一个新的图形
fig = plt.figure()

plt.plot([d[1] for d in spline_data], [d[2] for d in spline_data], label='Spline Trajectory')
plt.plot([d[1] for d in predict_data], [d[2] for d in predict_data], label='Predicted Trajectory')
plt.plot([d[1] for d in actual_data], [d[2] for d in actual_data], label='Actual Trajectory')
plt.title('Trajectories')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()