#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from nav_msgs.msg import Path
import csv
import matplotlib.pyplot as plt

# 全局变量，用于存储接收到的数据
reference_path_data = []
predict_data = []
actual_data = []

# 回调函数，处理接收到的数据
def reference_path_callback(msg):
    global reference_path_data
    for pose in msg.poses:
        reference_path.append([
            pose.header.stamp.to_sec(), 
            pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z, 
            pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w
        ])

def predict_callback(msg):
    global predict_data
    for pose in msg.poses:
        predict_data.append([
            pose.header.stamp.to_sec(), 
            pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z, 
            pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w
        ])

def actual_callback(msg):
    global actual_data
    for pose in msg.poses:
        actual_data.append([
            pose.header.stamp.to_sec(), 
            pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z, 
            pose.pose.orientation.x, 
            pose.pose.orientation.y, 
            pose.pose.orientation.z, 
            pose.pose.orientation.w
        ])
# 初始化ROS节点
rospy.init_node('plot_tum')

# 订阅主题
rospy.Subscriber('reference_path', Path, reference_path_callback)
rospy.Subscriber('traj_delay', Path, predict_callback)
rospy.Subscriber('traj', Path, actual_callback)

rospy.spin()

# 保存数据到csv文件
with open('reference_path.csv', 'w') as f:
    writer = csv.writer(f)
    for row in reference_path_data:
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

plt.plot([d[0] for d in reference_path_data], [d[1] for d in reference_path_data], label='reference_path Trajectory')
plt.plot([d[0] for d in predict_data], [d[1] for d in predict_data], label='Predicted Trajectory')
plt.plot([d[0] for d in actual_data], [d[1] for d in actual_data], label='Actual Trajectory')
plt.title('Trajectories')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()