#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from nav_msgs.msg import Path
import csv
import matplotlib.pyplot as plt

# 全局变量，用于存储接收到的数据
reference_path_data = []
predict_path_data = []
actual_path_data = []

# 回调函数，处理接收到的数据
# def reference_path_callback(msg):
#     global reference_path_data
#     for pose in msg.poses:
#         reference_path_data.append([
#             pose.header.stamp.to_sec(), 
#             pose.pose.position.x, 
#             pose.pose.position.y, 
#             pose.pose.position.z, 
#             pose.pose.orientation.x, 
#             pose.pose.orientation.y, 
#             pose.pose.orientation.z, 
#             pose.pose.orientation.w
#         ])


# def reference_path_callback(msg):
#     global reference_path_data
#     with open('reference_path.tum', 'a') as f:
#         for pose in msg.poses:
#             f.write(f"{pose.header.stamp.to_sec()} {pose.pose.position.x} {pose.pose.position.y} {pose.pose.position.z} {pose.pose.orientation.x} {pose.pose.orientation.y} {pose.pose.orientation.z} {pose.pose.orientation.w}\n") 

def reference_path_callback(msg):
    global reference_path_data
    with open('reference_path.tum', 'a') as f_tum, open('reference_path.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        for pose in msg.poses:
            data = [
                pose.header.stamp.to_sec(), 
                pose.pose.position.x, 
                pose.pose.position.y, 
                pose.pose.position.z, 
                pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w
            ]
            f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
            writer.writerow(data)
            reference_path_data.append(data)


def predict_path_callback(msg):
    global predict_path_data
    with open('predict_path.tum', 'a') as f_tum, open('predict_path.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        for pose in msg.poses:
            data = [
                pose.header.stamp.to_sec(), 
                pose.pose.position.x, 
                pose.pose.position.y, 
                pose.pose.position.z, 
                pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w
            ]
            f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
            writer.writerow(data)
            predict_path_data.append(data)



def actual_path_callback(msg):
    global actual_path_data
    with open('actual_path.tum', 'a') as f_tum, open('actual_path.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        for pose in msg.poses:
            data = [
                pose.header.stamp.to_sec(), 
                pose.pose.position.x, 
                pose.pose.position.y, 
                pose.pose.position.z, 
                pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w
            ]
            f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
            writer.writerow(data)
            actual_path_data.append(data)




# 初始化ROS节点
rospy.init_node('plot_tum')

# 订阅主题
rospy.Subscriber('mpc_car/reference_path', Path, reference_path_callback)
rospy.Subscriber('mpc_car/traj_delay', Path, predict_path_callback)
rospy.Subscriber('mpc_car/traj', Path, actual_path_callback)

rospy.spin()



# 绘制图形
# 创建一个新的图形
fig = plt.figure()

plt.plot([d[1] for d in reference_path_data], [d[2] for d in reference_path_data], label='reference_path Trajectory')
plt.plot([d[1] for d in predict_path_data], [d[2] for d in predict_path_data], label='Predicted Trajectory')
plt.plot([d[1] for d in actual_path_data], [d[2] for d in actual_path_data], label='Actual Trajectory')
plt.title('Trajectories')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()