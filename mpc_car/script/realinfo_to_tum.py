#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from nav_msgs.msg import Path
from omniGKF_control.msg import omniGKFinfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import csv

import matplotlib.pyplot as plt

# 全局变量，用于存储接收到的数据
omniGKFinfo_data = []
Odometry_data = []
reference_path_data = []

# 回调函数，处理接收到的数据
# def omniGKFinfo_data_callback(msg):
#     global omniGKFinfo_data
#     for pose in info.poses:
#         spline_data.append([msg.header.stamp.to_sec(),pose.pose.position.x, pose.pose.position.y])

# def Odometry_data_callback(msg):
#     global Odometry_data
#     for pose in msg.poses:
#         Odometry_data.append([msg.header.stamp.to_sec(),pose.pose.position.x, pose.pose.position.y])
def reference_path_callback(msg):
    pose = msg.pose
    global reference_path_data
    with open('reference_path.tum', 'a') as f_tum, open('reference_path.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        data = [
                msg.header.stamp.to_sec(), 
                pose.position.x, 
                pose.position.y, 
                pose.position.z, 
                pose.orientation.x, 
                pose.orientation.y, 
                pose.orientation.z, 
                pose.orientation.w
        ]
        f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
        writer.writerow(data)
        reference_path_data.append(data)

def Odometry_data_callback(msg):
    global Odometry_data
    # Access position data from the Odometry message
    position = msg.pose.pose.position
    x = position.x
    y = position.y
    z = position.z
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)


    with open('Odometry_data.tum', 'a') as f_tum, open('Odometry_data.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        data = [
            msg.header.stamp.to_sec(), 
            position.x, 
            position.y, 
            position.z, 
            euler[0], 
            euler[1], 
            euler[2],
            quaternion[3]
        ]
        f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
        writer.writerow(data)
        Odometry_data.append(data)



def omniGKFinfo_data_callback(msg):
    global omniGKFinfo_data
    with open('omniGKFinfo_data.tum', 'a') as f_tum, open('omniGKFinfo_data.csv', 'a') as f_csv:
        writer = csv.writer(f_csv)
        data = [
            msg.header.stamp.to_sec(), 
            msg.velocity[0], 
            msg.heading, 
            0,0,0,0,0,0
        ]
        f_tum.write(f"{data[0]} {data[1]} {data[2]} {data[3]} {data[4]} {data[5]} {data[6]} {data[7]}\n")
        writer.writerow(data)
        omniGKFinfo_data.append(data)
   
# 初始化ROS节点
rospy.init_node('realinfo_data_get')

# 订阅主题
rospy.Subscriber('omniGKFinfo', omniGKFinfo, omniGKFinfo_data_callback)
rospy.Subscriber('Odometry', Odometry, Odometry_data_callback)
rospy.Subscriber('mpc_car/ref_state', PoseStamped, reference_path_callback)

rospy.spin()

# # 保存数据到csv文件
# with open('Odometry_data.csv', 'w') as f:
#     writer = csv.writer(f)
#     for row in Odometry_data:
#         writer.writerow(row)

# with open('predict_data.csv', 'w') as f:
#     writer = csv.writer(f)
#     for row in predict_data:
#         writer.writerow(row)

# with open('actual_data.csv', 'w') as f:
#     writer = csv.writer(f)
#     for row in actual_data:
#         writer.writerow(row)

# 绘制图形
# 创建一个新的图形
# fig = plt.figure()

# plt.plot([d[1] for d in spline_data], [d[2] for d in spline_data], label='Spline Trajectory')
# plt.plot([d[1] for d in predict_data], [d[2] for d in predict_data], label='Predicted Trajectory')
# plt.plot([d[1] for d in actual_data], [d[2] for d in actual_data], label='Actual Trajectory')
# plt.title('Trajectories')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.legend()
# plt.show()