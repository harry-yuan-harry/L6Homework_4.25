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

# Path to the input file
input_file_path = '/home/harry/Odomety_data_256.tum'

# Read the file
with open(input_file_path, 'r') as file:
    lines = file.readlines()

# Convert tab-separated values to space-separated values
converted_lines = []
for line in lines:
    if '\t' in line:
        # Replace tabs with single spaces
        converted_line = line.replace('\t', ' ')
        converted_lines.append(converted_line)
    else:
        converted_lines.append(line)

# Path to the output file
output_file_path = '/home/harry/Converted_Odomety_data_256.tum'

print ("输出完成！")

# Write the converted lines to the new file
with open(output_file_path, 'w') as file:
    file.writelines(converted_lines)
