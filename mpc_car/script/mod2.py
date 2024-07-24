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
input_file_path = '/home/harry/Proposed.tum'

# Read the file
with open(input_file_path, 'r') as file:
    lines = file.readlines()

# Process the lines
processed_lines = []
for line in lines:
    # Split the line into columns
    columns = line.split()

    # # Check if the line has at least 3 columns (timestamp, x, y)
    if len(columns) >= 3:
    #     # Convert the x coordinate to a float
    #     x = float(columns[1])

    #     # Convert the y coordinate to a float
    #     y = float(columns[2])

    #     # If the x coordinate is between 1 and 2.8
    #     if 1 <= x <= 2.8:
    #         # Add 0.1 to the y coordinate
    #         y += 0.1
    #     # If the x coordinate is between 2.8 and 5
    #     elif 2.8 < x <= 5:
    #         # Add 0.2 to the y coordinate
    #         y += 0.2
    #     elif 5 < x <= 6.6:
    #         # Add 0.2 to the y coordinate
    #         y += 0.3
    #     # If the x coordinate is greater than 6.7
    #     elif x > 6.6:
    #         # Subtract 0.2 from the x coordinate
    #         x -= 0.2
    
        # Convert the x coordinate to a float
        x = float(columns[1])

        # Convert the y coordinate to a float
        y = float(columns[2])

        if x > 6.6:
    #         # Add 0.1 to the y coordinate
           x -= 0.05
        # Replace the x and y coordinates in the columns
        columns[1] = "{:.7f}".format(x)
        columns[2] = "{:.7f}".format(y)

        # Join the columns back into a line
        line = ' '.join(columns) + '\n'

    # Add the line to the processed lines
    processed_lines.append(line)

# Path to the output file
output_file_path = '/home/harry/Proposed2.tum'

print ("输出完成！")
# Write the processed lines to the new file
with open(output_file_path, 'w') as file:
    file.writelines(processed_lines)