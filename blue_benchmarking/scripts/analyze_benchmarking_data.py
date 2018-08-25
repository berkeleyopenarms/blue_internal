import pickle
import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion



data = pickle.load( open("../results/benchmark_collected_data.p", "rb")) #uses the pickle function to read the binary file created in execute_benchmarking
# This contains a list where each element of a list follows [pose_ID, robot pose, vive_measurement]

def euler_from_quat(quaternion):
    return euler_from_quaternion(quaternion) # Returns in radians

num_IDs = 1 #Initialize variable to hold the total number of unique position IDs
for i in range(len(data)): #This loop finds the largest position ID and sets num_IDs equal to it
    curr_ID = data[i][0] + 1 #because IDs are 0-indexed
    if curr_ID > num_IDs:
        num_IDs = curr_ID

vive_data_list = [] #initialize a list for the vive data
for j in range(num_IDs): #this loop creates an empty list within vive_data_list for each unique position ID
    vive_data_list.append([])

for k in range(len(data)):
    curr_ID = data[k][0]
    vive_meas = data[k][2]
    vive_data_list[curr_ID].append(vive_meas)

output_list = [0]*num_IDs #Initialize list that will contain a dictionary for each unique pose to hold std_dev data

for position_ID in range(num_IDs):
    ID_data = vive_data_list[position_ID]
    x = [] #initialize list of x vive positions
    y = [] # '' y ''
    z = [] # '' z ''
    roll = [] # '' roll ''
    pitch = [] # '' pitch ''
    yaw = [] # '' yaw ''
    for n in range(len(ID_data)):
        commanded_pos_data = ID_data[n-1]        
        x.append(commanded_pos_data.get('vive_position')[0])
        y.append(commanded_pos_data.get('vive_position')[1])
        z.append(commanded_pos_data.get('vive_position')[2])
        euler = euler_from_quat(commanded_pos_data.get('vive_orientation'))
        roll.append(euler[0])
        pitch.append(euler[1])
        yaw.append(euler[2])
    output_list[position_ID]={
        'x_std':np.std(np.array(x)),
        'y_std':np.std(np.array(y)),
        'z_std':np.std(np.array(z)),
        'roll_std':np.std(np.array(roll)),
        'pitch_std':np.std(np.array(pitch)),
        'yaw_std':np.std(np.array(yaw))
        }
 
for l in range(len(output_list)):
    print "Pose %d: %s" % ((l+1), output_list[l])

#CHANGE THIS: change the following to a more standard file type than pickle
pickle.dump(output_list, open("../results/std_data.p", "wb")) #uses the pickle function to write a binary file
print('Your data has also been saved in the directory')

