import pickle  # will use to save data between program launches
import sys
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot
sys.path.append('../../blue_vive_tracking/scripts')
from vive_tracking import ViveTracker
import random
import os
import time
import numpy as np

kk = BlueInterface("right","hekate.cs.berkeley.edu")  # creates object of class KokoInterface at the IP in quotes with the name 'kk'
kk.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)

vv = ViveTracker()

kk.command_gripper(2.05,.1) #This isn't working

if not os.path.isfile('../results/benchmark_joint_angle_list.p'):
	print("You should first run 'record_positions.py")
	#TODO make this an actual error

joint_angle_list = pickle.load( open("../results/benchmark_joint_angle_list.p", "rb")) #uses the pickle function to read the binary file created in record_poses.py
pose_list = pickle.load( open("../results/benchmark_pose_list.p", "rb"))

location_list = [0 for x in range(len(joint_angle_list))] #initialize a list of locations where each entry is of the form [joint_angle_position, pose]
for k in range(len(joint_angle_list)):
	location_ID = k
	location_list[k] = [location_ID, joint_angle_list[k], pose_list[k]]

comp_pt = [] #initialize a data point list - this will hold the commanded and measured location at each command
collected_data = [] #initialize a list that will hold all the comp_pt lists and can be saved to an external file at the end

num_cycles = input("Please enter the number of cycles you would like to test: ")

shuff = raw_input("Do you want to shuffle the pose order? [y/n] ")
if shuff != 'y':
	print "Order will not be shuffled"

for i in range(num_cycles): #iterate over the number of cycles
	if shuff == 'y':	
		random.shuffle(location_list) #randomly shuffles the order of the list so that it goes to the poses in a different random order each time
	print "Cycle #%d" % (i+1)	
	for j in range(len(location_list)): #iterate over the shuffled pose list	
		
		curr_position = kk.get_joint_positions()
		num_steps = 20
		final = np.array(location_list[j][1])
		for i in range(num_steps):
			increment = (final-curr_position)/num_steps
			incremental = curr_position + i*increment
			kk.set_joint_positions() # tell the robot to go to a set of joint angles
		
		print('position set')
		time.sleep(3) #wait for any residual motion to subside
		measured_pose = vv.get_vive_tracker_data() #measure the pose using the vive system
		#print(measured_position)
		if measured_pose == 0:
			print "Don't forget to restart the Vive system on the other computer. Quitting now."
			sys.exit()
		pos = measured_pose.pose.position
		ori = measured_pose.pose.orientation
		measured_position = {'vive_position':[pos.x, pos.y, pos.z], 'vive_orientation':[ori.x, ori.y, ori.z, ori.w]}

		comp_pt = [] #initialize a data point list - this will hold the commanded and measured location at each command
		comp_pt = [location_list[j][0], location_list[j][2], measured_position] #collect the command pose and measured pose into a list
		collected_data.append(comp_pt) #put the list into the list of all data

if num_cycles == 0:
	print('No data collected: requested number of cycles was 0')
else:
	pickle.dump(collected_data, open("../results/benchmark_collected_data.p", "wb")) #uses the pickle function to write a binary file
	print('Your data has been saved in the results directory')

kk.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)
sys.exit()
