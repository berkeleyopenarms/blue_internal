import numpy as np #will use to create arrays
import pickle #will use to save data between program launches

import sys
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot
blu = BlueInterface("right","hekate.cs.berkeley.edu") #creates object of class KokoInterface at the IP in quotes with the name 'blu' # THIS IS THE LINE THAT GETS MY PGORAM STUCK FROM EXITING
blu.disable_control() #this turns off any other control currently on the robot (leaves it in gravtiy comp mode)


joint_angle_list = [] #Initialize the list to hold our joint positions
pose_list = []

done_collecting_positions = False #create a variable to track when we've collected all the desired poses
while not done_collecting_positions:
	s = raw_input("Move the robot to a new position and press enter to record or enter d for done and press enter")
	if s != "d": #if the user is not done
		position = blu.get_joint_positions() #record the pose, this function returns a dictionary object
		joint_angle_list.append(position)
		pose = blu.get_cartesian_pose()
		pose_list.append(pose)
		print("Position recorded!")
	else:
		done_collecting_positions = True #set the variable such that the loop doesn't run again

if len(joint_angle_list)==0:
	print('You did not save any positions')
else:
	pickle.dump(joint_angle_list, open("../results/benchmark_joint_angle_list.p", "wb")) #uses the pickle function to write a binary file
	pickle.dump(pose_list, open("../results/benchmark_pose_list.p", "wb"))
	print('Your position list has been saved in the results directory')

blu.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)
sys.exit()
