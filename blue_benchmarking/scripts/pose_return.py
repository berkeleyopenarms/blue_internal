import pickle  # will use to save data between program launches
import sys
import numpy as np
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot
blu = BlueInterface("right","hekate.cs.berkeley.edu")  # creates object of class KokoInterface at the IP in quotes with the name 'kk'
blu.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)
import numpy as np

joint_angle_list = pickle.load( open("../results/benchmark_joint_angle_list.p", "rb")) #uses the pickle function to read the binary file created in record_poses.py

done = False
while not done:
	s = raw_input("Type the pose number to which you want to return. Type 'd' for done. ")
	if s != 'd':
		if int(s) in range(len(joint_angle_list)):
			blu.set_joint_positions(np.array(joint_angle_list[int(s)])) # tell the robot to go to a set of joint angles
			print('position set')
		else:
			print "Please enter a valid pose number between 0 and %d " % (len(joint_angle_list)-1)
	else:
		done = True
		blu.disable_control()
