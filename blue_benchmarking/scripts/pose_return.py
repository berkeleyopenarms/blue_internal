import pickle  # will use to save data between program launches
import sys
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot
kk = BlueInterface("right","hekate.cs.berkeley.edu")  # creates object of class KokoInterface at the IP in quotes with the name 'kk'
kk.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)

joint_angle_list = pickle.load( open("benchmark_joint_angle_list.p", "rb")) #uses the pickle function to read the binary file created in record_poses.py

done = False
while not done:
	s = raw_input("Type the pose number to which you want to return. Type 'd' for done.")
	if s != d:
		kk.set_joint_positions(np.array(joint_angle_list[s])) # tell the robot to go to a set of joint angles
		print('position set')
	else:
		done = True
