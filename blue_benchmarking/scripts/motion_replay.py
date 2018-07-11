import pickle  # will use to save data between program launches
import sys
import numpy as np
import time
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot
kk = BlueInterface("right","hekate.cs.berkeley.edu")  # creates object of class KokoInterface at the IP in quotes with the name 'kk'

# This turns off any other control currently on the robot (leaves it in gravtiy comp mode)
kk.disable_control() 

joint_angle_list = pickle.load( open("benchmark_joint_angle_list.p", "rb")) #uses the pickle function to read the binary file created in record_poses.py

try:
    for i in range (len(joint_angle_list)):
        kk.set_joint_positions(np.array(joint_angle_list[i])) # tell the robot to go to a set of joint angles
        print('position set')
        time.sleep(0.02)
except:
    pass

kk.disable_control() 
kk.cleanup()
kk.shutdown()
