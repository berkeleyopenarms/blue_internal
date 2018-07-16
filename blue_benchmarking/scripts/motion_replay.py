#!/usr/bin/python3
import pickle  # will use to save data between program launches
import sys
import numpy as np
import time
import argparse
sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Replay a motion of a blue arm.')
    parser.add_argument('port', type=int, help='Port that the ros web host was started on')
    parser.add_argument('record_file', type=str, help='Loads a recorded motion from this file')
    args = parser.parse_args()
    
    filename = args.record_file
    
    blue = BlueInterface("right","hekate.cs.berkeley.edu")  # creates object of class KokoInterface at the IP in quotes with the name 'blue'
    
    # This turns off any other control currently on the robot (leaves it in gravtiy comp mode)
    blue.disable_control() 
    
    data = pickle.load( open(filename, "rb")) #uses the pickle function to read the binary file created in record_poses.py
    joint_angle_list, _, gripper_list = data
    
    input("Press enter to start replay. To exit, press <ctrl+c>.")
    
    try:
        for i in range (len(joint_angle_list)):
            blue.set_joint_positions(np.array(joint_angle_list[i])) # tell the robot to go to a set of joint angles
            #blue.command_gripper(gripper_list[i], 30.0)
            if gripper_list[i] < -0.2:
                blue.command_gripper(-1.3, 15.0)
            else:
                blue.command_gripper(0, 2.0)
            time.sleep(0.02)
    except:
        pass
    
    blue.disable_control() 
    blue.cleanup()
    blue.shutdown()
