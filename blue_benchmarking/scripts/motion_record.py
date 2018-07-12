#!/usr/bin/python3
import numpy as np #will use to create arrays
import pickle #will use to save data between program launches
import time

import sys
import argparse

sys.path.append('blue_interface')
from blue_interface import BlueInterface  # this is the API for the robot

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Record motion of a blue arm.')
    parser.add_argument('record_file', type=str, help='Saves a recorded motion to this file')
    args = parser.parse_args()

    filename = args.record_file

    kk = BlueInterface("right","hekate.cs.berkeley.edu") #creates object of class KokoInterface at the IP in quotes with the name 'kk'
    kk.disable_control() #this turns off any other control currently on the robot (leaves it in gravtiy comp mode)
    
    joint_angle_list = [] #Initialize the list to hold our joint positions
    pose_list = []
    gripper_list = []
    
    input("Press enter to start recording. To finish recording press <ctrl+c>.")

    try: 
        while True:
            position = kk.get_joint_positions() #record the pose, this function returns a dictionary object
            joint_angle_list.append(position)
            pose = kk.get_cartesian_pose()
            pose_list.append(pose)
            gripper_pos = kk.get_gripper_position()
            gripper_list.append(gripper_pos)
            print("Position recorded!")
            time.sleep(0.02)
    except:
        print(joint_angle_list)
    
    if len(joint_angle_list)==0:
        print('You did not save any positions')
    else:
        pickle.dump((joint_angle_list, pose_list, gripper_list), open(filename, "wb")) #uses the pickle function to write a binary file
        print('Your position list has been saved in the directory')
    
    kk.disable_control()  # this turns off any other control currently on the robot (leaves it in gravtiy comp mode)
    kk.cleanup()
    kk.shutdown()
