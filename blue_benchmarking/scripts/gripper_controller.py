#!/usr/bin/env python3

# A basic example of using BlueInterface for gripper control.
# It allows a user to open and close the gripper.

import sys
sys.path.append('blue_interface')
from blue_interface import BlueInterface
import numpy as np
import consts

if __name__ == '__main__':
    #blue = BlueInterface("left", "10.42.0.1")
    blue = BlueInterface(consts.default_arm, consts.default_address) #creates object of class KokoInterface at the IP in quotes with the name 'blue'
    opened = True 
    try:
        while True:
            input("Press enter to open/close the gripper. To exit, press <ctrl+c>.")
    
            if opened:
                blue.command_gripper(-1.0,10.0)
                print("Closing...")
            else:
                blue.command_gripper(0.0,10.0)
                print("Opening...")
            opened = not opened
    except:
        pass

    blue.disable_control() 
    blue.cleanup()
    blue.shutdown()
