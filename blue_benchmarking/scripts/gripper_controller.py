#!/usr/bin/env python3

# A basic example of using BlueInterface for gripper control.
# It allows a user to open and close the gripper.

import sys
sys.path.append('blue_interface')
from blue_interface import BlueInterface
import numpy as np

if __name__ == '__main__':
    blue = BlueInterface("right", "hekate.cs.berkeley.edu")
    opened = True 
    try:
        while True:
            input("Press enter to open/close the gripper. To exit, press <ctrl+c>.")
    
            if opened:
                blue.command_gripper(-1,30.0)
                print("Closing...")
            else:
                blue.command_gripper(0.0,3.0)
                print("Opening...")
            opened = not opened
    except:
        pass

    blue.disable_control() 
    blue.cleanup()
    blue.shutdown()

