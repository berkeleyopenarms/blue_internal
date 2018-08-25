#!/usr/bin/env python
import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

#global tracker_data
tracker_data = 0

def tracking_callback(pos):
	global tracker_data
	tracker_data = pos

class ViveTracker():
	def get_vive_tracker_data(self):   
		return tracker_data



	def __init__(self):
		rospy.init_node("vive_tracking")
		rospy.Subscriber("upper_arm_tracker_pose", PoseStamped, tracking_callback)



