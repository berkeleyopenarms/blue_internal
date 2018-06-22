#!/usr/bin/env python
import rospy
import sys
import actionlib
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

#global tracker_data
reference = 0
response = 0
count = 0

def reference_callback(ref):
    global reference
    reference = ref

def output_callback(act):
    global response
    global count
    if act != response:
        count = count +1
    response = act
    
class ControllerVis():
    def get_reference_data(self):          
        return reference

    def get_response_data(self):
        return response

    def get_count(self):
        return count

    def __init__(self):	
        rospy.init_node("controller_visualization")
        rospy.Subscriber("right_arm/blue_controllers/joint_position_controller/command", Float64MultiArray, reference_callback)
        rospy.Subscriber("joint_states", JointState, output_callback)
