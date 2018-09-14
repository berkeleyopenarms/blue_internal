#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from blue_hardware_drivers.msg import MotorState


reset_torque = 1
slam_torque = 10
class Base:
    def __init__(self):
        rospy.init_node('Base_Calibration', anonymous=True)
        self.rate = rospy.Rate(200)

        self.cmd = rospy.Publisher("/blue_controllers/torque_controller/command", Float64MultiArray, queue_size=1)
        self.pos = [0.0]
        rospy.Subscriber("/joint_states", JointState, self.update_joints)

    def set_torque(self, cmds):
        # print(cmds)
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds
        self.cmd.publish(j_cmd)

    def update_joints(self, joint_msg):
        self.pos[0] = joint_msg.position[0]

def torque_time(b, val, time):
    now_time = rospy.get_time()
    b.set_torque(val)
    while now_time + time  > rospy.get_time():
        b.rate.sleep()
    b.set_torque(0.0)


def cycle(b):
    torque_time(b, reset_torque, 3)
    torque_time(b, reset_torque, 1)

if __name__ == '__main__':
    b = Base()
    count = 0
    while not rospy.is_shutdown():
        cycle(b)
        count ++;
        if count % 10 == 0:
            print(count)


