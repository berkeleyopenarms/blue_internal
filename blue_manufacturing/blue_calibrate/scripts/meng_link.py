#!/usr/bin/env python

import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from blue_hardware_drivers.msg import MotorState
from duty_cycle import Gripper

# reset_torque = -10
# slam_torque = 0.5
reset_torque = -11
slam_torque = 12
# slam_torque = 0.5

class Link:
    def __init__(self):
        rospy.init_node('Calibration', anonymous=True)
        self.rate = rospy.Rate(200)

        self.cmd = rospy.Publisher("/blue_controllers/torque_controller/command", Float64MultiArray, queue_size=1)
        self.pos = [0.0, 0.0]

        rospy.Subscriber("/joint_states", JointState, self.update_joints)

    def set_torque(self, cmds):
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds
        self.cmd.publish(j_cmd)


    def update_joints(self, joint_msg):
        for i, n in enumerate(joint_msg.name):
            if n == 'lift_joint':
                self.pos[0] = joint_msg.position[i]
            elif n == 'roll_joint':
                self.pos[1] = joint_msg.position[i]
            else:
                pass

def torque_time(b, val, time):
    now_time = rospy.get_time()
    b.set_torque(val)
    while now_time + time  > rospy.get_time() and not rospy.is_shutdown():
        b.rate.sleep()
    b.set_torque([0.0, 0.0])

def cycle(l):
    # torque_time(l, [slam_torque, 0.0], 0.8)
    # torque_time(l, [reset_torque, 0.0], 0.05)
    torque_time(l, [slam_torque, 0.0], 0.2)
    torque_time(l, [0.0, 0.0], 1.5)
    torque_time(l, [reset_torque, 0.0], 0.4)
    torque_time(l, [reset_torque-2, 0.0], 0.3)
    torque_time(l, [reset_torque-4, 0.0], 1.3)
    # torque_time(l, [0.0, 0.0], 0.1)
    # torque_time(l, [reset_torque, 0.0], 0.9)
    # torque_time(l, [reset_torque/3, 0.0], 2.5)

def gripper_is_ok(g,b,count=0):
    return True
    try:
        states = []
        now_time = rospy.get_time()
        while now_time + 1 > rospy.get_time() and not rospy.is_shutdown():
            states.append(g.get_state())
            b.rate.sleep()
        before = np.mean(np.array(states))
        print(before)

        now_time = rospy.get_time()
        g.set_cmd(0.3)
        while now_time + 1 > rospy.get_time() and not rospy.is_shutdown():
            g.set_cmd(0.3)
            b.rate.sleep()
        b.set_torque([0.0, 0.0])

        states = []
        now_time = rospy.get_time()
        while now_time + 1 > rospy.get_time() and not rospy.is_shutdown():
            states.append(g.get_state())
            b.rate.sleep()
        after = np.mean(np.array(states))
        print(after)

        now_time = rospy.get_time()
        g.set_cmd(-0.2)
        while now_time + 2 > rospy.get_time() and not rospy.is_shutdown():
            g.set_cmd(-0.2)
            b.rate.sleep()
        b.set_torque([0.0, 0.0])

        diff = np.abs(before-after)
        print(before, after, diff)
        if diff > 3:
            print("working")
            return True
        print("not working")
        return False
    except:
        print("exception, trying again")
        if count > 1000:
            print("more than 1000 executive failures, something sucks")
            return False
        time.sleep(0.01)
        return gripper_is_ok(g, b, count + 1)


if __name__ == '__main__':
    b = Link()
    # g = Gripper('/dev/ttyUSB1', 56)
    # gripper_is_ok(g,b)
    # cycle(b)
    torque_time(b, [0.0, 0.0], 2)
    torque_time(b, [5, 0.0], 1)

    count = 0
    while not rospy.is_shutdown():
        cycle(b)
        count = count + 1
        # if (count % 10) == 0:
        print(count)
        # if gripper_is_ok(g,b):
        #     pass
        # else:
        #     break

    print("final count: {}".format(count))
