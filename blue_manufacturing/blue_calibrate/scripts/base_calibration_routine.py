#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from blue_hardware_drivers.msg import MotorState

class Base:
    def __init__(self):
        self.offset = 0.52935836

        rospy.init_node('Base_Calibration', anonymous=True)

        rate = rospy.Rate(200)

        self.cmd = rospy.Publisher("/blue_controllers/joint_position_controller/command", Float64MultiArray, queue_size=1)
        self.pos = [0.0]
        self.motor_pos = [0.0]

        rospy.Subscriber("/joint_states", JointState, self.update_joints)
        rospy.Subscriber("/blue_hardware/motor_states", MotorState, self.update_motors)

    def set_pos(self, cmds):
        # print(cmds)
        j_cmd = Float64MultiArray()
        j_cmd.data = cmds
        self.cmd.publish(j_cmd)

    def update_motors(self, joint_msg):
        self.motor_pos[0] = joint_msg.position[0]

    def update_joints(self, joint_msg):
        self.pos[0] = joint_msg.position[0]

    def center_joint(self, j=0, _iter=1):
        max_pos_list = []
        min_pos_list = []
        for _ in range(_iter):
            min_pos_list.append(self.find_end(j, -1.0))
            max_pos_list.append(self.find_end(j, 1.0))
        print(max_pos_list)
        print(min_pos_list)
        max_pos = sum(max_pos_list) / float(_iter)
        min_pos = sum(min_pos_list) / float(_iter)
        return (max_pos + min_pos) / 2.0, max_pos, min_pos

    def find_end(self, j, direction):
        eps = 2.0 # joint position erro in radians
        curr_error = 0
        delta = 0.05
        command_pos = self.pos[:]
        while curr_error < eps and not rospy.is_shutdown():
            # rospy.logerr("{}".format(curr_error))
            command_pos[j] = command_pos[j] + direction *  delta
            self.set_pos(command_pos)
            curr_error = np.abs(command_pos[j] - self.pos[j])
            rospy.logerr("{}".format(curr_error))
            rospy.sleep(0.01)

        rospy.sleep(0.5)
        rospy.loginfo('done finding the end')
        return self.pos[j]

if __name__ == '__main__':
    link = Base()
    iterations = 3
    link = Base()
    rospy.loginfo('Link setup complete')
    # set to start up position
    rospy.sleep(2)
    rospy.loginfo('Set to zero')
    rospy.sleep(2)
    # centering roll link first
    center_roll, max_roll, min_roll = link.center_joint(_iter=iterations)
    print(max_roll - min_roll)
    go_to_value = center_roll - link.offset
    print('Finding end')
    link.set_pos([go_to_value])
    rospy.sleep(2)
    print(link.pos[0])
    print(go_to_value)
    print(link.pos[0])

    rospy.sleep(1)
    link.set_pos([go_to_value])
    print(center_roll, "center roll")
    rospy.sleep(2)
    rospy.loginfo('Script Complete')

    print("Motor Position")
    print(link.motor_pos[0])
    print(link.motor_pos[0])
    print(link.motor_pos[0]%(2*np.pi))
