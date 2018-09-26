#!/usr/bin/env python
import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from blue_hardware_drivers.msg import MotorState

class Base:
    def __init__(self):
        self.offset = 0.52935836

        rospy.init_node('Base_Calibration', anonymous=True)
        pub = rospy.Publisher('hi', String, queue_size=10)
        while(not rospy.is_shutdown()):
            rospy.sleep(1)
            rospy.logerr("hi")
            msg = String()
            msg.data = "hi"
            pub.publish(msg)

if __name__ == '__main__':
    link = Base()
