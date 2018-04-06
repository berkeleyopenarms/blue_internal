#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState
import numpy as np
import tf.transformations as transformations

REPORT_FREQ = 2

temperatures = {}

def get_temp(msg):
    global temperatures

    temp = {} # pun intended
    for i, name in enumerate(msg.name):
        temp[name] = msg.temperature[i]
    temperatures = temp

#################################################################################################

def main():
    global temperatures

    rospy.init_node('temperature_monitor', anonymous=True)
    rospy.Subscriber("koko_hardware/motor_states", MotorState, get_temp, queue_size=1)

    first = True
    r = rospy.Rate(REPORT_FREQ)
    motor_names = []
    while not rospy.is_shutdown():
        if len(temperatures) > 0:
            values = []
            if first:
                motor_names = sorted(temperatures.keys())
                values.append("time")
                values.extend(motor_names)
            else:
                values.append(rospy.get_time())
                for name in motor_names:
                    values.append(temperatures[name])
            values = [str(x) for x in values]
            print(','.join(values))
            first = False
        r.sleep()

if __name__ == '__main__':
    main()
