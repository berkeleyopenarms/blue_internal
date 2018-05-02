#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Vector3
from koko_hardware_drivers.msg import MotorState
import numpy as np
import tf.transformations as transformations

#
# The output of this script is intended to be piped to a CSV file
# $ rosrun koko_misc joint_position_benchmarking.py | tee ~/joint_pos_log.csv
#

REPORT_FREQ = 100

temperatures = {}
d_currents = {}
q_currents = {}

def get_temp(msg):
    # in retrospect, we could probably just save
    # the msg object instead of using all these
    # global dicts

    global temperatures
    temp = dict(temperatures) # pun intended
    for i, name in enumerate(msg.name):
        temp[name] = msg.temperature[i]
    temperatures = temp

    global d_currents
    temp = dict(d_currents)
    for i, name in enumerate(msg.name):
        temp[name] = msg.direct_current[i]
    d_currents = temp

    global q_currents
    temp = dict(q_currents)
    for i, name in enumerate(msg.name):
        temp[name] = msg.quadrature_current[i]
    q_currents = temp

#################################################################################################

def main():
    global temperatures

    rospy.init_node('temperature_monitor', anonymous=True)
    rospy.Subscriber("koko_hardware/motor_states", MotorState, get_temp, queue_size=1)

    r = rospy.Rate(REPORT_FREQ)
    motor_names = []
    while not rospy.is_shutdown():
        if len(temperatures) > 0:
            values = []
            if len(temperatures) > len(motor_names):
                motor_names = sorted(temperatures.keys())
                values.append("time")
                values.extend([n + " temp" for n in motor_names])
                values.extend([n + " d current" for n in motor_names])
                values.extend([n + " q current" for n in motor_names])
            else:
                values.append(str(rospy.get_time()))
                def add_to_output(source):
                    for name in motor_names:
                        values.append(str(source[name]))
                add_to_output(temperatures)
                add_to_output(d_currents)
                add_to_output(q_currents)

            print ','.join(values)
            sys.stdout.flush()
        r.sleep()

if __name__ == '__main__':
    main()
