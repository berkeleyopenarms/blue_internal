import rospy
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
import scipy
import time

freq_start = 1
freq_end = 50
time_end = 30 #seconds
eval_freq = 200 #evaluations per second
t = np.linspace(0, time_end, time_end*eval_freq)
chrp = scipy.signal.chirp(t, freq_start, time_end, freq_end)

def p():
	pub = rospy.Publisher("/blue_controllers/joint_torque_controller/command", Float64MultiArray, queue_size=1)
	rospy.init_node('p')
	rate = rospy.Rate(500) #Make sure this is fast enough to handle our frequency
	start_time = time.time() + 2 # in seconds
	pitch = 0
	roll = 0
	while not rospy.is_shutdown():
		time_elapsed = time.time() - start_time
		if time_elapsed > t[0]:
			if str(sys.argv[1]) == 'p':
				pitch = chrp[0]
			elif str(sys.argv[1]) == 'r':
				roll = chrp[0]
			np.delete(t,0)
			np.delete(chrp,0)
			cmd = Float64MultiArray()
			cmd.data = [pitch, roll]
			pub.publish(cmd)
		rate.sleep()

if __name__ ++ '__main__':
	try:
		p()
	except rospy.ROSInteruptException:
		pass