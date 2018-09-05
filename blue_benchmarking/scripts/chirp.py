import rospy
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import signal
import time
import serial
import csv

# use roslaunch blue_calibrate calibrate_link.launch

ser = serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_1797530-if00') #serial address
ser.flushInput()
print("Serial connected")

mode = str(sys.argv[1])

freq_start = 0.1 #Hz
freq_end = 40 #Hz
time_end = 60 #seconds
eval_freq = 400 #evaluations per second
max_torque = 4.5 #Nm

file_name = str(sys.argv[2]) + '-' + str(max_torque) + 'Nm' + '.csv'


t = np.linspace(0, time_end, time_end*eval_freq)
chrp = signal.chirp(t, freq_start, time_end, freq_end)

def p():
	pub = rospy.Publisher("/blue_controllers/torque_controller/command", Float64MultiArray, queue_size=1)
	rospy.init_node('p')
	rate = rospy.Rate(500) #Make sure this is fast enough to handle our frequency
	start_time = time.time() + 1 # in seconds
	pitch = 0
	roll = 0
	count = 0
	cmd = Float64MultiArray()
	while not rospy.is_shutdown():
		time_elapsed = time.time() - start_time
		if time_elapsed > t[count]:
			if mode == 'p':
				pitch = chrp[count]
			elif mode == 'r':
				roll = max_torque*chrp[count]
			else:
				print('Neither')
			#print("t val: " + str(t[count]))
			cmd.data = [pitch, roll]
			count += 1
			pub.publish(cmd)
			ser_bytes = ser.readline()
			decoded_bytes = str((ser_bytes[0:len(ser_bytes)-2].decode("utf-8")))
			lst = decoded_bytes.split(',');
			if len(lst)==2:
				with open(file_name,"a") as f:
					writer = csv.writer(f,delimiter=",")
					writer.writerow([time.time(),float(lst[0]),float(lst[1]), float(pitch + roll)])
					flt_lst = []
		if count == len(t):
			cmd.data = [0, 0]
			pub.publish(cmd)
			ser.close()
			print("All done")
			break
		rate.sleep()

if __name__ == '__main__':
	p()