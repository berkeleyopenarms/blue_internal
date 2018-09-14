import rospy
import sys
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import signal
import time
import serial
import csv

ser = serial.Serial('/dev/serial/by-id/usb-Teensyduino_USB_Serial_1797530-if00') #serial address
ser.flushInput()
print("Serial connected")

mode = str(sys.argv[1])

# Torque bandwidth: 0.1, 40, 60, 400, 6
# Hysteresis: .1, .1, 180, 400, 6
freq_start = 2#0.1#.25 #Hz
freq_end = 2#40#.25 #Hz
time_end = 3#400 #seconds
eval_freq = 400#300#200 #evaluations per second
max_torque = 15#10 #Nm

file_name = str(sys.argv[2]) + '_' + str(max_torque) + 'Nm' + '.csv'


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
				pitch = max_torque*chrp[count]
			elif mode == 'r':
				roll = max_torque*chrp[count]
			else:
				print('Neither')
			#print("t val: " + str(t[count]))
			cmd.data = [pitch, roll]
			count += 1
			pub.publish(cmd)
			print('b')
			ser_bytes = ser.readline()
			print('a')
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
	print(time_elapsed)

if __name__ == '__main__':
	p()
