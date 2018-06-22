import sys
sys.path.append('../../../../../blue_interface')
from blue_interface import BlueInterface
from control_benchmarking import ControllerVis
import time
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import sleep

control_info = ControllerVis()

blu = BlueInterface("right","hekate.cs.berkeley.edu")
blu.disable_control()

sampling_delay = 1.0/125 # seconds, samples at about the same rate as the response is published (use rostopic hz /joint_states)
sample_duration = 2

# Helper functions
def record_rxn():
	time_end = time.time() + sample_duration # 3 seconds from now

	# Initialize
	cmd = []
	response = []
	while time.time()< time_end:
		cmd.append(control_info.get_reference_data())
		response.append(control_info.get_response_data())
		
		sleep(sampling_delay)
	print "Samples collected: %d" % len(cmd)
	return (cmd, response)

def plot_response(command, response):
	# Manipulate the data to be useful (i.e. break up the 'command' and 'response by joint
	j_res = [[],[],[],[],[],[],[],[]] # [[base],[elbow lift],[elbow roll],[gripper],[shoulder lift],[shoulder roll],[wrist lift],[wrist roll]] for responses
	j_eff = [[],[],[],[],[],[],[],[]] # [[base],[elbow lift],[elbow roll],[gripper],[shoulder lift],[shoulder roll],[wrist lift],[wrist roll]] for joint effort
	for i in range(len(response)):
		for k in range(len(j_res)):
			j_res[k].append(response[i].position[k])
			j_eff[k].append(response[i].effort[k])

	j_cmd = [[],[],[],[],[],[],[]] # [[base],[shoulder lift],[shoulder roll],[elbow lift],[elbow roll],[wrist lift],[wrist roll]] for commands
	for i in range(len(command)):
		for k in range(len(j_cmd)):
			if command[i] == 0:
				j_cmd[k].append(0)
			else:
				j_cmd[k].append(command[i].data[k])

	# Make time vector
	time = [0]
	for i in range(len(j_cmd[0])-1): # using j_cmd[0] is simply the number of samples in the data	
		time.append(time[i]+sampling_delay)
	
	# Reorder the vectors to be in a better (and unified) order
	j_names = ["Base roll","Shoulder lift","Shoulder roll","Elbow lift","Elbow roll","Wrist lift","Wrist roll"]	
	j_res_ordered = [j_res[0],j_res[4],j_res[5],j_res[1],j_res[2],j_res[6],j_res[7]]
	j_cmd_ordered = [j_cmd[0],j_cmd[1],j_cmd[2],j_cmd[3],j_cmd[4],j_cmd[5],j_cmd[6]]
	j_eff_ordered = [j_eff[0],j_eff[4],j_eff[5],j_eff[1],j_eff[2],j_eff[6],j_eff[7]]

	# Calculate response characteristics
	ss_error = [abs(j_cmd_ordered[i][-1]-j_res_ordered[i][-1]) for i in range(len(j_names))]	
	percent_os = [get_percent_os(j_res_ordered[i], j_cmd_ordered[i]) for i in range(len(j_names))]
	rise_time = [get_rise_time(j_res_ordered[i], j_cmd_ordered[i], time) for i in range(len(j_names))]
	settling_time = [get_settling_time(j_res_ordered[i], j_cmd_ordered[i], time, .05) for i in range(len(j_names))]

	# Make the plots
	fig, ax = plt.subplots(nrows = 7, ncols = 1, figsize=(10,12))
	ax_eff = [ax[0].twinx(), ax[1].twinx(), ax[2].twinx(), ax[3].twinx(), ax[4].twinx(), ax[5].twinx(), ax[6].twinx()]

	for n in range(len(j_names)):
		ax[n].plot(time, j_cmd_ordered[n])
		ax[n].plot(time, j_res_ordered[n])
		
		ax[n].plot([2.5], [0]) # This just adds some whitespace on the plots for the text

		ax[n].set_title(j_names[n])
		ax[n].set_xlabel("Time [s]")
		ax[n].set_ylabel("Magnitude [rad]")

		ax[n].set_ylim((min(j_res_ordered[n])-.2, max(j_res_ordered[n])+.2))
		
		ax[n].text(.8,.5, "SS error: %.3f [rad]\nPercent OS: %.3f [%%]\nRise Time: %.3f [s]\nSettling Time: %.3f [s]" % (ss_error[n], percent_os[n], rise_time[n], settling_time[n]), horizontalalignment='left', verticalalignment='center', transform=ax[n].transAxes)
	
	for m in range(len(ax_eff)):
		ax_eff[m].plot(time, j_eff_ordered[m], color="green")
		ax_eff[m].set_ylabel("Effort [Nm]", color = "green")	
		ax_eff[m].tick_params(axis = 'y', colors = 'green')
	
	plt.tight_layout()
	plt.show()

	fig2, ax2 = plt.subplots(nrows = 1, ncols = 1, figsize=(15,15))
	ax2.scatter(time, j_res_ordered[0])
	plt.show()
'''
	fig3, ax3 = plt.subplots(nrows = 1, ncols = 1, figsize=(15,15))
	ax3.plot(time, j_eff[0])
	plt.show()
'''
def get_percent_os(response, command):
	final_cmd_value = command[-1]
	initial_value = response[0]
	if initial_value < final_cmd_value: # i.e. we are looking for %os ABOVE the command
		extreme = max(response)
		if max(response) < final_cmd_value: # i.e. there is no OS
			return 0 
	else: # i.e. we are looking for %os BELOW the command
		extreme = min(response)
		if min(response) > final_cmd_value: # i.e. there is no OS
			return 0
	pos = 100*(abs(abs(extreme)-abs(final_cmd_value)))/abs(final_cmd_value)	
	return pos

def get_rise_time(response, command, time):
	final_value = command[-1]
	initial_value = response[0]
	delta = abs(abs(final_value)-abs(initial_value))
	if initial_value > final_value: # i.e. the plot is trending downwards
		ten_value = initial_value - 0.1 * delta
		ninety_value = initial_value - 0.9 * delta
	else: # i.e. the plot is trending upwards
		ten_value = initial_value + 0.1 * delta 
		ninety_value = initial_value + 0.9*delta
	ten_index = find_index_of_closest(response, ten_value)
	ninety_index = find_index_of_closest(response, ninety_value)
	ten_time = time[ten_index]
	ninety_time = time[ninety_index]	
	rise_time = ninety_time - ten_time
	return rise_time

def find_index_of_closest(old_lst, arg):	
	lst = list(old_lst)	
	lst.reverse()
	delta = 1000 # Arbitratily large number
	index = len(lst)
	for i in range(len(lst)):
		diff = abs(abs(arg)-abs(lst[i]))
		if diff < delta:
			delta = diff
			index = (len(lst)-1)-i #since we have reversed the list but want to return the index in the "proper" list direction	
	return index

def get_settling_time(response, command, time, error_band):
	final_cmd = command[-1]
	initial_value = response[0]
	delta = abs(abs(final_cmd)-abs(initial_value))
	error_top = final_cmd + abs(error_band*delta)
	error_bottom = final_cmd - abs(error_band*delta)
	response_rev = list(response)
	response_rev.reverse()
	index = len(response)-1
	for i in range(len(response_rev)):
		if (response_rev[i] > error_top) or (response_rev[i] < error_bottom):
			index = len(response_rev)-1 -i
			break
	settled_time = time[index]
	return settled_time

def main():
	# Record a position
	raw_input("Move the robot to the position of interest and press enter")
	joint_positions = blu.get_joint_positions()
	print "Position recorded"

	# Return to the position every time you press enter
	done = False
	while not done:
		s = raw_input("Press enter to move and plot the data, or press d to exit without plotting")
		if s != "d":
			print "Please wait"
			blu.set_joint_positions(joint_positions)
			command, response = record_rxn()
			plot_response(command, response)
			blu.disable_control()
		else:
			done = True

if __name__ == "__main__":
	main()


