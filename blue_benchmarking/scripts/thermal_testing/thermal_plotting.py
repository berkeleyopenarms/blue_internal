import pickle
import sys
import matplotlib.pyplot as plt

inputs = sys.argv[1::]

for i in range(len(inputs)):
	file_name = str(inputs[i])

	data = pickle.load( open(file_name + ".p", "rb"))
	time_data = data[0]
	temp_data = data[1]

	# Remove 0's from data
	for j in range(len(temp_data)):
		if temp_data[j] == 0 and j != 0:
			temp_data[j] = temp_data[j-1]

	print("Plot 'em")
	plt.plot(time_data, temp_data)
	plt.xlabel("Time [s]")
	plt.ylabel("Temperature [C]")
	plt.title("Temperature vs. Time for Stalled Link at %s" % file_name)

plt.legend(inputs)
plt.show()
print("Plotted 'em")
