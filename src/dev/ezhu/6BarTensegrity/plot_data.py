# This script can be used to plot the data gathered from simulations

import sys
import matplotlib.pyplot as plt
import numpy as np
import csv

# Parse input argument
filename = sys.argv[1]
print ('Plotting data from:', filename)

# Open csv file
file = csv.DictReader(open(filename))

SimTime = []
CoM_posX = []
CoM_posY = []
CoM_posZ = []
CoM_velX = []
CoM_velY = []
CoM_velZ = []
onGround = []
contactCounter = []

for row in file:
	SimTime.append(float(row["SimTime"]))
	CoM_posX.append(float(row["CoM_posX"]))
	CoM_posY.append(float(row["CoM_posY"]))
	CoM_posZ.append(float(row["CoM_posZ"]))
	CoM_velX.append(float(row["CoM_velX"]))
	CoM_velY.append(float(row["CoM_velY"]))
	CoM_velZ.append(float(row["CoM_velZ"]))
	onGround.append(int(row["onGround"]))
	contactCounter.append(int(row["contactCounter"]))

data_size = len(SimTime)
# print(data_size)

plt.figure(1)
plt.subplot(311)
plt.plot(SimTime,CoM_posX)
plt.title("CoM Position")
plt.ylabel("X")
plt.subplot(312)
plt.plot(SimTime,CoM_posY)
plt.ylabel("Y")
plt.subplot(313)
plt.plot(SimTime,CoM_posZ)
plt.ylabel("Z")

plt.figure(2)
plt.subplot(311)
plt.plot(SimTime,CoM_velX)
plt.title("CoM Velocity")
plt.ylabel("X")
plt.subplot(312)
plt.plot(SimTime,CoM_velY)
plt.ylabel("Y")
plt.subplot(313)
plt.plot(SimTime,CoM_velZ)
plt.ylabel("Z")

# contactStart = onGround.index(1)+1
# contactEnd = onGround[contactStart:].index(0)+contactStart+1
#
# plt.figure(3)
# plt.subplot(211)
# plt.plot(SimTime[contactStart:contactEnd],CoM_posY[contactStart:contactEnd])
# plt.ylabel("Y")
# plt.subplot(212)
# plt.plot(SimTime[contactStart:contactEnd],CoM_velY[contactStart:contactEnd])
# plt.ylabel("Vy")

plt.show()
