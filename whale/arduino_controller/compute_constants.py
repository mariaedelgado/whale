#!/usr/bin/python

import sys
import numpy as np

v = []
t = []

command = raw_input("Enter command (F = forward / V = reverse / L = left / R = right): ")

it = 0
while it <= 0:

	if(command == "L" or command == "R"):
	
		theta_deg = input("Enter turned angle (in degrees): ")
		time = input("Enter time elapsed to turn the angle (in seconds): ")
		theta_rad = (theta_deg*np.pi)/180
		w = float(theta_rad)/float(time)
		vel = w*0.1
		print("Angle (deg): " + str(theta_deg) + " Angle (rad): " + str(theta_rad) + " Time: " + str(time) + " Vel ang: " + str(w) + " Vel: " + str(vel))
		v.append(vel)
		t.append(time)

	elif(command == "F" or command == "V"):

		pos = input("Enter position increment (in m): ")
		time = input("Enter time elapsed to change position (in seconds): ")
		vel = float(pos)/float(time)
		print("Pos: " + str(pos) + " Time: " + str(time) + " Vel: " + str(vel))
		v.append(vel)
		t.append(time)

	else:
		print("Wrong command")
		break

	it = it + 1

if v and t:
	ct = float(v[0])/float(pow(t[0], 2))
	print("Result for command " + command + " is CT1 = " + str(ct) )
	##A = np.array([[pow(t[0], 3), pow(t[0], 2)], [pow(t[1], 3), pow(t[1], 2)]])
	##B = np.array([v[0], v[1]])
	##x = np.linalg.solve(A,B)

	#print("Result for command " + command + " is CT1 = " + str(x[0]) + " CT2 = " + str(x[1]) )
	print("Bye")

