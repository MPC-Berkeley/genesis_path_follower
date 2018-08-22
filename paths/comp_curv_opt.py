# python code to compute (s, c(s)) along a reference trajectory
# code can be improved, but not too bad (in the author's humble opinion)

# this script loads loads Nitin's curvature profile into mine

import scipy.io as sio
import numpy as np
import matplotlib.pyplot as plt
import pdb	# import debugger
import math
import argparse


# function which, plot the (X,Y)-cord from frenet coordinates

def reconstruct_frenet(x0, y0, p0, s_array, curv_array):
	xr = [x0] # initial x (s=0)
	yr = [y0] # initial y (s=0)
	pr = [p0] # initial heading (atan2(dy, dx) evaluated at s = 0)
	s_interp = np.arange(s_array[0], s_array[-1], 0.1) # s interpolation range
	# pdb.set_trace()
	k_interp = np.interp(s_interp, s_array, curv_array)


	for i in range(len(s_interp)-1):
		ds = s_interp[i+1] - s_interp[i]
		xn = xr[-1] + ds * math.cos(pr[-1])    
		yn = yr[-1] + ds * math.sin(pr[-1])
		k = k_interp[i]
		pn = pr[-1] + ds * k
		xr.append(xn)
		yr.append(yn)
		pr.append(pn)

	skip = int(round(len(xr)/10))
	return xr[0::max(skip,1)], yr[0::max(skip,1)], pr[0::max(skip,1)] # return reconstructed path with heading



''' Code to compute curvature 
loads data from "infile"
saves data with curvature profile in "outfile"

paramaters that can be used to configure stuff
	"dt"	:	discretization of MPC controller
	"N"		: 	discretization of MPC controller
'''

def comp_curvature_opt():
	# this code literally take's Nitin's 
	dt_mpc = 0.1
	N_mpc = 16			# could be longer in practice

	# define gps-resolution
	dt_gps = 0.01 	# sampling time [s] of data; 
					# 0.01 for new GPS on Black Genesis
					# 0.07 for Azera data

	# load reference path (GPS or local coordinates)
	path = sio.loadmat("slow_lap_8_20.mat")
	outfile = "slow_lap_8_20_curv.mat"      # max for slow lap < 15m/2

	# over the horizon max vel is 152/s
	# at dt_mpc*N_mpc, that's at most 1.6*15m/s <= 20m look-ahead
	# predict c(s) in discrete (Euler steps) of 0.25m --> 80 steps
	s_max = 20	# look ahead 20m
	ds = 0.5	# store curvature value every 0.5m

	# access to dictionary
	# path.keys()	# shows all keys
	# path['a']
	# path['a'].shape
	# path['test_key'] = 'vijay' 	# add new keys
	# sio.savemat(path, save_dir)

	# extract (X,Y,Psi,t) coordinates
	X_all = np.ravel(path['x'])	# same as X_tmp = path['x'], X_all = X_tmp[0]
	Y_all = np.ravel(path['y'])
	Psi_all = np.ravel(path['psi'])
	cdists = []		# will collect all 's', stands for cummulative distance (Vijay)
	Curv_all = []	# collects all curvature
	T_all = np.ravel(path['t'])

	# plot path figure
	# plt.figure()
	# plt.plot(X_all,Y_all)
	# plt.axis('equal')
	# plt.show()


	# compute "s" along path
	# data points are about 1m apart from each other
	for i in range(len(X_all)): # loop over reference path
		# cdists = "s" ; unit of "s" is m
		if len(cdists) == 0: 		# i.e. the first point on the trajectory
			cdists.append(0.0)	# s = 0
		else:					# later points on the trajectory
			d = math.sqrt( (X_all[i] - X_all[i-1])**2 + (Y_all[i] - Y_all[i-1])**2 ) + cdists[-1]
			cdists.append(d) 	# s = s_prev + dist(z[i], z[i-1])

	# plt.figure()
	# plt.plot(cdists)
	# ylabel('s')
	# plt.show()

	# compute raw curvature data
	ds_tmp = []	# can be deleted, just for debugging purposes
	# compute Curv (along path)
	for i in range(len(cdists)-1):
		# by def, Psi'(s) = c(s) * s'	
		# approximate this using finite difference
		delta_s = cdists[i+1]-cdists[i]
		delta_Psi = Psi_all[i+1]-Psi_all[i]
		if delta_s == 0:	# catch division by 0
			print('1st time delta_s = 0  -- fixed!')
			print(i)
			delta_s = cdists[i+2]-cdists[i]
			delta_Psi = Psi_all[i+2]-Psi_all[i]
			if delta_s == 0:
				print('2nd time delta_s = 0 -- NOT GOOD / NEED FIX')
				print(i)
		curv = delta_Psi / delta_s 	# compute curvature
		if abs(curv) > 0.2:	# if curvature too large --> numerical errors
			curv = 0
		Curv_all.append(curv)
		ds_tmp.append(delta_s)
	# append last element so that all vertices of same 
	Curv_all.append(0)
	ds_tmp.append(0)	
	print('-- done computing (raw) Curv_all --')

	Curv_all[-1-300:-1] = np.zeros(300) 	# remove chattering 

	plt.figure()
	plt.title('Raw curv data')
	plt.plot(cdists,Curv_all,'ob')
	plt.xlabel('s')
	plt.ylabel('c(s)')
	plt.show()


	#### start computing curvature
	for i in range(len(cdists)-1000):
		s_curr = cdists








	pdb.set_trace()




	# pdb.set_trace()


	# store curvature profile 
	path['curv'] = curv_Nitin_interp
	path['cdists'] = cdists
 	# sio.savemat(outfile, path)

	# pdb.set_trace()

	plt.figure()
	plt.plot(X_all,Y_all)
	for i in range(0,len(X_all),50):
		s0 = cdists[i]
		t0 = T_all[i]
		tEnd = t0 + N_mpc*dt_mpc
		tmp = np.abs(T_all-tEnd)
		closest_ind = np.argmin(tmp) 	# extract index along pat
		sEnd = cdists[closest_ind]
		s_array = cdists[i:closest_ind] 	# cut out relevant piece of track
		curv_array = curv_Nitin_interp[i:closest_ind]  # cut out relevant 
		# s_max = sEnd - s0
		x_recon, y_recon, psi_recon = reconstruct_frenet(X_all[i], Y_all[i], Psi_all[i], s_array, curv_array)
		# print(curv_coeff[i,:])
		# pdb.set_trace()
		plt.plot(x_recon, y_recon,'--',lw=2)
	plt.show()

	pdb.set_trace()


	
''' sample use
		python comp_curv.py --infile cpg_clean.mat --outfile cpg_clean_curv.mat
'''
if __name__ == "__main__":

	comp_curvature_opt()


