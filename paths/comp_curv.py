# python code to compute (s, c(s)) along a reference trajectory
# code can be improved, but not too bad (in the author's humble opinion)

# sample use
# 	python comp_curv.py --infile slow_lap_8_20.mat --outfile slow_lap_8_20_curv.mat


import scipy.io as sio
import numpy as np
import matplotlib.pyplot as plt
import pdb	# import debugger
import math
import argparse

# aux function that 
def cubic_func(t,a3,a2,a1,a0):              # f(t)
	return a0 + a1*t + a2*t**2 + a3*t**3

# function which, plot the (X,Y)-cord from frenet coordinates
def reconstruct_frenet(x0, y0, p0, s_0, s_max, K_coeff):
	xr = [x0] # initial x (s=0)
	yr = [y0] # initial y (s=0)
	pr = [p0] # initial heading (atan2(dy, dx) evaluated at s = 0)

	s_interp = np.arange(0, s_max, 0.1) # s interpolation range

	for i in range(len(s_interp) - 1):  # integrate ds to get the trajectory
		# Update position
		ds = s_interp[i+1] - s_interp[i]                
		xn = xr[-1] + ds * math.cos(pr[-1])    
		yn = yr[-1] + ds * math.sin(pr[-1])

        # Update heading
		k = cubic_func(s_0+s_interp[i], *K_coeff) # compute curvature at s
		pn = pr[-1] + ds * k
		# pdb.set_trace()
		xr.append(xn)
		yr.append(yn)
		pr.append(pn)

	skip = int(round(len(xr)/10))
	# use numpy slicing: [start:stop:step]; e.g. xs[1::4]
	# pdb.set_trace()

	return xr[0::max(skip,1)], yr[0::max(skip,1)], pr[0::max(skip,1)] # return reconstructed path with heading




''' Code to compute curvature 
loads data from "infile"
saves data with curvature profile in "outfile"

paramaters that can be used to configure stuff
	"dt"	:	discretization of MPC controller
	"N"		: 	discretization of MPC controller
'''

def comp_curvature(infile, outfile):
	# PARAMETERS USED TO COMPUTE 
	# HOW FAR AHEAD LOOK???????????
	dt_mpc = 0.2
	N_mpc = 8		# should be longer in practice

	# define gps-resolution
	dt_gps = 0.01 	# sampling time [s] of data; 
					# 0.01 for new GPS on Black Genesis
					# 0.07 for Azera data


	# load reference path (GPS or local coordinates)
	path = sio.loadmat(infile)
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

	# tmp = np.diff(T_all)
	# max(tmp)
	# min(tmp)
	# np.mean(tmp)


	pdb.set_trace()
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


	# heuristic: remove the jumps in the last few c(s)
	# useful for RFS when car comes to stop
	Curv_all[-1-300:-1] = np.zeros(300)

	# plt.figure()
	# plt.title('Raw curv data')
	# plt.plot(cdists,Curv_all,'o')
	# plt.xlabel('s')
	# plt.ylabel('c(s)')
	# plt.show()

	# plt.figure()
	# plt.plot(cdists,ds_tmp,'o')
	# plt.show()

	# pdb.set_trace()


	# this works
	# idxS0 = 0	# index associated with S0
	# tm_interp = np.arange(T_all[idxS0], T_all[idxS0] + N_mpc*dt_mpc, 0.01)	# interpolate time, WHAT IS A GOOD DISCRETIZATION? 10ms for now
	# s_interp = np.interp(tm_interp, T_all, cdists)		# obtain corresponding 's'; s_des   = f_interp(t_des, t_actual, s_actual)
	# curv_interp = np.interp(tm_interp, T_all, Curv_all)
	# curv_coeff = np.polyfit(s_interp, curv_interp, 3)	# get coefficient, starts from highest order
	# # plot empirically the curvature
	# curv_fit = []
	# for i in range(len(s_interp)):
	# 	tmp = 0
	# 	poly_order = len(curv_coeff)-1
	# 	for j in range(len(curv_coeff)):
	# 		tmp = tmp + curv_coeff[j]*s_interp[i]**(poly_order-j)
	# 	curv_fit.append(tmp)
	# plt.figure()
	# plt.plot(s_interp, curv_interp,'bx')
	# plt.plot(s_interp, curv_fit, 'ro')
	# plt.show()

	# this generalizes
	poly_order = 3	# order of fit
	# define set of curvature coefficients
	curv_coeff = np.zeros((len(cdists),poly_order+1))				# this guy stores all the curv_coeff along 's'; 0 by default (straight)
	curv_fit = np.empty((len(cdists), int(math.ceil(N_mpc*dt_mpc/dt_gps)) ))	# this is for plotting/verification purposes
	s_fit = np.empty((len(cdists), int(math.ceil(N_mpc*dt_mpc/dt_gps)) ))		# this is for plotting/verification purposes

	print('-- start fitting  --')


	# tm_length = []
	t_interpOffset = 1*dt_mpc	# offset used to add chunks to beginning and end for interpolation
	for ii in range( int(len(cdists) - round(2*(N_mpc*dt_mpc)/dt_gps)) ) : # loop over index of 
		if ii%1000 == 0:
			print(ii)
		ii_interp = max(0,ii - int(t_interpOffset/dt_gps))	# look ahead 0.1 sec (/0.01 because data comes in every 0.01s)
		# data is sampled on average every dt_gps (e.g. 10ms = 0.01 for Black Genesis)
		tm_interp = np.arange(T_all[ii_interp], T_all[ii_interp] + N_mpc*dt_mpc+t_interpOffset, dt_gps)	# interpolate time, WHAT IS A GOOD DISCRETIZATION? 10ms for now
		tm_fit = np.arange(T_all[ii], T_all[ii] + N_mpc*dt_mpc, dt_gps)	# Helper for checking fit (over MPC horizon)

		# tm_length.append(len(tm_interp))
		s_interp = np.interp(tm_interp, T_all, cdists)		# obtain corresponding 's'; s_des   = f_interp(t_des, t_actual, s_actual)
		s_fit_tmp = np.interp(tm_fit, T_all, cdists)		# Helper for checking fit (only over MPC horizon)

		curv_interp = np.interp(tm_interp, T_all, Curv_all)
		curv_coeff_tmp = np.polyfit(s_interp, curv_interp, poly_order)	# get coefficient, starts from highest order
		curv_coeff[ii,:] = curv_coeff_tmp	# 
		# plot empirically the curvature
		# curv_fit_tmp = []
		for i in range(len(s_fit_tmp)):
			tmp = 0	# stupid evaluation of polynomial (I'm sure there are much better ways)
			for j in range(len(curv_coeff_tmp)):
				tmp = tmp + curv_coeff_tmp[j]*s_fit_tmp[i]**(poly_order-j)

			curv_fit[ii,i] = tmp
			# curv_fit_tmp.append(tmp)


		# now save the s and curv_fit
		# curv_fit[ii,:] = curv_fit_tmp
		s_fit[ii,:] = s_fit_tmp

	# plt.figure()
	# plt.plot(s_interp, curv_interp,'bx')
	# plt.plot(s_interp, curv_fit, 'ro')
	# plt.show()

	plt.figure()
	plt.plot(cdists,Curv_all)
	# plt.plot(s_fit.T,curv_fit.T)	# plots column-wise
	for i in range(0,len(cdists),10):
		plt.plot(s_fit[i,:],curv_fit[i,:],lw=2)
	plt.show()

	# pdb.set_trace()

	# save c(s) coeff to path
	path['curv'] = curv_coeff
	path['cdists'] = cdists
	sio.savemat(outfile, path)



	# use this to stop in work space
	# 'c' = continue ; 'q' = quit
	# pdb.set_trace()

	# in this part of the code, we want to reconstruct 

	plt.figure()
	plt.plot(X_all,Y_all)
	for i in range(0,len(X_all),50):
		s0 = cdists[i]
		t0 = T_all[i]
		tEnd = t0 + N_mpc*dt_mpc
		tmp = np.abs(T_all-tEnd)
		closest_ind = np.argmin(tmp) 	# extract index along pat
		sEnd = cdists[closest_ind]
		s_max = sEnd - s0
		x_recon, y_recon, psi_recon = reconstruct_frenet(X_all[i], Y_all[i], Psi_all[i], s0, s_max, curv_coeff[i,:])
		# print(curv_coeff[i,:])
		# pdb.set_trace()
		plt.plot(x_recon, y_recon,'--',lw=2)
	plt.show()

	pdb.set_trace()





''' sample use
		python comp_curv.py --infile cpg_clean.mat --outfile cpg_clean_curv.mat
'''
if __name__ == "__main__":

	# 	python scripts/analysis/parse_bag.py -m Sim --infile bags/Frenet.bag --outfile paths/Frenet.mat
	# parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	# parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
	# parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
	# parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
	# args = parser.parse_args()
	# parse_rosbag(args.mode, args.infile, args.outfile)

	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: MAT File.')
	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: MAT File.')
	args = parser.parse_args()

	# define path Name
	# infile = 'cpg_clean.mat'
	# outfile = 'cpg_clean_curv.mat'
	comp_curvature(args.infile, args.outfile)


