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


''' Code extract info

'''

# PARAMETERS USED TO COMPUTE 
path = sio.loadmat('NN_test.mat')


## parameters for LONGITUDINAL control
#  		input parameters: 	s0=s(0), v0=v(0), a_prev=a(-1), s_ref, v_ref
#  		output parameters:	a_pred; da_pred      (either can be used)

# vectors of dimension: #training
s_curr_orig = np.ravel(path['s_curr']) 	# s(0)
v_curr_orig = np.ravel(path['v_curr'])	# v(0)
a_prev_orig = np.ravel(path['a_prev'])	# a(-1)



# opt solutions
# matrices of dimension: #training x N_mpc
a_pred = path['a_pred']
da_pred = path['da_pred']

# extract tracking/ref parameters
s_ref_orig = path['s_ref']
v_ref_orig = path['v_ref']
# first elements needs to be removed since not used in MPC
s_ref = np.delete(s_ref_orig, 0, 1)  # delete first column of s_ref
v_ref = np.delete(v_ref_orig, 0, 1)  # delte first columns of s_ref

# stack all parameters 
s_curr = np.array([s_curr_orig])  	# allows for transpose of vector
v_curr = np.array([v_curr_orig]) 	# allows for transpose of vector
a_prev = np.array([a_prev_orig])	# allows for transpose

inputParam_long = np.hstack((s_curr.T, v_curr.T ,a_prev.T, s_ref, v_ref ))
outputParamAcc_long = a_pred
outputParamDacc_long = da_pred




####### parameters for LATERAL control
#  		input parameters: 	ey0=ey(0), epsi0=epsi(0), delta_prev=a(-1), v_pred, c_pred
#  		output parameters:	a_pred; da_pred      (either can be used)
# the data are collected row-wise (if matrix)

ey_curr_orig = np.ravel(path['ey_curr']) 	# s(0)
epsi_curr_orig = np.ravel(path['epsi_curr'])	# v(0)
df_prev_orig = np.ravel(path['df_prev'])	# a(-1)

# optimal solutions
# matrices of dimension: #training x N_mpc
df_pred = path['df_pred']
ddf_pred = path['ddf_pred']

# extract predicted velocity parameter
v_pred_orig = path['v_pred'] 	# contains v0
v_pred = np.delete(v_pred_orig, -1, 1)  # delete last column of v_pred, since not used in MPC
s_pred_orig = path['s_pred']	# needed to compute curvature
s_pred = np.delete(s_pred_orig, -1, 1)	# delete last column b/c not needed

numTrPts = np.size(df_pred,0)	# number of training points
N_mpc = np.size(df_pred,1)	# number of training points
k_coeffs = path['K_coeff']  # numTrPts x 4
# extract curvature
c_pred = np.zeros([numTrPts, N_mpc])


for i in range(numTrPts):
	for j in range(N_mpc):
		c_pred[i,j] = k_coeffs[i,0]*s_pred[i,j]**3 + k_coeffs[i,1]*s_pred[i,j]**2 + k_coeffs[i,2]*s_pred[i,j] + k_coeffs[i,3]


# stack all parameters 
ey_curr = np.array([ey_curr_orig])  	# allows for transpose of vector
epsi_curr = np.array([epsi_curr_orig]) 	# allows for transpose of vector
df_prev = np.array([df_prev_orig])	# allows for transpose

inputParam_lat = np.hstack((ey_curr.T, epsi_curr.T ,df_prev.T, v_pred, c_pred))
outputParamDf_lat = df_pred
outputParamDdf_lat = ddf_pred


### save the data ###
trainingDataDict = {}
trainingDataDict['inputParam_long'] = inputParam_long
trainingDataDict['outputParamAcc_long'] = outputParamAcc_long
trainingDataDict['outputParamDacc_long'] = outputParamDacc_long

trainingDataDict['inputParam_lat'] = inputParam_lat
trainingDataDict['outputParamDf_lat'] = outputParamDf_lat
trainingDataDict['outputParamDdf_lat'] = outputParamDdf_lat


sio.savemat('NN_test_trainingData.mat', trainingDataDict)


print('--- Done ---')


# use this to stop in work space
# 'c' = continue ; 'q' = quit
# pdb.set_trace()

# in this part of the code, we want to reconstruct 

# plt.figure()
# plt.plot(X_all,Y_all)
# for i in range(0,len(X_all),50):
# 	s0 = cdists[i]
# 	t0 = T_all[i]
# 	tEnd = t0 + N_mpc*dt_mpc
# 	tmp = np.abs(T_all-tEnd)
# 	closest_ind = np.argmin(tmp) 	# extract index along pat
# 	sEnd = cdists[closest_ind]
# 	s_max = sEnd - s0
# 	x_recon, y_recon, psi_recon = reconstruct_frenet(X_all[i], Y_all[i], Psi_all[i], s0, s_max, curv_coeff[i,:])
# 	# print(curv_coeff[i,:])
# 	# pdb.set_trace()
# 	plt.plot(x_recon, y_recon,'--',lw=2)
# plt.show()

# pdb.set_trace()





# ''' sample use
# 		python comp_curv.py --infile cpg_clean.mat --outfile cpg_clean_curv.mat
# '''
# if __name__ == "__main__":

# 	# 	python scripts/analysis/parse_bag.py -m Sim --infile bags/Frenet.bag --outfile paths/Frenet.mat
# 	# parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
# 	# parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
# 	# parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
# 	# parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
# 	# args = parser.parse_args()
# 	# parse_rosbag(args.mode, args.infile, args.outfile)

# 	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
# 	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: MAT File.')
# 	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: MAT File.')
# 	args = parser.parse_args()

# 	# define path Name
# 	# infile = 'cpg_clean.mat'
# 	# outfile = 'cpg_clean_curv.mat'
# 	comp_curvature(args.infile, args.outfile)


