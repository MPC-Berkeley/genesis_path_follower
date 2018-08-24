import argparse
import scipy.io as sio
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import math
import scipy.spatial.distance as ssd
import pdb

# Compute cumulative distance along an array of waypoints, aka s.
def compute_path_s(xy_arr):
	cdists = []
	for i in range(xy_arr.shape[0]):
		if i == 0:
			cdists.append(0.0)	# s = 0
		else:
			dx = xy_arr[i,0] - xy_arr[i-1,0]
			dy = xy_arr[i,1] - xy_arr[i-1,1]
			d = math.sqrt( dx**2 + dy**2 ) + cdists[-1]
			cdists.append(d) 	# s = s_prev + dist(z[i], z[i-1])
	return np.array(cdists)

def compute_lateral_error(xy_query, xy_path_prev, xy_path_next):
	'''
	Assumptions: path is locally linear.  In this region, it is represented 
	as a vector starting at xy_path_prev and ending at xy_path_next.
	Let v1 = xy_path_next-xy_path_prev
	    v2 = v2 = xy_query - xy_path_prev
	We can use the cross product v1 x v2 to find the area of the parallelogram spanned by these vectors.

	Dividing by the norm of v1 (i.e. the "base") gives the height, aka lateral error.
	The sign of the cross product gives the sign of the lateral error, as well.
	# Source: https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
	'''
	v1 = xy_path_next - xy_path_prev
	v2 = xy_query - xy_path_prev

	lat_error = np.cross(v1,v2)/np.linalg.norm(v1)
	return lat_error

# Find the closest point and corresponding distance/lateral error between two arrays of waypoints.
def compute_path_errors(xy_ref, xy_actual):
	# Input: m x n X1 and p x n X2 matrices.
	# Output: m x p matrix D with D[i,j] = dist(X1[i,:], X2[j,:])
	errors = np.ones(xy_actual.shape[0]) * np.nan
	lat_errors = np.ones(xy_actual.shape[0]) * np.nan
	closest_pt_ind = np.ones(xy_actual.shape[0]) * np.nan

	for st_ind in np.arange(0, xy_actual.shape[0], 1000):
		print '%d of %d' % (st_ind, xy_actual.shape[0])
		end_ind = min(xy_actual.shape[0], st_ind + 1000)
		dist = ssd.cdist(xy_ref, xy_actual[st_ind:end_ind])
		closest_pt_ind[st_ind:end_ind] = np.argmin(dist, axis=0)
		errors[st_ind:end_ind] = dist[ closest_pt_ind[st_ind:end_ind].astype(int) , np.arange(dist.shape[1]) ]

	closest_pt_ind = closest_pt_ind.astype(int)
	
	penultimate_pr_ind = xy_ref.shape[0] - 2
	for i in np.arange(0, xy_actual.shape[0]):
		# TODO: could try vectorizing this.  
		# This loop is reasonably fast for the paths we have though.
		if closest_pt_ind[i] >= penultimate_pr_ind:
			continue
			
		lat_errors[i] = compute_lateral_error(xy_actual[i], xy_ref[closest_pt_ind[i]], xy_ref[closest_pt_ind[i]+1])

	return closest_pt_ind, errors, lat_errors

# Fix heading variations over +/-pi.
def fix_heading(psi_arr):
		for i in range(len(psi_arr)):
			p = psi_arr[i]
			psi_cands_arr = np.array([p, p + 2*np.pi, p - 2*np.pi])
			best_cand = np.argmin(np.fabs(psi_cands_arr))
			psi_arr[i] = psi_cands_arr[best_cand]

		return psi_arr


# Main function to find tracking errors between a recorded and followed trajectory.
def get_errors(pr_matfile, pf_matfile, bagfile, st_ind=None, end_ind=None):
	mat_pr = sio.loadmat(pr_matfile)
	mat_pf = sio.loadmat(pf_matfile)

	if st_ind == None:
		if 't_en' in mat_pf.keys():
			diff_t = np.ravel(mat_pf['t'] - mat_pf['t_en'])
			st_ind = np.argmin(np.square(diff_t))
		else:
			st_ind = 0

	if end_ind == None:
		end_ind = len(np.ravel(mat_pf['t']))

	xy_pr = np.column_stack((np.ravel(mat_pr['x']),np.ravel(mat_pr['y'])))
	xy_pf = np.column_stack((np.ravel(mat_pf['x']),np.ravel(mat_pf['y'])))[st_ind:end_ind]
	v_pr = np.ravel(mat_pr['v']);       v_pf = np.ravel(mat_pf['v'])[st_ind:end_ind]
	p_pr = np.ravel(mat_pr['psi']);     p_pf = np.ravel(mat_pf['psi'])[st_ind:end_ind]
	a_pr = np.ravel(mat_pr['a']);       a_pf = np.ravel(mat_pf['a'])[st_ind:end_ind]
	df_pr = np.ravel(mat_pr['df']);     df_pf = np.ravel(mat_pf['df'])[st_ind:end_ind]
	
	s_pr = compute_path_s(xy_pr)

	# START INTERPOLATION
	# Discretize the path finely to help with analysis.
	path_disc = 0.05 # m
	s_interp = np.arange(0.0, s_pr[-1], path_disc)
	x_pr = np.interp(s_interp, s_pr, xy_pr[:,0])
	y_pr = np.interp(s_interp, s_pr, xy_pr[:,1])
	p_pr = np.interp(s_interp, s_pr, p_pr)
	a_pr = np.interp(s_interp, s_pr, a_pr)
	v_pr = np.interp(s_interp, s_pr, v_pr)
	df_pr = np.interp(s_interp, s_pr, df_pr)
	s_pr = s_interp
	xy_pr = np.column_stack((x_pr, y_pr))
	# END OF INTERPOLATION

	closest_ind_xy, error_xy, lat_error_xy = compute_path_errors(xy_pr, xy_pf)
	#################################################
	fig = plt.figure()
	plt.subplot(711)		
	plt.plot(xy_pr[:,0], xy_pr[:,1], 'k')
	plt.plot(xy_pf[:,0], xy_pf[:,1], 'r')
	plt.plot(xy_pr[0,0], xy_pr[0,1], 'bo')
	plt.plot(xy_pr[-1,0], xy_pr[-1,1], 'go')
	plt.xlabel('X (m)')
	plt.ylabel('Y (m)')
	plt.axis('equal')	

	plt.subplot(712)		
	plt.plot(s_pr, xy_pr[:,0], 'k', label='Recorded')
	plt.plot(s_pr[closest_ind_xy], xy_pf[:,0], 'r', label='Followed')
	plt.xlabel('S (m)')
	plt.ylabel('X (m)')

	plt.subplot(713)		
	plt.plot(s_pr, xy_pr[:,1], 'k')
	plt.plot(s_pr[closest_ind_xy], xy_pf[:,1], 'r')
	plt.xlabel('S (m)')
	plt.ylabel('Y (m)')

	plt.subplot(714)		
	plt.plot(s_pr, fix_heading(p_pr), 'k')
	plt.plot(s_pr[closest_ind_xy], fix_heading(p_pf), 'r')
	plt.xlabel('S (m)')
	plt.ylabel('Psi (rad)')
	
	plt.subplot(715)		
	plt.plot(s_pr, v_pr, 'k')
	plt.plot(s_pr[closest_ind_xy], v_pf, 'r')
	plt.xlabel('S (m)')
	plt.ylabel('v (m/s)')
	
	## LOAD MPC Cmds if Available.
	
	a_pf_mpc = np.array([])
	df_pf_mpc = np.array([])

	if bagfile != "":
		b = rosbag.Bag(bagfile)	
		t_a_mpc = []; a_mpc = [];  # /control/accel:	   Acceleration Command
		t_df_mpc = []; df_mpc = [] # /control/steer_angle: Steering Command
		
		for topic, msg, t in b.read_messages(topics='/control/accel'):
			t_a_mpc.append(t.secs + 1e-9 * t.nsecs)
			a_mpc.append(msg.data)
		for topic, msg, t in b.read_messages(topics='/control/steer_angle'):
			t_df_mpc.append(t.secs + 1e-9 * t.nsecs)
			df_mpc.append(msg.data)

		t_pf = np.ravel(mat_pf['t'])[st_ind:end_ind]

		a_pf_mpc = np.interp(t_pf, t_a_mpc, a_mpc)
		df_pf_mpc = np.interp(t_pf, t_df_mpc, df_mpc)

	l1 = None; l2 = None; l3 = None
	plt.subplot(716)		
	l1, = plt.plot(s_pr, a_pr, 'k')
	l2, =plt.plot(s_pr[closest_ind_xy], a_pf, 'r')
	if a_pf_mpc.size > 0:
		l3, = plt.plot(s_pr[closest_ind_xy], a_pf_mpc, 'b')

	plt.xlabel('S (m)')
	plt.ylabel('a (m/s)')

	if l3 != None:
		fig.legend((l1,l2,l3), ('Recorded', 'Followed', 'MPC Cmd'), 'right')
	else:
		fig.legend((l1,l2), ('Recorded', 'Followed'), 'upper right')
	
		
	plt.subplot(717)		
	plt.plot(s_pr, df_pr, 'k')
	plt.plot(s_pr[closest_ind_xy], df_pf, 'r')
	if df_pf_mpc.size > 0:
		plt.plot(s_pr[closest_ind_xy], df_pf_mpc, 'b')
	plt.xlabel('S (m)')
	plt.ylabel('df (rad)')
	#################################################
	fig = plt.figure()
	plt.subplot(511)
	#plt.plot(s_pr[closest_ind_xy], error_xy, 'r')
	plt.plot(s_pr[closest_ind_xy], lat_error_xy, 'b')
	plt.xlabel('S (m)')
	plt.ylabel('Path (m)')

	plt.subplot(512)
	ev = v_pr[closest_ind_xy] - v_pf
	plt.plot(s_pr[closest_ind_xy], ev, 'r')
	plt.xlabel('S (m)')
	plt.ylabel('Velocity (m/s)')

	plt.subplot(513)
	ep = fix_heading(p_pr[closest_ind_xy] - p_pf)
	plt.plot(s_pr[closest_ind_xy], ep, 'r')
	plt.xlabel('S (m)')
	plt.ylabel('Yaw (rad)')

	l1 = None; l2 = None; l3 = None
	plt.subplot(514)		
	l1, = plt.plot(s_pr, a_pr, 'k')
	l2, =plt.plot(s_pr[closest_ind_xy], a_pf, 'r')
	if a_pf_mpc.size > 0:
		l3, = plt.plot(s_pr[closest_ind_xy], a_pf_mpc, 'b')

	plt.xlabel('S (m)')
	plt.ylabel('a (m/s)')

	if l3 != None:
		fig.legend((l1,l2,l3), ('Recorded', 'Followed', 'MPC Cmd'), 'right')
	else:
		fig.legend((l1,l2), ('Recorded', 'Followed'), 'upper right')
			
	plt.subplot(515)		
	plt.plot(s_pr, df_pr, 'k')
	plt.plot(s_pr[closest_ind_xy], df_pf, 'r')
	if df_pf_mpc.size > 0:
		plt.plot(s_pr[closest_ind_xy], df_pf_mpc, 'b')
	plt.xlabel('S (m)')
	plt.ylabel('df (rad)')
	

	plt.suptitle('Tracking Error Over Reference Path')
	#################################################
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Get path tracking performance of MPC controller trajectory.')
	parser.add_argument('--pr',  type=str, required=True, help='Recorded Trajectory Reference Matfile.')
	parser.add_argument('--pf', type=str, required=True, help='MPC Followed Trajectory Matfile.')	
	parser.add_argument('--bf',  type=str, required=False, default="", help='Bag file to plot MPC commands.')
	args = parser.parse_args()
	get_errors(args.pr, args.pf, args.bf)
