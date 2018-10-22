import argparse
import scipy.io as sio
import rosbag
import numpy as np
import pdb
import math

''' Code to convert rosbag into a matfile for further plotting/analysis. 
	This code should be able to handle data recorded using either the state_est 
	and the state_est_dyn message types.
	GXZ: Code modified to also extract MPC solutions
'''
	
def parse_rosbag(mode, in_rosbag, out_mat):
	t = []; x = []; y = []; psi = []; v = []
	lat = []; lon = []; a = []; df = []
	se_v_x = []; se_v_y = []; se_yaw_rate = [];
	se_long_accel = []; se_lat_accel = [];
	
	# added by GXZ for policy learning (stores data from mpc solutions)
	s_ref = []; v_ref=[]; K_coeff = []; 
	a_pred = []; da_pred = []; df_pred = []; ddf_pred = [];
	solvStat_long = []; solvStat_lat = []; solvTime_long = []; solvTime_lat = [];
	ey_pred = []; epsi_pred = []; s_pred = []; v_pred = [];
	s_curr = []; v_curr = []; a_prev = []; ey_curr = []; epsi_curr = []; df_prev = [];

	b = rosbag.Bag(in_rosbag)
	state_est_topic_name = '/vehicle/state_est'
	mpc_path_topic_name  = '/vehicle/mpc_path'
	if '/vehicle/state_est_dyn' in  b.get_type_and_topic_info()[1].keys():
		state_est_topic_name = '/vehicle/state_est_dyn'
		mpc_path_topic_name  = '/vehicle/mpc_path_dyn'		
	
	if state_est_topic_name == '/vehicle/state_est_dyn':
		# Case 1: we have data collected with the dynamic model.  This is easier.
		for topic, msg, _ in b.read_messages(topics='/vehicle/state_est_dyn'):
			t.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
			x.append(msg.x)
			y.append(msg.y)
			psi.append(msg.psi)
			v.append(msg.v)

			lat.append(msg.lat)
			lon.append(msg.lon)

			a.append(msg.a)
			df.append(msg.df)

			se_v_x.append(msg.vx)
			se_v_y.append(msg.vy)
			se_yaw_rate.append(msg.wz)

			se_long_accel.append(msg.a_lon)
			se_lat_accel.append(msg.a_lat)
	else:
		# Case 2: we have data collected with the kinematic model.  We'll need to add some dynamic info
		# if this was run as a real experiment (not simulated!).
		for topic, msg, _ in b.read_messages(topics='/vehicle/state_est'):		
			t.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
			x.append(msg.x)
			y.append(msg.y)
			psi.append(msg.psi)
			v.append(msg.v)
	
			lat.append(msg.lat)
			lon.append(msg.lon)
	
			a.append(msg.a)
			df.append(msg.df)

		####### added by GXZ to extract data for learning  #######
		for topic, msg, _ in b.read_messages(topics='/vehicle/mpc_path'):
			# collect solver time and status
			solvStat_long.append(msg.solv_status_long)
			solvStat_lat.append(msg.solv_status_lat)
			solvTime_long.append(msg.solv_time_long)
			solvTime_lat.append(msg.solv_time_lat)
			s_ref.append(msg.sr)
			v_ref.append(msg.vr)
			K_coeff.append(msg.curv)
			a_pred.append(msg.acc)
			da_pred.append(msg.dacc)
			df_pred.append(msg.df)
			ddf_pred.append(msg.ddf)
			ey_pred.append(msg.eys_fren)
			epsi_pred.append(msg.epsis_fren)
			s_pred.append(msg.ss_fren)
			v_pred.append(msg.vs_fren)
			s_curr.append(msg.s_curr)
			v_curr.append(msg.v_curr)
			a_prev.append(msg.a_prev)
			ey_curr.append(msg.ey_curr)
			epsi_curr.append(msg.epsi_curr)
			df_prev.append(msg.df_prev)

			# print('append inside mpc_path')
			# pdb.set_trace()	
		
		if mode != 'Sim': # only populate dynamic fields if we have real data from the vehicle.
			tm = []
			lat_accel = []
			long_accel = []
			yaw_rate = []
			for topic, msg, _ in b.read_messages(topics='/vehicle/imu'):
				tm.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
				lat_accel.append(msg.lat_accel)
				long_accel.append(msg.long_accel)
				yaw_rate.append(math.radians(msg.yaw_rate))
		
			se_lat_accel = np.interp(t, tm, lat_accel)
			se_long_accel = np.interp(t, tm, long_accel)
			se_yaw_rate = np.interp(t, tm, yaw_rate)
		
			tm = []
			v_east = []
			v_north = []
			for topic, msg, _ in b.read_messages(topics='/gps/vel'):
				tm.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
				v_east.append(msg.twist.twist.linear.x)
				v_north.append(msg.twist.twist.linear.y)
	
			se_v_east  = np.interp(t, tm, v_east)
			se_v_north = np.interp(t, tm, v_north)

			# Rotate axes by psi to go from EN frame to XY frame
			se_v_x = []
			se_v_y = []	
			for i in range(len(t)):
				v_x = np.cos(psi[i]) * se_v_east[i] + np.sin(psi[i]) * se_v_north[i]
				v_y = -np.sin(psi[i]) * se_v_east[i] + np.cos(psi[i]) * se_v_north[i]
				se_v_x.append(v_x)
				se_v_y.append(v_y)
	
	# Estimate controller enable time by using the first optimal solution 
	# from the MPC module, based on the MPC command/path message.
	# TODO: alternatively use ada_stat/acc_mode status information to see 
	# if the controller is enabled or not.
	t_enable = None
	for topic, msg, _ in b.read_messages(topics=mpc_path_topic_name):
		if msg.solv_status_long == 'Optimal':
			t_enable = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
			break				
	
	# Some notes on the resulting output data.
	# If simulated data, lat/lon will just be an array of 0's.
	# If kinematic + simulated data, entries wz, vz, vy, a_long, a_lon will just be empty lists.
	# If this is a path recorded and not a path following experiment, t_enable will not be in the data.
	rdict = {}
	rdict['mode'] = mode
	rdict['t']   = t
	if t_enable != None:
		rdict['t_en'] = t_enable

	rdict['lat'] = lat
	rdict['lon'] = lon
	
	rdict['x']   = x
	rdict['y']   = y
	rdict['psi'] = psi
	rdict['v']   = v
	rdict['wz'] = se_yaw_rate
	rdict['vx'] = se_v_x
	rdict['vy'] = se_v_y
	
	rdict['a']   = a
	rdict['a_lat'] = se_lat_accel
	rdict['a_long'] = se_long_accel
	rdict['df']  = df

	rdict['solvStat_long'] = solvStat_long
	rdict['solvStat_lat'] = solvStat_lat
	rdict['solvTime_long'] = solvTime_long
	rdict['solvTime_lat'] = solvTime_lat
	rdict['s_ref'] = s_ref
	rdict['v_ref'] = v_ref
	rdict['K_coeff'] = K_coeff
	rdict['a_pred'] = a_pred
	rdict['da_pred'] = da_pred
	rdict['df_pred'] = df_pred
	rdict['ddf_pred'] = ddf_pred
	rdict['ey_pred'] = ey_pred
	rdict['epsi_pred'] = epsi_pred
	rdict['s_pred'] = s_pred
	rdict['v_pred'] = v_pred
	rdict['s_curr'] = s_curr
	rdict['v_curr'] = v_curr
	rdict['a_prev'] = a_prev
	rdict['ey_curr'] = ey_curr
	rdict['epsi_curr'] = epsi_curr
	rdict['df_prev'] = df_prev

				
	sio.savemat(out_mat, rdict)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
	args = parser.parse_args()
	parse_rosbag(args.mode, args.infile, args.outfile)
