import argparse
import scipy.io as sio
import rosbag
import numpy as np
import pdb
import math

''' Code to convert rosbag into a matfile for further plotting/analysis '''
def parse_rosbag(mode, in_rosbag, out_mat):
	t = []; x = []; y = []; psi = []; v = []
	lat = []; lon = []; a = []; df = []

	b = rosbag.Bag(in_rosbag)
	
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

	# Find t_enable.  Assume first two commands are not solved to optimality, ignore those.
	count = 0
	t_accel = None
	for topic, msg, m_tm in b.read_messages(topics='/control/accel'):
		count = count + 1

		if count == 3:
			t_accel = m_tm.secs + m_tm.nsecs * 1e-9
			break

	count = 0
	t_steer = None
	for topic, msg, m_tm in b.read_messages(topics='/control/steer_angle'):
		count = count + 1

		if count == 3:
			t_steer = m_tm.secs + m_tm.nsecs * 1e-9
			break

			
	if t_accel == None and t_steer == None:
		t_enable = None
	else:
		t_enable = np.max([t_accel, t_steer])
			
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

	rdict['yaw_rate_imu'] = se_yaw_rate
	rdict['vx_gps'] = se_v_x
	rdict['vy_gps'] = se_v_y
	
	rdict['a']   = a
	rdict['a_imu_lat'] = se_lat_accel
	rdict['a_imu_long'] = se_long_accel
	rdict['df']  = df
	sio.savemat(out_mat, rdict)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
	args = parser.parse_args()
	parse_rosbag(args.mode, args.infile, args.outfile)