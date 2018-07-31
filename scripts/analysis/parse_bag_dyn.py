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
	se_v_x = []; se_v_y = []; se_yaw_rate = [];
	se_long_accel = []; se_lat_accel = [];

	b = rosbag.Bag(in_rosbag)
	
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

	rdict['wz'] = se_yaw_rate
	rdict['vx'] = se_v_x
	rdict['vy'] = se_v_y
	
	rdict['a']   = a
	rdict['a_lat'] = se_lat_accel
	rdict['a_long'] = se_long_accel
	rdict['df']  = df
	sio.savemat(out_mat, rdict)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
	args = parser.parse_args()
	parse_rosbag(args.mode, args.infile, args.outfile)
