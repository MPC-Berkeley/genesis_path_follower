import argparse
import scipy.io as sio
import rosbag
import pdb

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

	# Temporary Script to Plot Lateral Acceleration measurements from the rosbag.
	'''
	temp_tm = []
	temp_lat_accel = []

	for topic, msg, _ in b.read_messages(topics='/vehicle/imu'):
		temp_tm.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		temp_lat_accel.append(msg.lat_accel)

	import matplotlib.pyplot as plt 
	plt.plot(temp_tm, temp_lat_accel)
	plt.xlabel('Time (s)')
	plt.ylabel('Lateral Acceleration (m/s^2)')
	plt.show()
	'''

	rdict = {}
	rdict['mode'] = mode
	rdict['t']   = t
	rdict['x']   = x
	rdict['y']   = y
	rdict['psi'] = psi
	rdict['v']   = v
	rdict['lat'] = lat
	rdict['lon'] = lon
	rdict['a']   = a
	rdict['df']  = df
	sio.savemat(out_mat, rdict)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-m', '--mode', choices=['Real', 'Sim', 'Follow'], type=str, required=True, help='Type of Rosbag: Real Data or Simulated.')
	parser.add_argument('-i', '--infile',  type=str, required=True, help='Input: Bag File.')
	parser.add_argument('-o', '--outfile', type=str, required=True, help='Output: Mat File.')
	args = parser.parse_args()
	parse_rosbag(args.mode, args.infile, args.outfile)