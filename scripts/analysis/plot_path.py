#!/usr/bin/env python
import argparse
import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np


''' Plot the state/input history of a real or simulated vehicle from a matfile '''

DF_MIN = -0.5
DF_MAX = 0.5
D_DF   = 0.5

A_MIN  = -3.0
A_MAX  = 2.0
D_A    = 1.5

''' Low Pass Filter Implementation '''
def lpf(signal, sig_coeff = 0.01):
	filt_signal = []
	filt_signal.append(signal[0])

	for i in range(1, len(signal)):
		filt_signal.append( sig_coeff* signal[i] + (1.0-sig_coeff) * filt_signal[-1])

	return filt_signal

def plot(matfile):
	data = sio.loadmat(matfile)
	if 't_en' in data.keys():
		ts = np.ravel(data['t'] - data['t_en'])
	else:
		ts = np.ravel(data['t'])

	ps   = np.ravel(data['psi'])
	vs   = np.ravel(data['v'])
	
	accs = np.ravel(data['a'])
	dfs  = np.ravel(data['df'])

	plt.figure()
	
	# Latitude/Longitude or XY Plot	
	plt.subplot(711)	
	if data['mode'] == 'Real' or data['mode']== 'Follow':
		lats = np.ravel(data['lat'])
		lons = np.ravel(data['lon'])
		plt.plot(lons, lats, 'k')
		plt.plot(lons[0], lats[0], 'bo')
		plt.plot(lons[-1], lats[-1], 'go')
		plt.xlabel('Lon (deg)')
		plt.ylabel('Lat (deg)')
		plt.axis('equal')
	else:
		xs = np.ravel(data['x'])
		ys = np.ravel(data['y'])
		plt.plot(xs, ys, 'k')
		plt.plot(xs[0], ys[0], 'bo')
		plt.plot(xs[-1], ys[-1], 'go')
		plt.xlabel('X (m)')
		plt.ylabel('Y (m)')
		plt.axis('equal')
	
	# Velocity Plot
	plt.subplot(712)
	plt.plot(ts, vs, 'k')
	plt.xlabel('Time (s)')
	plt.ylabel('Speed (m/s)')

	# Yaw Plot
	plt.subplot(713)
	plt.plot(ts, ps, 'k')
	plt.xlabel('Time (s)')
	plt.ylabel('Psi (rad)')

	# Steering Angle Plot
	plt.subplot(714)	
	plt.plot(ts, dfs, 'k')
	plt.axhline(y=DF_MAX, c='r')
	plt.axhline(y=DF_MIN, c='r')
	plt.xlabel('Time (s)')	
	plt.ylabel('D_f (rad)')

	# Acceleration Plot
	plt.subplot(715)	
	plt.plot(ts, accs, 'k')
	plt.axhline(y=A_MAX, c='r')
	plt.axhline(y=A_MIN, c='r')
	plt.xlabel('Time (s)')	
	plt.ylabel('Acc (m/s^2)')

	# Steering Angle Derivative Plot
	plt.subplot(716)	
	plt.plot(ts[:-1], lpf(np.divide(np.diff(dfs), np.diff(ts))), 'k')
	plt.axhline(y=-D_DF, c='r')
	plt.axhline(y=D_DF, c='r')
	plt.xlabel('Time (s)')	
	plt.ylabel('D_f_dot (rad/s)')

	# Jerk Plot		
	plt.subplot(717)	
	plt.plot(ts[:-1], lpf(np.divide(np.diff(accs), np.diff(ts))), 'k')
	plt.axhline(y=-D_A, c='r')
	plt.axhline(y=D_A, c='r')
	plt.xlabel('Time (s)')	
	plt.ylabel('Jerk (m/s^3)')
	if data['mode'] == 'Real':
		plt.suptitle('Trajectory Demonstration')
	else:
		plt.suptitle('Simulated Demonstration')
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot processed matfile containing state/input history from a path following experiment.')
	parser.add_argument('-f', '--file', type=str, required=True, help='Matfile location.')
	args = parser.parse_args()
	plot(args.file)