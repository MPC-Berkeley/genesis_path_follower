import matplotlib.pyplot as plt
import rosbag
import argparse
import numpy as np

def make_llc_plot(bagfile):
	b = rosbag.Bag(bagfile)
	
	t_se = []; df_se = []; a_se = []
	t_a_long = []; a_long = []

	t_a_cmd = []; a_cmd = [];
	t_df_cmd = []; df_cmd = [];

	# Measured Acceleration and Steering
	for topic, msg, t in b.read_messages(topics='/vehicle/state_est'):
		t_se.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		a_se.append(msg.a) # as of 7/20/18, filtered with alpha = 0.01, may be too aggressive
		df_se.append(msg.df) # this one should be fine, no filtering involved

	# Measured Longitudinal Acceleration (Vehicle IMU)
	for topic, msg, _ in b.read_messages(topics='/vehicle/imu'):
		t_a_long.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)	
		a_long.append(msg.long_accel)
	
	# MPC Commands (Desired Values)	
	for topic, msg, t in b.read_messages(topics='/control/accel'):
		t_a_cmd.append(t.secs + 1e-9 * t.nsecs)
		a_cmd.append(msg.data)
	for topic, msg, t in b.read_messages(topics='/control/steer_angle'):
		t_df_cmd.append(t.secs + 1e-9 * t.nsecs)
		df_cmd.append(msg.data)

	# Messy way to estimate enable time (first optimal solution):
	# Assume first two commands are not solved to optimality, ignore those.
	t_enable = None

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
		t_enable = 0.0
	else:
		t_enable = np.max([t_accel, t_steer])

	t_se     = np.array(t_se)     - t_enable
	t_a_long = np.array(t_a_long) - t_enable
	t_a_cmd  = np.array(t_a_cmd)  - t_enable
	t_df_cmd = np.array(t_df_cmd) - t_enable

	plt.figure()
	plt.subplot(211)
	plt.plot(t_a_cmd, a_cmd, 'r', label='MPC')
	plt.plot(t_a_long, a_long, 'k', label='ACT')
	plt.xlabel('t (s)')
	plt.ylabel('Acceleration (m/s^2)')
	plt.legend()

	plt.subplot(212)
	plt.plot(t_df_cmd, df_cmd, 'r', label='MPC')
	plt.plot(t_se, df_se, 'k', label='ACT')
	plt.xlabel('t (s)')
	plt.ylabel('Steer Angle (rad)')

	plt.suptitle('Low Level Tracking Response')
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plots MPC command vs. actual low level acceleration + steering controller behavior')
	parser.add_argument('--bf',  type=str, required=True, help='Bag file for path followed.')
	args = parser.parse_args()
	make_llc_plot(args.bf)
