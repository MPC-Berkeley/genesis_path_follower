import matplotlib.pyplot as plt
import rosbag
import argparse
import numpy as np

def make_llc_plot(bagfile):
	b = rosbag.Bag(bagfile)
	state_est_topic_name = '/vehicle/state_est'
	mpc_path_topic_name  = '/vehicle/mpc_path'
	if '/vehicle/state_est_dyn' in  b.get_type_and_topic_info()[1].keys():
		state_est_topic_name = '/vehicle/state_est_dyn'
		mpc_path_topic_name  = '/vehicle/mpc_path_dyn'		
	
	t_se = []; df_se = []; a_se = []
	t_a_long = []; a_long = []
	t_a_cmd = []; a_cmd = [];
	t_df_cmd = []; df_cmd = [];

	# Measured Acceleration and Steering
	for topic, msg, _ in b.read_messages(topics=state_est_topic_name):
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

	# Estimate controller enable time by using the first optimal solution 
	# from the MPC module, based on the MPC command/path message.
	# TODO: alternatively use ada_stat/acc_mode status information to see 
	# if the controller is enabled or not.
	t_enable = None
	# for topic, msg, _ in b.read_messages(topics=mpc_path_topic_name):
	# 	if msg.solv_status_long == 'Optimal':
	# 		t_enable = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
			# break
			
	if t_enable == None:
		t_enable = max(t_a_cmd[0], t_df_cmd[0])
	
	t_se     = np.array(t_se)     - t_enable
	t_a_long = np.array(t_a_long) - t_enable
	t_a_cmd  = np.array(t_a_cmd)  - t_enable
	t_df_cmd = np.array(t_df_cmd) - t_enable

	# TODO: Do we want to allow simulated, path following data too?  If so, need to switch a_long with a_se.
	t_lims = [0.0, max(t_se[-1], t_a_long[-1], t_a_cmd[-1], t_df_cmd[-1])]

	plt.figure()
	plt.subplot(211)
	plt.plot(t_a_cmd, a_cmd, 'r', label='MPC')
	
	if len(a_long) == 0:
		plt.plot(t_se, a_se, 'k', label='ACT')	     # Use filtered acceleration for simulated data.
		print("Note: Using filtered acceleration for simulated bag file.")
	else:
		plt.plot(t_a_long, a_long, 'k', label='ACT') # Use longitudinal acceleration measured by vehicle for real data.
		
	plt.xlim(t_lims)
	plt.xlabel('t (s)')
	plt.ylabel('Acceleration (m/s^2)')
	plt.legend()

	plt.subplot(212)
	plt.plot(t_df_cmd, df_cmd, 'r', label='MPC')
	plt.plot(t_se, df_se, 'k', label='ACT')
	plt.xlim(t_lims)
	plt.xlabel('t (s)')
	plt.ylabel('Steer Angle (rad)')

	plt.suptitle('Low Level Tracking Response')
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plots MPC command vs. actual low level acceleration + steering controller behavior.  Meant for real, path following data.')
	parser.add_argument('--bf',  type=str, required=True, help='Bag file for path followed.')
	args = parser.parse_args()
	make_llc_plot(args.bf)
