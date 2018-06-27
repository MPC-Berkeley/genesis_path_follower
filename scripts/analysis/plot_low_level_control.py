import matplotlib.pyplot as plt
import rosbag
import argparse

def make_llc_plot(bagfile):
	b = rosbag.Bag(bagfile)
	t_act = []; a_act = []; df_act = []
	t_a_cmd = []; a_cmd = [];
	t_df_cmd = []; df_cmd = [];

	for topic, msg, t in b.read_messages(topics='/vehicle/state_est'):
		t_act.append(msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs)
		a_act.append(msg.a)
		df_act.append(msg.df)
	for topic, msg, t in b.read_messages(topics='/control/accel'):
		t_a_cmd.append(t.secs + 1e-9 * t.nsecs)
		a_cmd.append(msg.data)
	for topic, msg, t in b.read_messages(topics='/control/steer_angle'):
		t_df_cmd.append(t.secs + 1e-9 * t.nsecs)
		df_cmd.append(msg.data)

	plt.figure()
	plt.subplot(211)
	plt.plot(t_a_cmd, a_cmd, 'r')
	plt.plot(t_act, a_act, 'k')
	plt.xlabel('t (s)')
	plt.ylabel('Acceleration (m/s^2)')

	plt.subplot(212)
	plt.plot(t_df_cmd, df_cmd, 'r')
	plt.plot(t_act, df_act, 'k')
	plt.xlabel('t (s)')
	plt.ylabel('Steer Angle (rad)')

	plt.suptitle('Low Level Tracking Response')
	plt.show()

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plots MPC command vs. actual low level acceleration + steering controller behavior')
	parser.add_argument('--bf',  type=str, required=True, help='Bag file for path followed.')
	args = parser.parse_args()
	make_llc_plot(args.bf)
