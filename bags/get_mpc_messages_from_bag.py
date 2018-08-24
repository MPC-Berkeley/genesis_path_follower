import rosbag
import pdb

bag_name = 'path_follow_medLap_expm_noDynPreSolve.bag'

b = rosbag.Bag(bag_name)
msg_prev = None

for topic, msg, t in b.read_messages(topics='/vehicle/mpc_path_dyn'):
	if msg.model == 'dyn':
		pdb.set_trace()
		print(msg_prev)
		print(msg)
		break;
	else:
		msg_prev = msg