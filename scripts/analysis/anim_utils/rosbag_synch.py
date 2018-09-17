import rosbag
import numpy as np

# A class to organize messages by timestamp.  
# Main query function gets nearest message at a specified timestamp.
class MessageByTimestamp():
	def __init__(self, bag_object, topic_name, use_header_stamp=True):
		if use_header_stamp:
			self.bag_list = [(msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs, msg) \
				for _, msg, _ in bag_object.read_messages(topics=topic_name)]
		else:
			self.bag_list = [(t.secs + 1e-9*t.nsecs, msg) \
				for _, msg, t in bag_object.read_messages(topics=topic_name)]

		self.ts = np.array([x[0] for x in self.bag_list])		

	def get_start_time(self):
		return self.ts[0]

	def get_end_time(self):
		return self.ts[-1]

	def get_msg_at_tquery(self, t_query):
		if t_query < self.ts[0]:
			raise ValueError('Time stamp is earlier than collected data!')
		elif t_query > self.ts[-1]:
			raise ValueError('Time stamp is after than collected data!')

		return self.bag_list[ np.argmin(np.fabs(self.ts - t_query)) ][1]