import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
from genesis_path_follower.msg import cur_state
import numpy as np

def parseCurState(data):
	print(data.distance_m)



def parseBoxes(data):
	x = []
	y = []

	dist2crosswalk  = 10.0 #harcoded for now

	for i in range(len(data.boxes)):
		x.append(data.boxes[i].pose.position.x)
		y.append(data.boxes[i].pose.position.y)


	#search for clusters in the crosswalk
	x = np.array(x); y = np.array(y);
	arrY = np.nonzero(np.logical_and(y >= dist2crosswalk - 5, y <= dist2crosswalk + 5))
	arrX = np.nonzero(np.logical_and(x >= -5, x <= 5))

	possiblePedestrian = np.intersect1d(arrX, arrY)

	print([x[possiblePedestrian], y[possiblePedestrian]])



def lidar_loop():
	rospy.init_node('lidar_box_filter', anonymous = True)
	rospy.Subscriber('/bounding_boxes', BoundingBoxArray, parseBoxes, queue_size = 2)
	#rospy.Subscriber('/vehicle/cur_state', cur_state, parseCurState, queue_size = 2)

	rospy.spin()
if __name__=='__main__':
	lidar_loop()
