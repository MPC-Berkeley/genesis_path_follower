#!/usr/bin/env python
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
from genesis_path_follower.msg import ped_state
from genesis_path_follower.msg import vehicle_state
import numpy as np

#This class takes in bounding boxes from Euclidean clustering of LIDAR data
#and returns the X position of the pedestrian in the crosswalk

class LidarBoxSort():

	def __init__(self):
		rospy.init_node('lidar_box_filter', anonymous = True)
		rospy.Subscriber('/bounding_boxes', BoundingBoxArray, self.parseBoxes, queue_size = 2)
		rospy.Subscriber('/vehicle/vehicle_state', vehicle_state, self.parseVehicleState, queue_size = 2)
		self.ped_pub = rospy.Publisher("/ped_state", ped_state, queue_size =2)
		self.dist2crosswalk = 10.0 #initial value, will be updated by parseCurState
		self.posX = -100.; #pedestrian X position
		self.posY = -100.; #pedestrian Y position
		self.prevPosX = self.posX #last pedestrian X position (for velocity calculation)
		self.prevPosY = self.posY #last pedestrian Y position (for velocity calculation)

		self.yBounds = 5.0; #defines search region for Euclidean clusters
		self.xBounds = 5.0; #defines search region for Euclidean clusters
		self.rate = rospy.get_param('rate')
		self.r = rospy.Rate(self.rate)	
		self.t0 = 0.;
		self.t1 = 0.;	

		self.pub_loop()

	def parseVehicleState(self,data):
		self.dist2crosswalk = data.distance_m

	def parseBoxes(self,data):
		x = []
		y = []

		for i in range(len(data.boxes)):
			x.append(data.boxes[i].pose.position.x)
			y.append(data.boxes[i].pose.position.y)

 		#search for clusters in the crosswalk
		x = np.array(x); y = np.array(y);
		assert len(x) == len(y);

		arrY = np.nonzero(np.logical_and(y >= self.dist2crosswalk - self.yBounds, y <= self.dist2crosswalk + self.yBounds))
		arrX = np.nonzero(np.logical_and(x >= -self.xBounds, x <= self.xBounds))

		possiblePedestrian = np.intersect1d(arrX, arrY)

		#print(possiblePedestrian)
		if len(possiblePedestrian) > 1:
			print("Warning - more than 1 pedestrian found")

		if len(possiblePedestrian) > 0:
			self.posX = x[possiblePedestrian[0]].squeeze()
			self.posY = y[possiblePedestrian[0]].squeeze()

		self.t0 = self.t1
		self.t1 = rospy.Time.now().to_sec()


		# print([x[possiblePedestrian], y[possiblePedestrian]])


	def getVelX(self):


		dt = self.t1 - self.t0

		if dt > 0:
			velX = (self.posX - self.prevPosX) / dt #simple numerical differentation for now
		else:
			velX = 0


		self.prevPosX = self.posX

		return velX


	def getVelY(self):
		dt = self.t1 - self.t0

		if dt > 0:
			velY = (self.posY - self.prevPosY) / dt #simple numerical differentation for now
		else:
			velY = 0

		self.prevPosY = self.posY 

		return velY



	def pub_loop(self):
		while not rospy.is_shutdown():
			pedestrian_state = ped_state()
			pedestrian_state.header.stamp = rospy.Time.now()

			pedestrian_state.posX = self.posX
			pedestrian_state.posY = self.posY
			pedestrian_state.velX = self.getVelX()
			pedestrian_state.velY = self.getVelY()
			self.ped_pub.publish(pedestrian_state)
			self.r.sleep()
	





if __name__=="__main__":
	try:
		lbs = LidarBoxSort()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
