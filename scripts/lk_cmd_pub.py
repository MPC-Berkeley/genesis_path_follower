#!/usr/bin/env python
import rospy  ##TODO: Cannot find rospy
from genesis_path_follower.msg import state_est
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
import math
import argparse
import controllers
import path_lib
import vehicle_lib
import velocityprofile_lib
import sim_lib


#################################################################################################
### Path following code using lanekeeping controller from VSD 2015 paper ########################
#################################################################################################

def parseStateEstMessage(msg):
	X = msg.X
	Y = msg.Y
	psi = msg.psi
	Ux = msg.vel
	Ax = msg.acc_filt
	delta =  msg.df

	return X, Y, psi, Ux, Ax, delta


def pub_loop():

	##ToDo: check if these are the right messages - how does it know these are the messages?
	#Define steering and acc enable publishers
	enable_steer_pub = rospy.Publisher("control/enable_spas", UInt8, queue_size = 1, latch=True)
	enable_acc_pub   = rospy.Publisher("control/enable_accel", UInt8, queue_size =10, latch=True)  ##Why queue_size = 10?

	#Define steering and accel value publishers
	steer_pub = rospy.Publisher("control/steer_angle", Float32, queue_size = 1) 	
	accel_pub = rospy.Publisher("control/accel", 	   Float32, queue_size =10)


	#define subscribers
	rospy.Subscriber('state_est', state_est, parseStateEstMessage, queue_size=1)



	#Is this ok?
	rospy.init_node('lk_cmd_pub', anonymous=True)
	r = rospy.Rate(50.0)  ##TODO: Can we run this fast?

	while(enable_steer_pub.get_num_connections == 0):
		print('Waiting to enable steering!')
		r.sleep()

	while(enable_acc_pub.get_num_connections == 0):
		print('Waiting to enable acceleration!')
		r.sleep()


	enable_steer_pub.publish(1) # enable steering control.
	enable_acc_pub.publish(2) # enable acceleration control.


	t_start = rospy.Time.now()
	print('Path Tracking Test: %s, Started at %f' % (mode, t_start.secs + t_start.nsecs*1e-9))


	#Initialize vehicle
	genesis = vehicle_lib.Vehicle('genesis')

	#Initialize Path object
	path = path_lib.Path()
	path.loadFromMAT("paths/rightTurnRFSdownshifted.mat")
	path.setFriction(0.5)

	#Create speed profile
	speedProfile = velocityprofile_lib.VelocityProfile("racing")
	speedProfile.generate(genesis, path)

	#Create controller object - use lanekeeping
	controller = controllers.LaneKeepingController(path, genesis, speedProfile)

	#Create local and global states and control input object
	localState = sim_lib.LocalState()
	globalState = sim_lib.GlobalState()
	controlInput = controllers.ControlInput()

	#Initialize map matching object - use closest style
	mapMatch = MapMatch(path, "closest")




##########Main loop ############################################################################
	while not rospy.is_shutdown():
		t_now = rospy.Time.now()

		dt = t_now - t_start
		dt_secs = dt.secs + 1e-9 * dt.nsecs


		#Update local and global state - WE MAY NEED GLOBAL VARIABLES HERE
		#Yaw rate and Uy currently not returned by state publisher!
		localState.Update(Ux = Ux)
		globalState.Update(posE = X, posN = Y, psi = psi)

		#Localize Vehicle
		mapMatch.localize(localState, globalState)

		#Calculate control inputs
		controller.updateInput(localState, controlInput)
		delta = controlInput.delta
		Fx = controlInput.Fx

		# use F = m*a to get desired acceleration
		accel = Fx / genesis.m

		print('Steer Command: %f deg \t Accel Command: %f ms2 \t Time: %f seconds' % (controlInput.delta, accel, dt_secs))

		#Publish control inputs

		steer_pub.publish(delta)
		accel_pub.publish(accel)

		r.sleep()

	#Disable inputs after test is ended
	enable_steer_pub.publish(0) # disable steering control.
	enabl_acc_pub.publish(0) # disable acceleration control.


##TODO: Ask Vijay if we need this
if __name__=="__main__":
	try:
		pub_loop()
	except rospy.ROSInterruptException:
		pass # to handle node shutdown cleanly
