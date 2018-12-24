#!/usr/bin/env python

# MPC Command Publisher/Controller Module Interface to the Genesis.
# This version using the Nonlinear Kinematic Bicycle Model.

###########################################
#### ROBOTOS
###########################################
import rospy
from genesis_path_follower.msg import state_est
from genesis_path_follower.msg import mpc_path
from std_msgs.msg import UInt8 as UInt8Msg
from std_msgs.msg import Float32 as Float32Msg

###########################################
#### LOAD ROSPARAMS
###########################################
if rospy.has_param("mat_waypoints"):
	mat_fname = rospy.get_param("mat_waypoints")
else:
	raise ValueError("No Matfile of waypoints provided!")

if rospy.has_param("track_using_time") and rospy.has_param("target_vel"):
	track_with_time = rospy.get_param("track_using_time")
	target_vel = rospy.get_param("target_vel")	
else:
	raise ValueError("Invalid rosparam trajectory definition: track_using_time and target_vel")

if rospy.has_param("scripts_dir"):
	scripts_dir = rospy.get_param("scripts_dir")
else:
	raise ValueError("Did not provide the scripts directory!")

if rospy.has_param("lat0") and rospy.has_param("lat0") and rospy.has_param("lat0"):
	lat0 = rospy.get_param("lat0")
	lon0 = rospy.get_param("lon0")
	yaw0 = rospy.get_param("yaw0")
else:
	raise ValueError("Invalid rosparam global origin provided!")
###########################################
#### Reference GPS Trajectory Module
###########################################
# Access Python modules for path processing.  Ugly way of doing it, can seek to clean this up in the future.
import sys
gps_utils_loc = scripts_dir + "gps_utils/"
sys.path.append(gps_utils_loc)
import ref_gps_traj as rgt
grt = rgt.GPSRefTrajectory(mat_filename=mat_fname, LAT0=lat0, LON0=lon0, YAW0=yaw0)

###########################################
#### MPC Controller Module with Cost Function Weights.
#### Global Variables for Callbacks/Control Loop.
###########################################
import julia
jl = julia.Julia()
add_path_script = 'push!(LOAD_PATH, "%s/mpc_utils/")' % scripts_dir
jl.eval(add_path_script)
jl.eval('import GPSKinMPCPathFollower')
from julia import GPSKinMPCPathFollower as kmpc
kmpc.update_cost(9.0, 9.0, 10.0, 0.0, 100.0, 1000.0, 0.0, 0.0) # x,y,psi,v,da,ddf,a,df

# Reference for MPC
if target_vel > 0.0:
	des_speed = target_vel
else:
	des_speed = 0.00

ref_lock = False				
received_reference = False
x_curr  = 0.0
y_curr  = 0.0
psi_curr  = 0.0
v_curr  = 0.0
command_stop = False

###########################################
#### State Estimation Callback.
###########################################
def state_est_callback(msg):
	global x_curr, y_curr, psi_curr, v_curr
	global received_reference

	if ref_lock == False:
		x_curr = msg.x
		y_curr = msg.y
		psi_curr = msg.psi
		v_curr = msg.v
		received_reference = True

def pub_loop(acc_pub_obj, steer_pub_obj, mpc_path_pub_obj):
	loop_rate = rospy.Rate(20.0)
	while not rospy.is_shutdown():
		if not received_reference:
			# Reference not received so don't use MPC yet.
			loop_rate.sleep()
			continue

		# Ref lock used to ensure that get/set of state doesn't happen simultaneously.
		global ref_lock				
		ref_lock = True

		global x_curr, y_curr, psi_curr, v_curr, des_speed, command_stop

		if not track_with_time:
			# fixed velocity-based path tracking
			x_ref, y_ref, psi_ref, stop_cmd = grt.get_waypoints(x_curr, y_curr, psi_curr, des_speed)
			if stop_cmd == True:
				command_stop = True			
		else:
			# trajectory tracking
			x_ref, y_ref, psi_ref, stop_cmd = grt.get_waypoints(x_curr, y_curr, psi_curr)

			if stop_cmd == True:
				command_stop = True
		
		# Update Model
		kmpc.update_init_cond(x_curr, y_curr, psi_curr, v_curr)
		kmpc.update_reference(x_ref, y_ref, psi_ref, des_speed)

		ref_lock = False
		
		if command_stop == False:
			a_opt, df_opt, is_opt, solv_time = kmpc.solve_model()

			rostm = rospy.get_rostime()
			tm_secs = rostm.secs + 1e-9 * rostm.nsecs

			log_str = "Solve Status: %s, Acc: %.3f, SA: %.3f, ST: %.3f" % (is_opt, a_opt, df_opt, solv_time)
			rospy.loginfo(log_str)

			if is_opt == 'Optimal':
				acc_pub_obj.publish(Float32Msg(a_opt))
				steer_pub_obj.publish(Float32Msg(df_opt))

			kmpc.update_current_input(df_opt, a_opt)
			res = kmpc.get_solver_results()

			mpc_path_msg = mpc_path()
			mpc_path_msg.header.stamp = rostm
			mpc_path_msg.solv_status  = is_opt
			mpc_path_msg.solv_time = solv_time
			mpc_path_msg.xs   = res[0] 	# x_mpc
			mpc_path_msg.ys   = res[1] 	# y_mpc
			mpc_path_msg.vs   = res[2]	# v_mpc
			mpc_path_msg.psis = res[3] 	# psi_mpc	
			mpc_path_msg.xr   = res[4] 	# x_ref
			mpc_path_msg.yr   = res[5] 	# y_ref
			mpc_path_msg.vr   = [res[6]]# v_ref
			mpc_path_msg.psir = res[7] 	# psi_ref
			mpc_path_msg.df   = res[8]	# d_f
			mpc_path_msg.acc  = res[9]	# acc
			mpc_path_pub_obj.publish(mpc_path_msg)
		else:
			acc_pub_obj.publish(Float32Msg(-1.0))
			steer_pub_obj.publish(Float32Msg(0.0))						

		loop_rate.sleep()

def start_mpc_node():
	rospy.init_node("dbw_mpc_pf")
	acc_pub   = rospy.Publisher("/control/accel", Float32Msg, queue_size=2)
	steer_pub = rospy.Publisher("/control/steer_angle", Float32Msg, queue_size=2)

	acc_enable_pub   = rospy.Publisher("/control/enable_accel", UInt8Msg, queue_size=2, latch=True)
	steer_enable_pub = rospy.Publisher("/control/enable_spas",  UInt8Msg, queue_size=2, latch=True)

	mpc_path_pub = rospy.Publisher("mpc_path", mpc_path, queue_size=2)
	sub_state  = rospy.Subscriber("state_est", state_est, state_est_callback, queue_size=2)

	# Start up Ipopt/Solver.
	for i in range(3):
		kmpc.solve_model()

	acc_enable_pub.publish(UInt8Msg(2))
	steer_enable_pub.publish(UInt8Msg(1))
	pub_loop(acc_pub, steer_pub, mpc_path_pub)

if __name__=='__main__':	
	start_mpc_node()