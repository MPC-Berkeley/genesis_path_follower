import sys
from lk_utils.Classes import LMPCprediction, ClosedLoopData
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
from lk_utils.LMPC import ControllerLMPC
from lk_utils.controllers import *
from lk_utils.path_lib import *
from lk_utils.vehicle_lib import *
from lk_utils.velocityprofiles import *
from lk_utils.sim_lib import *
import gps_utils.ref_gps_traj as r
import pickle
import pdb
import glob, os
import datetime
import scipy.io as sio

from numpy import linalg as la


def main():
	homedir = os.path.expanduser("~")

	# getSecondOrderDelayDynamics([1,2,7,5])


		


	
	# file_data = open(homedir+'/genesis_data/ClosedLoopDataLMPC_n14a.obj', 'rb')

	file_data = open(homedir+'/genesis_data/ClosedLoopDataLMPC.obj', 'rb')
	# file_data2 = open(homedir+'/genesis_data/ClosedLoopDataLMPC_wo.obj', 'rb')
	# file_data3 = open(homedir+'/genesis_data/ClosedLoopDataLMPC_sinmeas.obj', 'rb')
	# file_data4 = open(homedir+'/genesis_data/ClosedLoopDataLMPC_n14a.obj', 'rb')

	# file_data2 = open(homedir+'/genesis_data/ClosedLoopDataLMPC_n14.obj', 'rb')
	# file_data3 = open(homedir+'/genesis_data/ClosedLoopDataLMPC_n14a.obj', 'rb')

	ClosedLoopData = pickle.load(file_data)
	LMPController = pickle.load(file_data)
	LMPCOpenLoopData = pickle.load(file_data) 

	# ClosedLoopData2 = pickle.load(file_data2)
	# LMPController2 = pickle.load(file_data2)
	# LMPCOpenLoopData2 = pickle.load(file_data2)

	# ClosedLoopData3 = pickle.load(file_data3)
	# LMPController3 = pickle.load(file_data3)
	# LMPCOpenLoopData3 = pickle.load(file_data3)

	# ClosedLoopData4 = pickle.load(file_data4)
	# LMPController4 = pickle.load(file_data4)
	# LMPCOpenLoopData4 = pickle.load(file_data4)
  	

	
	# file_data2.close()
	# file_data3.close()
	file_data.close()

	LapToPlot = range(4,8)
	# plotComputationalTime(LMPController, LapToPlot)

	# print "Track length is: ", LMPController.trackLength

	currentDirectory = os.getcwd()
	mat_name = currentDirectory+'/../paths/lmpcMap.mat'
	lat0 = 35.04884687
	lon0 = -118.040313
	yaw0 = 0.0


	grt = r.GPSRefTrajectory(mat_filename=mat_name, LAT0=lat0, LON0=lon0, YAW0=yaw0) # only 1 should be valid.

	# Plot Lap Time
	# plt.figure()
	# plt.plot([i*LMPController.dt for i in LMPController.LapCounter[1:LMPController.it]], '-o', label="Lap Time")
	# plt.legend()
	# plt.show()
	# pdb.set_trace()
	# ## Plot First initial learning
	LapToPlotLearningProcess = [0,1,2,4, 6, 8, 11, 13]
	# LapToPlotLearningProcess = [15,16]#[0, 2, 3, 4, 5, 7]
	LapCompare=[3]	

	plotClosedLoopLMPC(LMPController, grt, LapToPlotLearningProcess)
	# plotClosedLoopLMPC(LMPController2, grt, LapToPlotLearningProcess)
	# plotClosedLoopLMPC(LMPController3, grt, LapToPlotLearningProcess)
	# plotMeasuredAndAppliedSteering(LMPController, LapToPlotLearningProcess)
	# plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapToPlotLearningProcess)

	# plotClosedLoopColorLMPC(LMPController, grt, LapToPlotLearningProcess)
	plt.show()
	# plotCompareSteering(LMPController, LMPController3, LapCompare)#LMPController3, LapCompare)
	# CompareStates(LMPController,LMPController2,LMPController3, LMPController4, LapCompare)
	# CompareStates(LMPController,LMPController2,LMPController3, LapCompare)
	# plotA(LMPController, LapCompare)

	# plotA(LMPController, [4])
	
	# Now convergence
	LapToPlot = [16, 18, 20, 25, 30, 35]
	plotClosedLoopLMPC(LMPController, grt, LapToPlot)
	# plotMeasuredAndAppliedSteering(LMPController, LapToPlot)
	# plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapToPlot)
	plotClosedLoopColorLMPC(LMPController, grt, LapToPlot)
	plt.show()



	# # Plot Best Laps
	# # LapToPlot      = range(0, LMPController.it)
	# # BestNunberLaps = 2
	# # SortedTimes    = np.sort(LMPController.LapCounter[1:LMPController.it])
	# # LapToPlot      = np.argsort(LMPController.LapCounter[1:LMPController.it])[0:BestNunberLaps]
	# # # LapToPlot = range(15,19)
	
	# LapToPlot = [10, 11,15]
	# plotClosedLoopColorLMPC(LMPController, grt, LapToPlot)
	
	# pdb.set_trace()

	# plotClosedLoopLMPC(LMPController, grt, LapToPlot)
	# # Plot Acceleration
	# # plotAccelerations(LMPController, LapToPlot, map)
	
	# plt.show()

	# # Plot One Step Prediction Error
	# LapsToPlot = [3, 4, 10, 18]
	# plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapsToPlot)
	# plt.show()

	# # Computational Time    
	# plotComputationalTime(LMPController, LapToPlot)
	# plt.show()

	# print "Do you wanna create xy gif? [Lap #/n]"
	# inputKeyBoard = raw_input()
	# if inputKeyBoard != "n":
	# 	# animation_xy(grt, LMPCOpenLoopData, LMPController, int(inputKeyBoard))
	# 	saveGif_xyResults(grt, LMPCOpenLoopData, LMPController, int(inputKeyBoard))

	# print "Do you wanna create state gif? [Lap #/n]"
	# inputKeyBoard = raw_input()
	# if inputKeyBoard != "n":
	# 	Save_statesAnimation(grt, LMPCOpenLoopData, LMPController, int(inputKeyBoard))
	# pdb.set_trace()
	# animation_states(map, LMPCOpenLoopData, LMPController, 10)

	# print "Do you wanna create xy gif for sys ID? [Lap #/n]"
	# inputKeyBoard = raw_input()
	# if inputKeyBoard != "n":
	# 	saveGif_xyResultsSysID(map, LMPCOpenLoopData, LMPController, int(inputKeyBoard))

	# plt.show()
def getSecondOrderDelayDynamics(Laps):
	homedir = os.path.expanduser("~")

	Output_Vector=np.array([])
	Data_mat=np.empty((0,4))
	for root, dirs, files in os.walk(homedir+'/genesis_data/id_files/'):
		for file in files:
			if file.endswith(".obj"):
				file_data = open(os.path.join(root, file), 'rb')
				ClosedLoopData = pickle.load(file_data)
				LMPController = pickle.load(file_data)
				LMPCOpenLoopData = pickle.load(file_data)
				LapCounter  = LMPController.LapCounter
				SS      = LMPController.SS
				uSS     = LMPController.uSS
				O=np.array([])
				M=np.array([])
				C=np.array([])
				for i in Laps:
					O=np.concatenate((O,LMPController.measSteering[2:LapCounter[i]-1, 0, i]))
					M=np.concatenate((M,np.repeat(LMPController.measSteering[0:LapCounter[i] - 2, 0, i],2)[1:-1]))
					C=np.concatenate((C,np.repeat(uSS[0:LapCounter[i] - 2, 0, i],2)[1:-1]))
				Output_Vector=np.concatenate((Output_Vector, O))#.reshape((LapCounter[1]+LapCounter[2])/2,2)
				Meas_Data=M.reshape((M.shape[0]/2),2)#.reshape((LapCounter[1]+LapCounter[2])/2,2)			
				Cmd_Data=C.reshape((C.shape[0]/2),2)
				Data_mat=np.append(Data_mat,np.hstack((Meas_Data,Cmd_Data)), axis=0)							
				file_data.close()			
	PInv=np.linalg.pinv(Data_mat)
	Delay_Dyn=np.dot(PInv, Output_Vector)
	print(Delay_Dyn)

def plotAccelerations(LMPController, LapToPlot, map):
	n = LMPController.n
	x = np.zeros((10000, 1, LMPController.it+2))
	s = np.zeros((10000, 1, LMPController.it+2))
	y = np.zeros((10000, 1, LMPController.it+2))
	ax = np.zeros((10000, 1, LMPController.it+2))
	ay = np.zeros((10000, 1, LMPController.it+2))
	psiDot = np.zeros((10000, 1, LMPController.it+2))
	roll = np.zeros((10000, 1, LMPController.it+2))
	pitch = np.zeros((10000, 1, LMPController.it+2))
	LapCounter = np.zeros(LMPController.it+2).astype(int)

	homedir = os.path.expanduser("~")
	pathSave = os.path.join(homedir,"barc_data/estimator_output.npz")
	npz_output = np.load(pathSave)
	x_est_his           = npz_output["x_est_his"]
	y_est_his           = npz_output["y_est_his"]
	vx_est_his          = npz_output["vx_est_his"] 
	vy_est_his          = npz_output["vy_est_his"] 
	ax_est_his          = npz_output["ax_est_his"] 
	ay_est_his          = npz_output["ay_est_his"] 
	psiDot_est_his      = npz_output["psiDot_est_his"]  
	yaw_est_his         = npz_output["yaw_est_his"]  
	gps_time            = npz_output["gps_time"]
	imu_time            = npz_output["imu_time"]
	enc_time            = npz_output["enc_time"]
	inp_x_his           = npz_output["inp_x_his"]
	inp_y_his           = npz_output["inp_y_his"]
	inp_v_meas_his      = npz_output["inp_v_meas_his"]
	inp_ax_his          = npz_output["inp_ax_his"]
	inp_ay_his          = npz_output["inp_ay_his"]
	inp_psiDot_his      = npz_output["inp_psiDot_his"]
	inp_a_his           = npz_output["inp_a_his"]
	inp_df_his          = npz_output["inp_df_his"]
	roll_his            = npz_output["roll_his"]
	pitch_his           = npz_output["pitch_his"]
	wx_his              = npz_output["wx_his"]
	wy_his              = npz_output["wy_his"]
	wz_his              = npz_output["wz_his"]
	v_rl_his            = npz_output["v_rl_his"]
	v_rr_his            = npz_output["v_rr_his"]
	v_fl_his            = npz_output["v_fl_his"]
	v_fr_his            = npz_output["v_fr_his"]
	yaw_his             = npz_output["psi_raw_his"]

	halfTrack = 0
	iteration = 0
	TimeCounter = 0
	for i in range(0, len(x_est_his)):
		s_i, ey_i, epsi_i, _ = map.getLocalPosition(x_est_his[i], y_est_his[i], yaw_est_his[i])
		
		if s_i > LMPController.trackLength/4 and s_i < LMPController.trackLength/4*3:
			halfTrack = 1
		
		if s_i < LMPController.trackLength/4 and halfTrack == 1:
			print "Finishced unpacking iteration: ", iteration
			halfTrack = 0
			iteration += 1
			LapCounter[iteration-1] = TimeCounter - 1
			LapCounter[iteration] = 0
			TimeCounter = 0

		if iteration > LMPController.it:
			break

		s[TimeCounter, 0, iteration]      = s_i
		ax[TimeCounter, 0, iteration]     = inp_ax_his[i]
		ay[TimeCounter, 0, iteration]     = inp_ay_his[i]
		psiDot[TimeCounter, 0, iteration] = inp_psiDot_his[i]
		roll[TimeCounter, 0, iteration]   = roll_his[i]
		pitch[TimeCounter, 0, iteration]  = pitch_his[i]

		TimeCounter += 1


	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	plt.figure()
	plt.subplot(511)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], ax[0:LapCounter[i], 0, i], '-o', label=i, color=plotColors[counter])
		counter += 1
	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('ax [m/s^2]')
	plt.subplot(512)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], ay[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('ay [m/s^2]')
	plt.subplot(513)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], psiDot[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('psiDot [rad/s]')
	
	plt.subplot(514)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], pitch[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('psiDot [rad/s]')

	plt.subplot(515)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], roll[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('psiDot [rad/s]')

	plt.figure()
	plt.subplot(211)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], ay[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('ay [m/s^2]')
	plt.xlim([0, LMPController.trackLength])

	plt.subplot(212)
	counter = 0
	for i in LapToPlot:
		plt.plot(s[0:LapCounter[i], 0, i], roll[0:LapCounter[i], 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.xlim([0, LMPController.trackLength])
	plt.ylabel('roll [rad]')    
	plt.xlabel('s [m]')    
	
def plotA(LMPController, Laps):
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	plotColors = ['b','g','r','c','y','k','m']
	A0=LMPController.A0
	
	plt.figure()
	plt.subplot(311)
	plt.title("A Matrix")
	counter = 0
	for i in Laps:
		plt.plot(SS[1:LapCounter[i], 4, i], A0[0, 0, 1:LapCounter[i], i-3], '-o', color=plotColors[counter], label=0)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[0, 1, 1:LapCounter[i], i-3], '-*', color=plotColors[counter+1], label=1)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[0, 2, 1:LapCounter[i], i-3], '-s', color=plotColors[counter+2], label=2)
		counter += 1
		plt.legend()
	plt.ylabel('vx [m/s]')
	plt.subplot(312)
	counter = 0
	for i in Laps:
		plt.plot(SS[1:LapCounter[i], 4, i], A0[1, 0, 1:LapCounter[i], i-3], '-o', color=plotColors[counter], label=0)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[1, 1, 1:LapCounter[i], i-3], '-*', color=plotColors[counter+1], label=1)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[1, 2, 1:LapCounter[i], i-3], '-s', color=plotColors[counter+2], label=2)
		counter += 1
	plt.ylabel('vy [m/s]')
	plt.subplot(313)
	counter = 0
	for i in Laps:
		plt.plot(SS[1:LapCounter[i], 4, i], A0[2, 0, 1:LapCounter[i], i-3], '-o', color=plotColors[counter], label=0)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[2, 1, 1:LapCounter[i], i-3], '-*', color=plotColors[counter+1], label=1)
		plt.plot(SS[1:LapCounter[i], 4, i], A0[2, 2, 1:LapCounter[i], i-3], '-s', color=plotColors[counter+2], label=2)
		counter += 1
	plt.ylabel('wz [rad/s]')

	


def plotComputationalTime(LMPController, LapToPlot):
	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS
	qpTime  = LMPController.qpTime
	sysIDTime  = LMPController.sysIDTime
	contrTime  = LMPController.contrTime


	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	plt.figure()
	plt.subplot(311)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], qpTime[0:LapCounter[i], i], '-o', label=i, color=plotColors[counter])
		counter += 1
	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('QP solver time [s]')
	plt.subplot(312)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], sysIDTime[0:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('Sys ID time [s]')
	plt.subplot(313)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], qpTime[0:LapCounter[i], i] + sysIDTime[0:LapCounter[i], i], '-o', color=plotColors[counter])
		plt.plot(SS[0:LapCounter[i], 4, i], contrTime[0:LapCounter[i], i], '-*', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('Total [s]')


def plotOneStepPreditionError(LMPController, LMPCOpenLoopData, LapToPlot):
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS
	TotNumberIt = LMPController.it
	oneStepPredictionError = LMPCOpenLoopData.oneStepPredictionError

	plotColors = ['b','g','r','c','y','k','m']
	
	plt.figure(10)
	plt.subplot(611)
	plt.title("One Step Prediction Error")
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[0, 1:LapCounter[i], i], '-o', color=plotColors[counter], label=i)
		counter += 1
		plt.legend()
	plt.ylabel('vx [m/s]')
	plt.subplot(612)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[1, 1:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('vy [m/s]')
	plt.subplot(613)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[2, 1:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('wz [rad/s]')
	plt.subplot(614)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[3, 1:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('epsi [rad]')
	plt.subplot(615)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[4, 1:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('s [m]')
	plt.subplot(616)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[1:LapCounter[i], 4, i], oneStepPredictionError[5, 1:LapCounter[i], i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('ey [m]')
	plt.xlabel('s [m]')



def plotTrajectory(map, ClosedLoop):
	x = ClosedLoop.x
	x_glob = ClosedLoop.x_glob
	u = ClosedLoop.u
	
	Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
	Points1 = np.zeros((int(Points), 2))
	Points2 = np.zeros((int(Points), 2))
	Points0 = np.zeros((int(Points), 2))
	for i in range(0, int(Points)):
		Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
		Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
		Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

	plt.figure()
	plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
	plt.plot(Points0[:, 0], Points0[:, 1], '--')
	plt.plot(Points1[:, 0], Points1[:, 1], '-b')
	plt.plot(Points2[:, 0], Points2[:, 1], '-b')
	plt.plot(x_glob[:, 4], x_glob[:, 5], '-r')

	plt.figure()
	plt.subplot(711)
	plt.plot(x[:, 4], x[:, 0], '-o')
	plt.ylabel('vx')
	plt.subplot(712)
	plt.plot(x[:, 4], x[:, 1], '-o')
	plt.ylabel('vy')
	plt.subplot(713)
	plt.plot(x[:, 4], x[:, 2], '-o')
	plt.ylabel('wz')
	plt.subplot(714)
	plt.plot(x[:, 4], x[:, 3], '-o')
	plt.ylabel('epsi')
	plt.subplot(715)
	plt.plot(x[:, 4], x[:, 5], '-o')
	plt.ylabel('ey')
	plt.subplot(716)
	plt.plot(x[0:-1, 4], u[:, 0], '-o')
	plt.ylabel('steering')
	plt.subplot(717)
	plt.plot(x[0:-1, 4], u[:, 1], '-o')
	plt.ylabel('acc')
	plt.show()


def plotClosedLoopColorLMPC(LMPController, grt, LapToPlot):
	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS

	TotNumberIt = LMPController.it

	print "Number iterations: ", TotNumberIt
	plt.figure()
	x_global_traj = grt.get_Xs()
	y_global_traj = grt.get_Ys()
	plt.plot(x_global_traj, y_global_traj, 'k') 

	yaws = grt.get_yaws()
	x_track_ib=np.zeros_like(x_global_traj)
	y_track_ib=np.zeros_like(y_global_traj)
	x_track_ob=np.zeros_like(x_global_traj)
	y_track_ob=np.zeros_like(y_global_traj)
	for i in range(len(x_global_traj)):
		x_track_ib[i]=x_global_traj[i]-LMPController.halfWidth*np.cos(yaws[i])
		y_track_ib[i]=y_global_traj[i]-LMPController.halfWidth*np.sin(yaws[i])
		x_track_ob[i]=x_global_traj[i]+LMPController.halfWidth*np.cos(yaws[i])
		y_track_ob[i]=y_global_traj[i]+LMPController.halfWidth*np.sin(yaws[i])

	plt.plot(x_track_ib, y_track_ib, 'k')
	plt.plot(x_track_ob, y_track_ob, 'k')

	xPlot = []
	yPlot = []
	Color = []
	for i in LapToPlot:
		for j in range(0, len(SS_glob[0:LapCounter[i], 4, i].tolist())):
			xPlot.append(SS_glob[0:LapCounter[i], 4, i].tolist()[j])
			yPlot.append(SS_glob[0:LapCounter[i], 5, i].tolist()[j])
			Color.append(np.sqrt( (SS_glob[0:LapCounter[i], 0, i].tolist()[j])**2 +  (SS_glob[0:LapCounter[i], 0, i].tolist()[j]) ) )

	pdb.set_trace()
	plt.scatter(xPlot, yPlot, alpha=1.0, c = Color, s = 100)
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")

	# plt.scatter(SS_glob[0:LapCounter[i], 4, i], SS_glob[0:LapCounter[i], 5, i], alpha=0.8, c = SS_glob[0:LapCounter[i], 0, i])
	plt.colorbar()
	plt.show()

def CompareStates(LMPController,LMPController2, LMPController3, LMPController4, LapToPlot):
# def CompareStates(LMPController,LMPController2, LMPController3, LapToPlot):
	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS

	SS_glob2 = LMPController2.SS_glob
	LapCounter2  = LMPController2.LapCounter
	SS2      = LMPController2.SS
	uSS2     = LMPController2.uSS

	SS_glob3 = LMPController3.SS_glob
	LapCounter3  = LMPController3.LapCounter
	SS3      = LMPController3.SS
	uSS3     = LMPController3.uSS

	SS_glob4 = LMPController4.SS_glob
	LapCounter4  = LMPController4.LapCounter
	SS4      = LMPController4.SS
	uSS4     = LMPController4.uSS

	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	plt.figure()
	plt.subplot(711)
	counter=0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 0, i], '-o', label="exp", color=plotColors[counter])#"exp", color=plotColors[counter])
		plt.plot(SS2[0:LapCounter[i], 4, i], SS2[0:LapCounter[i], 0, i], '-o', label="w/o", color=plotColors[counter+1])
		plt.plot(SS3[0:LapCounter[i], 4, i], SS3[0:LapCounter[i], 0, i], '-o', label="sin w meas", color=plotColors[counter+2])
		plt.plot(SS4[0:LapCounter[i], 4, i], SS4[0:LapCounter[i], 0, i], '-o', label="sin w com", color=plotColors[counter+3])
		counter += 1
	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('vx [m/s]')
	plt.subplot(712)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 1, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i], 4, i], SS2[0:LapCounter[i], 1, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i], 4, i], SS3[0:LapCounter[i], 1, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i], 4, i], SS4[0:LapCounter[i], 1, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('vy [m/s]')
	plt.subplot(713)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 2, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i], 4, i], SS2[0:LapCounter[i], 2, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i], 4, i], SS3[0:LapCounter[i], 2, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i], 4, i], SS4[0:LapCounter[i], 2, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('wz [rad/s]')
	plt.subplot(714)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 3, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i], 4, i], SS2[0:LapCounter[i], 3, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i], 4, i], SS3[0:LapCounter[i], 3, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i], 4, i], SS4[0:LapCounter[i], 3, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('epsi [rad]')
	plt.subplot(715)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 5, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i], 4, i], SS2[0:LapCounter[i], 5, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i], 4, i], SS3[0:LapCounter[i], 5, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i], 4, i], SS4[0:LapCounter[i], 5, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('ey [m]')
	plt.subplot(716)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 0, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i]-1, 4, i], uSS2[0:LapCounter[i]-1, 0, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i]-1, 4, i], uSS3[0:LapCounter[i]-1, 0, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i]-1, 4, i], uSS4[0:LapCounter[i]-1, 0, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.ylabel('Steering [rad]')
	plt.subplot(717)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter], label="exp")
		plt.plot(SS2[0:LapCounter[i]-1, 4, i], uSS2[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter+1], label="w/o")
		plt.plot(SS3[0:LapCounter[i]-1, 4, i], uSS3[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter+2], label="sin w meas")
		plt.plot(SS4[0:LapCounter[i]-1, 4, i], uSS4[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter+3], label="sin w com")
		counter += 1
	plt.ylabel('Acc [m/s^2]')
	plt.xlabel('s [m]')

def plotClosedLoopLMPC(LMPController, grt, LapToPlot):
	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS

	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	TotNumberIt = LMPController.it
	print "Number iterations: ", TotNumberIt

	plt.figure()
	x_global_traj = grt.get_Xs()
	y_global_traj = grt.get_Ys()
	plt.plot(x_global_traj, y_global_traj, 'k') 

	yaws = grt.get_yaws()
	x_track_ib=np.zeros_like(x_global_traj)
	y_track_ib=np.zeros_like(y_global_traj)
	x_track_ob=np.zeros_like(x_global_traj)
	y_track_ob=np.zeros_like(y_global_traj)
	for i in range(len(x_global_traj)):
		x_track_ib[i]=x_global_traj[i]-LMPController.halfWidth*np.cos(yaws[i])
		y_track_ib[i]=y_global_traj[i]-LMPController.halfWidth*np.sin(yaws[i])
		x_track_ob[i]=x_global_traj[i]+LMPController.halfWidth*np.cos(yaws[i])
		y_track_ob[i]=y_global_traj[i]+LMPController.halfWidth*np.sin(yaws[i])

	plt.plot(x_track_ib, y_track_ib, 'k')
	plt.plot(x_track_ob, y_track_ob, 'k')

	
	counter = 0
	for i in LapToPlot:
		plt.plot(SS_glob[0:LapCounter[i], 4, i], SS_glob[0:LapCounter[i], 5, i], '-o', color=plotColors[counter], label=i)
		counter += 1
	plt.legend()
	plt.xlabel("x [m]")
	plt.ylabel("y [m]")

	plt.figure()
	plt.subplot(711)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 0, i], '-o', label=i, color=plotColors[counter])
		counter += 1
	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), borderaxespad=0, ncol=len(LapToPlot))

	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('vx [m/s]')
	plt.subplot(712)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 1, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('vy [m/s]')
	plt.subplot(713)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 2, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('wz [rad/s]')
	plt.subplot(714)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 3, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('epsi [rad]')
	plt.subplot(715)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i], 4, i], SS[0:LapCounter[i], 5, i], '-o', color=plotColors[counter])
		counter += 1
	plt.axvline(LMPController.trackLength, linewidth=4, color='g')
	plt.ylabel('ey [m]')
	plt.subplot(716)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 0, i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('Steering [rad]')
	plt.subplot(717)
	counter = 0
	for i in LapToPlot:
		plt.plot(SS[0:LapCounter[i]-1, 4, i], uSS[0:LapCounter[i] - 1, 1, i], '-o', color=plotColors[counter])
		counter += 1
	plt.ylabel('Acc [m/s^2]')
	plt.xlabel('s [m]')

def plotMeasuredAndAppliedSteering(LMPController, LapToPlot):
	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS

	plt.figure()
	counter = 0
	for i in LapToPlot:
		time=np.arange(0,SS[0:LapCounter[i]-1, 4, i].shape[0])
		plt.plot(time, uSS[0:LapCounter[i] - 1, 0, i], '-o', color=plotColors[counter], label="commanded Steering")
		plt.plot(time, LMPController.measSteering[0:LapCounter[i] - 1, 0, i], '--*', color=plotColors[counter], label="meausred Steering")
		counter += 1
	plt.legend()
	plt.ylabel('Steering [rad]')


def plotCompareSteering(LMPController, LMPController2, LapCompare):#LMPController3, LapCompare):
	plotColors = ['b','g','r','c','y','k','m','b','g','r','c','y','k','m']

	SS_glob = LMPController.SS_glob
	LapCounter  = LMPController.LapCounter
	SS      = LMPController.SS
	uSS     = LMPController.uSS

	SS_glob2 = LMPController2.SS_glob
	LapCounter2  = LMPController2.LapCounter
	SS2      = LMPController2.SS
	uSS2     = LMPController2.uSS

	#SS_glob3 = LMPController3.SS_glob
	# LapCounter3  = LMPController3.LapCounter
	# SS3      = LMPController3.SS
	# uSS3     = LMPController3.uSS

	plt.figure()
	counter = 0
	for i in LapCompare:
		time=np.arange(0,SS[0:LapCounter[i]-1, 4, i].shape[0])
		plt.plot(time, uSS[0:LapCounter[i] - 1, 0, i], '-o', color='b', label="commanded Steering_exp")
		plt.plot(time, LMPController.measSteering[0:LapCounter[i] - 1, 0, i], '--*', color='b', label="meausred Steering_exp")
		time2=np.arange(0,SS2[0:LapCounter2[i]-1, 4, i].shape[0])
		plt.plot(time2, uSS2[0:LapCounter2[i] - 1, 0, i], '-o', color='g', label="commanded Steering_sim")
		plt.plot(time2, LMPController2.measSteering[0:LapCounter2[i] - 1, 0, i], '--*', color='g', label="meausred Steering_sim")
		# time3=np.arange(0,SS3[0:LapCounter3[i]-1, 4, i].shape[0])
		# plt.plot(time3, uSS3[0:LapCounter3[i] - 1, 0, i], '-o', color='r', label="commanded Steering_wSRC")
		# plt.plot(time3, LMPController3.measSteering[0:LapCounter3[i] - 1, 0, i], '--*', color='r', label="meausred Steering_woSRC")
		counter += 1
	plt.legend()
	plt.ylabel('Steering [rad]')



def convertPathToGlobal(grt, s, e):
	#converts s and e vectors along a path defined by world into S and E coordinates

	pathS = grt.get_cdists()
	pathE = grt.get_Xs()
	pathN = grt.get_Ys()
	pathPsi = grt.get_yaws()

	centE = np.interp(s, pathS, pathE)
	centN = np.interp(s, pathS, pathN)
	theta = np.interp(s, pathS, pathPsi)

	E = centE - e * np.sin( np.pi / 2 - theta)
	N = centN - e * np.cos( np.pi / 2 - theta)

	return E, N

def animation_xy(grt, LMPCOpenLoopData, LMPController, it):
	SS_glob = LMPController.SS_glob
	LapCounter = LMPController.LapCounter
	SS = LMPController.SS
	uSS = LMPController.uSS

	plt.figure()
	x_global_traj = grt.get_Xs()
	y_global_traj = grt.get_Ys()
	plt.plot(x_global_traj, y_global_traj, 'k') 

	yaws = grt.get_yaws()
	x_track_ib=np.zeros_like(x_global_traj)
	y_track_ib=np.zeros_like(y_global_traj)
	x_track_ob=np.zeros_like(x_global_traj)
	y_track_ob=np.zeros_like(y_global_traj)
	for i in range(len(x_global_traj)):
		x_track_ib[i]=x_global_traj[i]-LMPController.halfWidth*np.cos(yaws[i])
		y_track_ib[i]=y_global_traj[i]-LMPController.halfWidth*np.sin(yaws[i])
		x_track_ob[i]=x_global_traj[i]+LMPController.halfWidth*np.cos(yaws[i])
		y_track_ob[i]=y_global_traj[i]+LMPController.halfWidth*np.sin(yaws[i])

	plt.plot(x_track_ib, y_track_ib, 'k')
	plt.plot(x_track_ob, y_track_ob, 'k')

	plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory",zorder=-1)

	ax = plt.axes()
	SSpoints_x = []; SSpoints_y = []
	xPred = []; yPred = []
	SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="SS",zorder=0)
	line, = ax.plot(xPred, yPred, '-or', label="Predicted Trajectory",zorder=1)

	v = np.array([[ 1.,  1.],
				  [ 1., -1.],
				  [-1., -1.],
				  [-1.,  1.]])
	rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
	ax.add_patch(rec)

	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
				mode="expand", borderaxespad=0, ncol=3)

	N = LMPController.N
	numSS_Points = LMPController.numSS_Points
	for i in range(0, int(LMPController.LapCounter[it])):

		xPred = np.zeros((N+1, 1)); yPred = np.zeros((N+1, 1))
		SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

		for j in range(0, N+1):
			if LMPCOpenLoopData.PredictedStates[j, 4, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] 

			xPred[j,0], yPred[j,0]  = convertPathToGlobal(grt, sPredicted,
															   LMPCOpenLoopData.PredictedStates[j, 5, i, it] )



			if j == 0:
				x = SS_glob[i, 4, it]
				y = SS_glob[i, 5, it]
				psi = SS_glob[i, 3, it]
				l = 0.4; w = 0.2
				car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l*np.cos(psi) + w * np.sin(psi),
						  x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
				car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
						  y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]


		for j in range(0, numSS_Points):
			if LMPCOpenLoopData.SSused[4, j, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it]

			SSpoints_x[j,0], SSpoints_y[j,0] = convertPathToGlobal(grt, sPredicted,
																	 LMPCOpenLoopData.SSused[5, j, i, it])
		SSpoints.set_data(SSpoints_x, SSpoints_y)

		line.set_data(xPred, yPred)

		rec.set_xy(np.array([car_x, car_y]).T)

		plt.draw()
		plt.pause(1e-17)

def animation_states(map, LMPCOpenLoopData, LMPController, it):
	SS_glob = LMPController.SS_glob
	LapCounter = LMPController.LapCounter
	SS = LMPController.SS
	uSS = LMPController.uSS

	xdata = []; ydata = []
	fig = plt.figure()

	axvx = fig.add_subplot(3, 2, 1)
	plt.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 0, it], '-ok', label="Closed-loop trajectory")
	lineSSvx, = axvx.plot(xdata, ydata, 'sb-', label="SS")
	linevx, = axvx.plot(xdata, ydata, 'or-', label="Predicted Trajectory")
	plt.ylabel("vx")
	plt.xlabel("s")

	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
				mode="expand", borderaxespad=0, ncol=3)

	axvy = fig.add_subplot(3, 2, 2)
	axvy.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 1, it], '-ok')
	lineSSvy, = axvy.plot(xdata, ydata, 'sb-')
	linevy, = axvy.plot(xdata, ydata, 'or-')
	plt.ylabel("vy")
	plt.xlabel("s")

	axwz = fig.add_subplot(3, 2, 3)
	axwz.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 2, it], '-ok')
	lineSSwz, = axwz.plot(xdata, ydata, 'sb-')
	linewz, = axwz.plot(xdata, ydata, 'or-')
	plt.ylabel("wz")
	plt.xlabel("s")

	axepsi = fig.add_subplot(3, 2, 4)
	axepsi.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 3, it], '-ok')
	lineSSepsi, = axepsi.plot(xdata, ydata, 'sb-')
	lineepsi, = axepsi.plot(xdata, ydata, 'or-')
	plt.ylabel("epsi")
	plt.xlabel("s")

	axey = fig.add_subplot(3, 2, 5)
	axey.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 5, it], '-ok')
	lineSSey, = axey.plot(xdata, ydata, 'sb-')
	lineey, = axey.plot(xdata, ydata, 'or-')
	plt.ylabel("ey")
	plt.xlabel("s")

	Points = np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4]))
	Points1 = np.zeros((Points, 2))
	Points2 = np.zeros((Points, 2))
	Points0 = np.zeros((Points, 2))
	for i in range(0, int(Points)):
		Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
		Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
		Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

	axtr = fig.add_subplot(3, 2, 6)
	plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
	plt.plot(Points0[:, 0], Points0[:, 1], '--')
	plt.plot(Points1[:, 0], Points1[:, 1], '-b')
	plt.plot(Points2[:, 0], Points2[:, 1], '-b')
	plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok')

	SSpoints_x = []; SSpoints_y = []
	xPred = []; yPred = []
	SSpoints_tr, = axtr.plot(SSpoints_x, SSpoints_y, 'sb')
	line_tr, = axtr.plot(xPred, yPred, '-or')

	N = LMPController.N
	numSS_Points = LMPController.numSS_Points
	for i in range(0, int(LMPController.LapCounter[it])):

		xPred    = LMPCOpenLoopData.PredictedStates[:, :, i, it]
		SSpoints = LMPCOpenLoopData.SSused[:, :, i, it]

		linevx.set_data(xPred[:, 4], xPred[:, 0])
		linevy.set_data(xPred[:, 4], xPred[:, 1])
		linewz.set_data(xPred[:, 4], xPred[:, 2])
		lineepsi.set_data(xPred[:, 4], xPred[:, 3])
		lineey.set_data(xPred[:, 4], xPred[:, 5])

		lineSSvx.set_data(SSpoints[4,:], SSpoints[0,:])
		lineSSvy.set_data(SSpoints[4,:], SSpoints[1,:])
		lineSSwz.set_data(SSpoints[4,:], SSpoints[2,:])
		lineSSepsi.set_data(SSpoints[4,:], SSpoints[3,:])
		lineSSey.set_data(SSpoints[4,:], SSpoints[5,:])

		xPred = np.zeros((N + 1, 1));yPred = np.zeros((N + 1, 1))
		SSpoints_x = np.zeros((numSS_Points, 1));SSpoints_y = np.zeros((numSS_Points, 1))

		for j in range(0, N + 1):
			xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
															 LMPCOpenLoopData.PredictedStates[j, 5, i, it])

		for j in range(0, numSS_Points):
			SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
																	   LMPCOpenLoopData.SSused[5, j, i, it])

		line_tr.set_data(xPred, yPred)
		SSpoints_tr.set_data(SSpoints_x, SSpoints_y)

		plt.draw()
		plt.pause(1e-17)

def saveGif_xyResults(grt, LMPCOpenLoopData, LMPController, it):
	SS_glob = LMPController.SS_glob
	LapCounter = LMPController.LapCounter
	SS = LMPController.SS
	uSS = LMPController.uSS

	
	fig = plt.figure()
	x_global_traj = grt.get_Xs()
	y_global_traj = grt.get_Ys()
	plt.plot(x_global_traj, y_global_traj, 'k') 

	yaws = grt.get_yaws()
	x_track_ib=np.zeros_like(x_global_traj)
	y_track_ib=np.zeros_like(y_global_traj)
	x_track_ob=np.zeros_like(x_global_traj)
	y_track_ob=np.zeros_like(y_global_traj)
	for i in range(len(x_global_traj)):
		x_track_ib[i]=x_global_traj[i]-LMPController.halfWidth*np.cos(yaws[i])
		y_track_ib[i]=y_global_traj[i]-LMPController.halfWidth*np.sin(yaws[i])
		x_track_ob[i]=x_global_traj[i]+LMPController.halfWidth*np.cos(yaws[i])
		y_track_ob[i]=y_global_traj[i]+LMPController.halfWidth*np.sin(yaws[i])

	plt.plot(x_track_ib, y_track_ib, 'k')
	plt.plot(x_track_ob, y_track_ob, 'k')

	plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory", markersize=1,zorder=-1)

	ax = plt.axes()
	SSpoints_x = []; SSpoints_y = []
	xPred = []; yPred = []
	SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="SS",zorder=0)
	line, = ax.plot(xPred, yPred, '-or', label="Predicted Trajectory",zorder=1)

	v = np.array([[ 1.,  1.],
				  [ 1., -1.],
				  [-1., -1.],
				  [-1.,  1.]])
	rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
	ax.add_patch(rec)

	plt.legend(mode="expand", ncol=3)
	# plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
	#             mode="expand", borderaxespad=0, ncol=3)

	N = LMPController.N
	numSS_Points = LMPController.numSS_Points

	def update(i):
		xPred = np.zeros((N + 1, 1)); yPred = np.zeros((N + 1, 1))
		SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

		for j in range(0, N + 1):
			if LMPCOpenLoopData.PredictedStates[j, 4, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] 

			xPred[j,0], yPred[j,0]  = convertPathToGlobal(grt, sPredicted,
															   LMPCOpenLoopData.PredictedStates[j, 5, i, it] )

			if j == 0:
				x = SS_glob[i, 4, it]
				y = SS_glob[i, 5, it]
				psi = SS_glob[i, 3, it]
				l = 0.4;w = 0.2
				car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
						 x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
				car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
						 y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

		for j in range(0, numSS_Points):
			if LMPCOpenLoopData.SSused[4, j, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it]

			SSpoints_x[j,0], SSpoints_y[j,0] = convertPathToGlobal(grt, sPredicted,
																	 LMPCOpenLoopData.SSused[5, j, i, it])

		SSpoints.set_data(SSpoints_x, SSpoints_y)

		line.set_data(xPred, yPred)

		rec.set_xy(np.array([car_x, car_y]).T)

	anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)
	
	print("Before exiting") 
	anim.save('/home/mpc/GenesisAutoware/ros/src/genesis_path_follower/scripts/gif/closedLoopXY/closedLoop.gif', dpi=80, writer='imagemagick')   
	# anim.save('/home/nkapania/catkin_ws/src/genesis_path_follower/scripts/gif/ClosedLoopXY/ClosedLoop.gif', dpi=80, writer='imagemagick')
	# anim.save('gif/ClosedLoop/ClosedLoop.gif', dpi=80, writer='imagemagick')

def saveGif_xyResultsSysID(grt, LMPCOpenLoopData, LMPController, it):
	SS_glob = LMPController.SS_glob
	LapCounter = LMPController.LapCounter
	SS = LMPController.SS
	uSS = LMPController.uSS

	Points = int(np.floor(10 * (map.PointAndTangent[-1, 3] + map.PointAndTangent[-1, 4])))
	Points1 = np.zeros((Points, 2))
	Points2 = np.zeros((Points, 2))
	Points0 = np.zeros((Points, 2))
	for i in range(0, Points):
		Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth)
		Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth)
		Points0[i, :] = map.getGlobalPosition(i * 0.1, 0)

	pdb.set_trace()
	fig = plt.figure()
	# plt.ylim((-5, 1.5))
	fig.set_tight_layout(True)
	plt.plot(map.PointAndTangent[:, 0], map.PointAndTangent[:, 1], 'o')
	plt.plot(Points0[:, 0], Points0[:, 1], '--')
	plt.plot(Points1[:, 0], Points1[:, 1], '-b')
	plt.plot(Points2[:, 0], Points2[:, 1], '-b')
	plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok', label="Closed-loop trajectory", markersize=1,zorder=-1)

	ax = plt.axes()
	SSpoints_x = []; SSpoints_y = []
	xPred = []; yPred = []
	SSpoints, = ax.plot(SSpoints_x, SSpoints_y, 'sb', label="Used Data",zorder=0)
	line, = ax.plot(xPred, yPred, '-or',zorder=1)

	v = np.array([[ 1.,  1.],
				  [ 1., -1.],
				  [-1., -1.],
				  [-1.,  1.]])
	rec = patches.Polygon(v, alpha=0.7,closed=True, fc='r', ec='k',zorder=10)
	ax.add_patch(rec)

	plt.legend(mode="expand", ncol=3)
	# plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
	#             mode="expand", borderaxespad=0, ncol=3)

	N = LMPController.N
	numSS_Points = LMPController.numSS_Points

	def update(i):
		xPred = np.zeros((N + 1, 1)); 
		yPred = np.zeros((N + 1, 1))

		scaling = np.array([[0.1, 0.0, 0.0, 0.0, 0.0],
							[0.0, 1.0, 0.0, 0.0, 0.0],
							[0.0, 0.0, 1.0, 0.0, 0.0],
							[0.0, 0.0, 0.0, 1.0, 0.0],
							[0.0, 0.0, 0.0, 0.0, 1.0]])

		xLin = np.hstack((SS[i, [0, 1, 2], it], uSS[i, :, it]))
		h = 2 * 5
		lamb = 0.0
		stateFeatures = [0, 1, 2]
		usedIt = [it - 2, it -1]

		indexSelected = []
		K = []
		LapCounter = LMPController.LapCounter
		MaxNumPoint= LMPController.MaxNumPoint
		print xLin
		for iterationUsed in usedIt:
			indexSelected_i, K_i = ComputeIndex(h, SS, uSS, LapCounter, iterationUsed, xLin, stateFeatures, scaling, MaxNumPoint,1, 0, 0)
			indexSelected.append(indexSelected_i)
			K.append(K_i)

		Counter = 0

		# Compute Matrices For Local Linear Regression
		stateFeatures = [4, 5]
		DataSysID   = np.empty((0,len(stateFeatures)))

		for indexIterations in usedIt:
			DataSysID = np.append( DataSysID, np.squeeze(SS_glob[np.ix_(indexSelected[Counter], stateFeatures, [indexIterations])]), axis=0)
			Counter = Counter + 1

		# pdb.set_trace()

		# SSpoints_x = np.zeros((numSS_Points, 1)); SSpoints_y = np.zeros((numSS_Points, 1))

		# for j in range(0, N + 1):
		#     xPred[j, 0], yPred[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.PredictedStates[j, 4, i, it],
		#                                                      LMPCOpenLoopData.PredictedStates[j, 5, i, it])

		#     if j == 0:
		#         x = SS_glob[i, 4, it]
		#         y = SS_glob[i, 5, it]
		#         psi = SS_glob[i, 3, it]
		#         l = 0.4;w = 0.2
		#         car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
		#                  x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
		#         car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
		#                  y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

		# for j in range(0, numSS_Points):
		#     SSpoints_x[j, 0], SSpoints_y[j, 0] = map.getGlobalPosition(LMPCOpenLoopData.SSused[4, j, i, it],
		#                                                                LMPCOpenLoopData.SSused[5, j, i, it])
		SSpoints.set_data(DataSysID[:,0], DataSysID[:,1])

		x = SS_glob[i, 4, it]
		y = SS_glob[i, 5, it]
		psi = SS_glob[i, 3, it]
		l = 2*0.15;
		w = 2*0.075
		car_x = [x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
				 x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
		car_y = [y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
				 y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]
		rec.set_xy(np.array([car_x, car_y]).T)

	anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)

	anim.save('gif/ClosedLoop/ClosedLoop.gif', dpi=80, writer='imagemagick')



def ComputeIndex(h, SS, uSS, LapCounter, it, x0, stateFeatures, scaling, MaxNumPoint, ConsiderInput, steeringDelay, idDelay):
	startTimer = datetime.datetime.now()  # Start timer for LMPC iteration

	# What to learn a model such that: x_{k+1} = A x_k  + B u_k + C
	startTime = 0
	endTime   = LapCounter[it] - 1

	oneVec = np.ones( (SS[startTime:endTime, :, it].shape[0], 1) )

	x0Vec = (np.dot( np.array([x0]).T, oneVec.T )).T

	if ConsiderInput == 1:
		DataMatrix = np.hstack((SS[startTime:endTime, stateFeatures, it], uSS[startTime:endTime, :, it]))
	else:
		DataMatrix = SS[startTime:endTime, stateFeatures, it]

	diff  = np.dot(( DataMatrix - x0Vec ), scaling)
	# print 'x0Vec \n',x0Vec
	norm = la.norm(diff, 1, axis=1)
	
	# Need to make sure that the indices [0:steeringDelay] are not selected as it us needed to shift the input vector
	if (steeringDelay+idDelay) > 0:
		norm[0:(steeringDelay+idDelay)] = 10000

	indexTot =  np.squeeze(np.where(norm < h))
	# print indexTot.shape, np.argmin(norm), norm, x0
	if (indexTot.shape[0] >= MaxNumPoint):
		index = np.argsort(norm)[0:MaxNumPoint]
		# MinNorm = np.argmin(norm)
		# if MinNorm+MaxNumPoint >= indexTot.shape[0]:
		#     index = indexTot[indexTot.shape[0]-MaxNumPoint:indexTot.shape[0]]
		# else:
		#     index = indexTot[MinNorm:MinNorm+MaxNumPoint]
	else:
		index = indexTot

	K  = ( 1 - ( norm[index] / h )**2 ) * 3/4
	# K = np.ones(len(index))

	return index, K

def Save_statesAnimation(grt, LMPCOpenLoopData, LMPController, it):
	SS_glob = LMPController.SS_glob
	LapCounter = LMPController.LapCounter
	SS = LMPController.SS
	uSS = LMPController.uSS

	xdata = []; ydata = []
	fig = plt.figure()
	fig.set_tight_layout(True)

	axvx = fig.add_subplot(3, 2, 1)
	plt.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 0, it], '-ok', label="Closed-loop trajectory")
	lineSSvx, = axvx.plot(xdata, ydata, 'sb-', label="SS")
	linevx, = axvx.plot(xdata, ydata, 'or-', label="Predicted Trajectory")
	plt.ylabel("vx")
	plt.xlabel("s")

	plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left",
				mode="expand", borderaxespad=0, ncol=3)

	axvy = fig.add_subplot(3, 2, 2)
	axvy.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 1, it], '-ok')
	lineSSvy, = axvy.plot(xdata, ydata, 'sb-')
	linevy, = axvy.plot(xdata, ydata, 'or-')
	plt.ylabel("vy")
	plt.xlabel("s")

	axwz = fig.add_subplot(3, 2, 3)
	axwz.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 2, it], '-ok')
	lineSSwz, = axwz.plot(xdata, ydata, 'sb-')
	linewz, = axwz.plot(xdata, ydata, 'or-')
	plt.ylabel("wz")
	plt.xlabel("s")

	axepsi = fig.add_subplot(3, 2, 4)
	axepsi.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 3, it], '-ok')
	lineSSepsi, = axepsi.plot(xdata, ydata, 'sb-')
	lineepsi, = axepsi.plot(xdata, ydata, 'or-')
	plt.ylabel("epsi")
	plt.xlabel("s")

	axey = fig.add_subplot(3, 2, 5)
	axey.plot(SS[0:LapCounter[it], 4, it], SS[0:LapCounter[it], 5, it], '-ok')
	lineSSey, = axey.plot(xdata, ydata, 'sb-')
	lineey, = axey.plot(xdata, ydata, 'or-')
	plt.ylabel("ey")
	plt.xlabel("s")

	axtr = fig.add_subplot(3, 2, 6)
	x_global_traj = grt.get_Xs()
	y_global_traj = grt.get_Ys()
	plt.plot(x_global_traj, y_global_traj, 'k') 

	yaws = grt.get_yaws()
	x_track_ib=np.zeros_like(x_global_traj)
	y_track_ib=np.zeros_like(y_global_traj)
	x_track_ob=np.zeros_like(x_global_traj)
	y_track_ob=np.zeros_like(y_global_traj)
	for i in range(len(x_global_traj)):
		x_track_ib[i]=x_global_traj[i]-LMPController.halfWidth*np.cos(yaws[i])
		y_track_ib[i]=y_global_traj[i]-LMPController.halfWidth*np.sin(yaws[i])
		x_track_ob[i]=x_global_traj[i]+LMPController.halfWidth*np.cos(yaws[i])
		y_track_ob[i]=y_global_traj[i]+LMPController.halfWidth*np.sin(yaws[i])

	plt.plot(x_track_ib, y_track_ib, 'k')
	plt.plot(x_track_ob, y_track_ob, 'k')

	plt.plot(SS_glob[0:LapCounter[it], 4, it], SS_glob[0:LapCounter[it], 5, it], '-ok')

	SSpoints_x = []; SSpoints_y = []
	xPred = []; yPred = []
	SSpoints_tr, = axtr.plot(SSpoints_x, SSpoints_y, 'sb')
	line_tr, = axtr.plot(xPred, yPred, '-or')

	N = LMPController.N
	numSS_Points = LMPController.numSS_Points

	def update(i):
		xPred    = LMPCOpenLoopData.PredictedStates[:, :, i, it]
		SSpoints = LMPCOpenLoopData.SSused[:, :, i, it]

		linevx.set_data(xPred[:, 4], xPred[:, 0])
		linevy.set_data(xPred[:, 4], xPred[:, 1])
		linewz.set_data(xPred[:, 4], xPred[:, 2])
		lineepsi.set_data(xPred[:, 4], xPred[:, 3])
		lineey.set_data(xPred[:, 4], xPred[:, 5])

		lineSSvx.set_data(SSpoints[4,:], SSpoints[0,:])
		lineSSvy.set_data(SSpoints[4,:], SSpoints[1,:])
		lineSSwz.set_data(SSpoints[4,:], SSpoints[2,:])
		lineSSepsi.set_data(SSpoints[4,:], SSpoints[3,:])
		lineSSey.set_data(SSpoints[4,:], SSpoints[5,:])

		xPred = np.zeros((N + 1, 1));yPred = np.zeros((N + 1, 1))
		SSpoints_x = np.zeros((numSS_Points, 1));SSpoints_y = np.zeros((numSS_Points, 1))

		for j in range(0, N + 1):
			if LMPCOpenLoopData.PredictedStates[j, 4, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.PredictedStates[j, 4, i, it] 

			xPred[j,0], yPred[j,0]  = convertPathToGlobal(grt, sPredicted,
															   LMPCOpenLoopData.PredictedStates[j, 5, i, it] )

		for j in range(0, numSS_Points):
			if LMPCOpenLoopData.SSused[4, j, i, it] > LMPController.trackLength:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it] - LMPController.trackLength
			else:
				sPredicted = LMPCOpenLoopData.SSused[4, j, i, it]

			SSpoints_x[j,0], SSpoints_y[j,0] = convertPathToGlobal(grt, sPredicted,
																	 LMPCOpenLoopData.SSused[5, j, i, it])

		line_tr.set_data(xPred, yPred)
		SSpoints_tr.set_data(SSpoints_x, SSpoints_y)

	anim = FuncAnimation(fig, update, frames=np.arange(0, int(LMPController.LapCounter[it])), interval=100)
	rospack = rospkg.RosPack()
	pkg_path=rospack.get_path('genesis_path_follower')
	# anim.save(pkg_path+'/scripts/gif/closedLoop/ClosedLoop.gif', dpi=80, writer='imagemagick')
	# anim.save('/home/mpc/GenesisAutoware/ros/src/genesis_path_follower/scripts/gif/closedLoopState/closedLoop.gif', dpi=80, writer='imagemagick')   
	anim.save('/home/mpc-ubuntu/catkin/src/genesis_path_follower/scripts/gif/closedLoopState/closedLoop.gif', dpi=80, writer='imagemagick')   


main()