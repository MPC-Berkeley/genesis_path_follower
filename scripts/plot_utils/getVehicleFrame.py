# This is a helper script written by Nitin Kapania and used by Vijay Govindarajan for plotting the vehicle geometry.
# Original script is from: https://github.com/nkapania/Wolverine/blob/master/utils/sim_lib.py
import numpy as np


def plotVehicle(posE, posN, psi, delta, a, b, d, rW):
	# VG EDIT: use psi (yaw angle) to be compatible with this code.
	#returns position of vehicle frame given coordinates of vehicle cg, steer angle, and dimensions a, b, d, and rW
	
	FrontAxle_Center_x = posE + a*np.cos(psi);
	FrontAxle_Center_y = posN + a*np.sin(psi);

	RearAxle_Center_x = posE - b*np.cos(psi);
	RearAxle_Center_y = posN - b*np.sin(psi);

	FrontAxle_Right_x = FrontAxle_Center_x + (d/2)*np.sin(psi);
	FrontAxle_Right_y = FrontAxle_Center_y - (d/2)*np.cos(psi);

	FrontAxle_Left_x = FrontAxle_Center_x - (d/2)*np.sin(psi);
	FrontAxle_Left_y = FrontAxle_Center_y + (d/2)*np.cos(psi);

	RearAxle_Right_x = RearAxle_Center_x + (d/2)*np.sin(psi);
	RearAxle_Right_y = RearAxle_Center_y - (d/2)*np.cos(psi);

	RearAxle_Left_x = RearAxle_Center_x - (d/2)*np.sin(psi);
	RearAxle_Left_y = RearAxle_Center_y + (d/2)*np.cos(psi);

	RightFrontTire_Front_x = FrontAxle_Right_x + rW*np.cos(psi+delta);
	RightFrontTire_Front_y = FrontAxle_Right_y + rW*np.sin(psi+delta);

	RightFrontTire_Back_x = FrontAxle_Right_x - rW*np.cos(psi+delta);
	RightFrontTire_Back_y = FrontAxle_Right_y - rW*np.sin(psi+delta);

	RightRearTire_Front_x = RearAxle_Right_x + rW*np.cos(psi);
	RightRearTire_Front_y = RearAxle_Right_y + rW*np.sin(psi);

	RightRearTire_Back_x = RearAxle_Right_x - rW*np.cos(psi);
	RightRearTire_Back_y = RearAxle_Right_y - rW*np.sin(psi);

	LeftFrontTire_Front_x = FrontAxle_Left_x + rW*np.cos(psi+delta);
	LeftFrontTire_Front_y = FrontAxle_Left_y + rW*np.sin(psi+delta);

	LeftFrontTire_Back_x = FrontAxle_Left_x - rW*np.cos(psi+delta);
	LeftFrontTire_Back_y = FrontAxle_Left_y - rW*np.sin(psi+delta);

	LeftRearTire_Front_x = RearAxle_Left_x + rW*np.cos(psi);
	LeftRearTire_Front_y = RearAxle_Left_y + rW*np.sin(psi);

	LeftRearTire_Back_x = RearAxle_Left_x - rW*np.cos(psi);
	LeftRearTire_Back_y = RearAxle_Left_y - rW*np.sin(psi);


	FrontBody =  np.array([[posE, FrontAxle_Center_x], [posN, FrontAxle_Center_y]]).squeeze()
	RearBody  =  np.array([[posE, RearAxle_Center_x] , [posN, RearAxle_Center_y]]).squeeze()
	FrontAxle =  np.array([[FrontAxle_Left_x, FrontAxle_Right_x], [FrontAxle_Left_y, FrontAxle_Right_y]]).squeeze()
	RearAxle  =  np.array([[RearAxle_Left_x, RearAxle_Right_x], [RearAxle_Left_y, RearAxle_Right_y]]).squeeze()
	RightFrontTire = np.array([[RightFrontTire_Front_x, RightFrontTire_Back_x],  [RightFrontTire_Front_y, RightFrontTire_Back_y]]).squeeze()
	RightRearTire  = np.array([[RightRearTire_Front_x, RightRearTire_Back_x],    [RightRearTire_Front_y, RightRearTire_Back_y]]).squeeze()
	LeftFrontTire  = np.array([[LeftFrontTire_Front_x, LeftFrontTire_Back_x],    [LeftFrontTire_Front_y, LeftFrontTire_Back_y]]).squeeze()
	LeftRearTire   = np.array([[LeftRearTire_Front_x, LeftRearTire_Back_x],      [LeftRearTire_Front_y, LeftRearTire_Back_y]]).squeeze()

	return FrontBody, RearBody, FrontAxle, RearAxle, RightFrontTire, RightRearTire, LeftFrontTire, LeftRearTire
