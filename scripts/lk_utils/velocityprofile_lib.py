import numpy as np
import matplotlib.pyplot as plt
#Defines a velocity profile class

#currently just uses Shelley values
class VelocityProfile:
	def __init__(self, profileType):
		self.type = profileType
		self.s = np.array([[0]])
		self.Ux = np.array([[0]]) 
		self.Ax = np.array([[0]])


	def generate(self, vehicle, path):
		if self.type is "racing":
			self.s, self.Ax, self.Ux = generateRacingProfile(vehicle, path)

		else:
			print("Error")

		



def generateRacingProfile(vehicle, path):
	#Extract Peformance Limits and parameters
	g = vehicle.g
	vMax = 5.0 ##DO NOT COMMIT
	AxMax = path.friction * g
	AyMax = path.friction * g
	K = path.curvature
	s = path.s

	numSteps = path.s.size
	
	#Pre-allocate three velocity profiles (steady state, braking, decel)
	UxInit1 = np.zeros(numSteps)
	UxInit2 = np.zeros(numSteps)
	UxInit3 = np.zeros(numSteps)

	#Pre-allocate Ax and Ay
	ax = np.zeros(numSteps)
	ay = np.zeros(numSteps)

	


	#Desired velocity should meet lateral acceleration requirement
	UxInit1 = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )

	# #Integrate forward to find acceleration limit
	for i in range(UxInit2.size-1):
	 	temp = np.sqrt( UxInit2[i]**2 + 2*AxMax*(s[i+1] - s[i]))
		
	 	if temp > vMax:
	 		temp = vMax

	 	if temp > UxInit1[i+1]:
	 		temp = UxInit1[i+1]

	 	UxInit2[i+1] = temp

	#Moving rearward, integrate backwards
	for i in reversed(range(1,UxInit3.size)):
		temp = np.sqrt( UxInit3[i]**2 + 2* AxMax * (s[i] - s[i-1]) )
		

		if temp > UxInit2[i-1]:
			temp = UxInit2[i-1]

		UxInit3[i-1] = temp



	#calculate acceleration profile from physics
	ax = np.divide( (np.roll(UxInit3,1)**2 - UxInit3**2) , (2 * (np.roll(s,1) - path.s) ) )
	ax[0] = ax[1] #avoid bug where vehicle starts with initial desired acceleration
	

	return s, ax, UxInit3






















