import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
#Defines a velocity profile class


#This speed profile generation algorithm is based on a simple "3 pass" method that accounts for steady state speeds
#and integration according to a friction circle. It is best used for educational purposes or for very simple lanekeeping
#demonstrations well below the limits. It tends to cause instability in braking regions due to improper handling of weight
#transfer and generally aggressive braking in the straight segments. 

class PrecalculatedProfile():
	def __init__(self, filePath):
		profile = sio.loadmat(filePath, squeeze_me = True)
		self.s  =  profile['vp']['s'].sum()
		self.Ux =  profile['vp']['Ux'].sum()
		self.Ax =  profile['vp']['Ax'].sum()


class BasicProfile():
    def __init__(self, vehicle, path, friction = 0.3, vMax = 10., AxMax = 9.81):
		self.vehicle = vehicle
		self.path = path

	    #initialize variables
		self.s = path.s
		self.Ux = np.zeros(self.s.shape)
		self.Ax = np.zeros(self.s.shape)

		if isinstance(vMax, np.ndarray):
			self.vMax = vMax
		else: 
			self.vMax = vMax * np.ones(self.s.shape)

		if isinstance(friction, np.ndarray):
			self.mu = friction
		else:
			self.mu = friction * np.ones(self.s.shape)



	    
		if path.isOpen:
			self.generateBasicProfileOpen(AxMax)

		else:
			self.generateBasicProfileClosed(AxMax)

    def generateBasicProfileClosed(self, AxMax):
	    g = 9.81
	    K = self.path.curvature
	    s = self.s
	    AyMax = self.mu* g
	    AxMax = min( np.append(self.mu * g, abs(AxMax)))

	    #calculate lowest velocity point
	    UxSS = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )
	    minUx = np.amin(UxSS)
	    maxUx = self.vMax
	    idx = np.argmin(UxSS)

	    #shift so we generate starting at lowest point
	    inds = np.arange(len(UxSS))
	    shiftedInds = np.roll(inds, -idx)
	    kShifted = K[shiftedInds]
	    maxUxShifted = maxUx[shiftedInds]
	    AyMaxShifted = AyMax[shiftedInds]

	    UxShift, AxShift = self.genSpeed(kShifted, minUx, maxUxShifted, AxMax, AyMaxShifted)

	    #unshift back to original
	    self.Ux = np.roll(UxShift, idx)
	    self.Ax = np.roll(AxShift, idx)

	    return

    def generateBasicProfileOpen(self, AxMax):
    	g = 9.81
    	K = self.path.curvature
    	AyMax = self.mu* g
    	AxMax = min( np.append(self.mu * g, abs(AxMax)))
    	self.Ux, self.Ax = self.genSpeed(K, 0, self.vMax, AxMax, AyMax) #minimum velocity is zero

    def genSpeed(self, K, minUx, maxUx, AxMax, AyMax):
	    #Extract Peformance Limits and parameters
	    g = 9.81
	    s = self.s
	    
	    numSteps = s.size
	    
	    #Pre-allocate three velocity profiles (steady state, braking, decel)
	    UxInit1 = np.zeros(numSteps)
	    UxInit2 = np.zeros(numSteps); UxInit2[0]  = minUx
	    UxInit3 = np.zeros(numSteps); UxInit3[-1] = minUx 

	    #Pre-allocate Ax and Ay
	    ax = np.zeros(numSteps)
	    ay = np.zeros(numSteps)


	    #Desired velocity should meet lateral acceleration requirement
	    UxInit1 = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )

	    # #Integrate forward to find acceleration limit
	    for i in range(UxInit2.size-1):
	         temp = np.sqrt( UxInit2[i]**2 + 2*AxMax*(s[i+1] - s[i]))
	         
	         if temp > maxUx[i]:
	             temp = maxUx[i]

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
	    ax = np.divide( (np.roll(UxInit3,1)**2 - UxInit3**2) , (2 * (np.roll(s,1) - s) ) )
	    ax[0] = ax[1] #avoid bug where vehicle starts with initial desired acceleration
	    

	    return UxInit3, ax
















