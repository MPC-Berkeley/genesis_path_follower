import numpy as np
import matplotlib.pyplot as plt
#Defines a velocity profile class


#This speed profile generation algorithm is based on a simple "3 pass" method that accounts for steady state speeds
#and integration according to a friction circle. It is best used for educational purposes or for very simple lanekeeping
#demonstrations well below the limits. It tends to cause instability in braking regions due to improper handling of weight
#transfer and generally aggressive braking in the straight segments. 

class BasicProfile():
    def __init__(self, vehicle, path, friction = 0.3, vMax = 10., AxMax = 9.81):
		self.vehicle = vehicle
		self.path = path
		self.mu = friction
		self.vMax = vMax

	    #initialize variables
		self.s = path.s
		self.Ux = np.zeros(self.s.shape)
		self.Ax = np.zeros(self.s.shape)
	    
		if path.isOpen:
			self.generateBasicProfileOpen(AxMax)

		else:
			self.generateBasicProfileClosed(AxMax)

    def generateBasicProfileClosed(self, AxMax):
	    g = 9.81
	    K = self.path.curvature
	    s = self.s
	    AyMax = self.mu* g
	    AxMax = min( abs(AxMax) , self.mu * g) 

	    #calculate lowest velocity point
	    UxSS = np.sqrt ( np.divide(AyMax, np.abs(K + 1e-8) ) )
	    minUx = np.amin(UxSS)
	    idx = np.argmin(UxSS)

	    #shift so we generate starting at lowest point
	    inds = np.arange(len(UxSS))
	    shiftedInds = np.roll(inds, -idx)
	    kShifted = K[shiftedInds]

	    UxShift, AxShift = self.genSpeed(kShifted, minUx, AxMax, AyMax)

	    #unshift back to original
	    self.Ux = np.roll(UxShift, idx)
	    self.Ax = np.roll(AxShift, idx)

	    return

    def generateBasicProfileOpen(self, AxMax):
    	g = 9.81
    	K = self.path.curvature
    	AyMax = self.mu* g
    	AxMax = min( abs(AxMax) , self.mu * g)
    	self.Ux, self.Ax = self.genSpeed(K, 0, AxMax, AyMax) #minimum velocity is zero

    def genSpeed(self, K, minUx, AxMax, AyMax):
	    #Extract Peformance Limits and parameters
	    g = 9.81
	    maxUx = self.vMax
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
	        
	         if temp > maxUx:
	             temp = maxUx

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















#This is a more sophisticated velocity profile generation algorithm that accounts for weight 
#transfer, front and rear wheel friction limitations, etc. Note that it is not the full version
#published by Subosits and Gerdes acccounting for bank, grade, and vertical curvature. 

#attribution: John Subosits, Mick Kritayakirana

# class RacingProfile():
#     def __init__(self, vehicle, path, friction = 0.3, vMax = 10):
#         self.vehicle = vehicle
#         self.path = path
#         self.mu = friction
#         self.vMax = vMax
#         self.dynamicBeta = self.vehicle.beta #changes over time as braking, deceling  

#         #initialize variables
#         self.s = path.s
#         self.Ux = np.zeros(self.s.shape)
#         self.Ax = np.zeros(self.s.shape)
        
#         self.findSpeedProfile()
        
#         return self.s, self.Ax, self.Ux


#     def findSpeedProfile(self):
#         n = len(self.s)
#         UxInit1 = np.zeros(self.s.shape) #for constant steady state speed
#         UxInit2 = np.zeros(self.s.shape) #for integrating forwards
#         UxInit3 = np.zeros(self.s.shape) #for integrating backwards - final velocity profile
#         self.Ax = np.zeros(self.s.shape)

#         #find velocity minimum based on pointwise friction constraint
#         UxInit1 = self.getSteadyStateVelocity()

#         #integrate backward and take the minimum
#         UxInit2 = self.integrateBackward(UxInit1)

#         #integrate forwards and take the minimum
#         self.integrateForward(UxInit2)

#         return


#     def integrateBackward(self, UxInit1, AxDes):
#     	a = self.vehicle.a
#     	b = self.vehicle.b
#     	h = self.vehicle.h
#     	m = self.vehicle.m
#     	D = self.vehicle.D
#     	K = self.path.curvature

#     	Vsquared = endSpeed ** 2
#     	numPoints = len(self.s)


#     	#Integrate backwards
#     	for i in reversed(range(numPoints)):
#     		Ux[-1] = np.sqrt(Vsquared)
#     		Ax[-1] = AxDes

#     		#find limits on front and rear tires and max speed
#     		axF = DecelFront(Vsquared, K[i], dK[i]*Iz, K[i]*Iz)
#     		axR = DecelRear(Vsquared, K[i], dK[i]*Iz, K[i]*Iz)

#     		#Vmax = UxInit1[]
    	

#         return UxInit2
        
#     def integrateForward(self, UxInit2):
#         return



#     #% finds the deceleration achievable by the front tires and the front/rear
# 	# % weight disribution beta.  Parameter usage is consistent with previous
# 	# % examples.

#     def getFrontDecelLimit(self, Vsquared, K, dK, kIz):
# 		L = self.vehicle.a + self.vehicle.b
# 		D = 0 #Air drag ignored to compensate for increased brake temperatures in higher speed corners.
# 		g3 = self.vehicle.g;
# 		fv23 = K
# 		mv23 = dK
# 		mvdot3 = kIz;

# 		beta = self.vehicle.beta
# 		m = self.vehicle.m

# 		# find terms for vdot equation
# 		g = beta*m/(1 + beta);
# 		h = 0;
# 		c = mvdot3/L; 
# 		d = mv23*Vsquared/L;
# 		e = -(m*hcg)/L;
# 		f = m*b*(fv23*Vsquared+g3) 


# 		A = g**2 + c**2  - mu**2*e**2;
# 		B = 2*(g*h + c*d - mu**2*(e)*(f));
# 		C = (h)**2 + (d)**2 - mu**2*(f)**2;

# 		#Use negative solution to quadratic eqn.
# 		vdot = (-B - np.sqrt(B**2 - 4*A*C))/(2*A) 
# 		if np.isnan(vdot) || (vdot > 0):
# 		    vdot = 0   # if limits exceeded, grade and drag only
		
		
# 		Fzf = e*vdot + f;
# 		Fzr = m*(fv23*Vsquared + g3) - Fzf;
# 		self.dynamicBeta = Fzf/Fzr
# 		vdot = vdot * self.vehicle.brakeFactor

# 		return vdot
    	


#     def getRearDecelLimit(self, Vsquared, K, dK, kIz):
# 		L = self.vehicle.a+ self.vehicle.b;
# 		g1 = 0;
# 		g2 = 0;
# 		g3 = self.vehicle.g;
# 		fv22 = 0;
# 		fv23 = K;
# 		mv22 = 0;
# 		mv23 = dK*self.vehicle*Iz;
# 		mvdot2 = 0;
# 		mvdot3 = K*Iz;

# 		g = m/(1 + self.dynamicBeta);
# 		h = 0
# 		c = -mvdot3/L;
# 		d = (-mv23*Vsquared)/L;
# 		e = (m*hcg)/L;
# 		f = (m*a*(fv23*Vsquared+g3))

# 		A = g**2 + (c)**2 - mu**2*(e)**2;
# 		B = 2*((g)*(h) + (c)*(d) - mu**2*(e)*(f));
# 		C = (h)**2 + (d)**2 - mu**2*(f)**2;


# 		# use negative solution
# 		vdot = (-B - np.sqrt(B**2 - 4*A*C))/(2*A)
# 		if (isnan(vdot) || (vdot > 0))
# 		    vdot = 0   
		
# 		Fzr = e*vdot + f
# 		Fzf = m*(fv23*Vsquared + g3) - Fzr
# 		self.dynamicBeta = Fzf/Fzr
# 		vdot = vdot*self.vehicle.brakeFactor    	

#     def getSteadyStateVelocity(self):
#         a = self.vehicle.a
#         b = self.vehicle.b
#         L = a + b
#         m = self.vehicle.m
#         D = self.vehicle.D
#         hcg = self.vehicle.h
#         beta = self.vehicle.beta #ratio of front to rear wheel torques
#         K = self.path.curvature
#         s = self.s
#         g = self.vehicle.g
#         mu = self.mu
#         Iz = self.vehicle.Iz

#         dK  = np.divide( np.diff(K) , np.diff(s) )
#         dK  = np.insert(dK, 0, 0) #prepend zero to keep same size

#         ####################FRONT WHEELS################################

#         #Front wheels terms for Ux**2
#         g = beta*D/(1+beta)
#         h = beta*(m*g)/(1+beta)
#         d = 0. #only nonzero when considering bank, grade and vertical curvature. we are not. 
#         e = -hcg*D/L
#         f = self.vehicle.FzF #FzF
        
#         #arrays for equation
#         c =  (b*m*K + dK * self.vehicle.Iz)/L #array

#         A = g**2 + np.multiply(c, c) - mu**2 * e ** 2 #array
#         B = 2 * (g*h + c*d - mu**2 * e* f)
#         C = h **2 + d**2 - mu**2 * f **2 

#         #use positive soln to quadratic eqn
#         VmaxF = np.divide (np.sqrt( -B + np.sqrt( B ** 2 - 4*A*C)) , 2*A )
#         nonReal = np.isnan(VmaxF)
#         noFricLim = nonReal & (A < 0) & (B<0)

#         VmaxF[noFricLim] = 999. #avoid setting equal to inf as this causes issues


#         ###################REAR WHEELS #######################################
#         #######################(SHOULD BE IN FUNCTION BUT IT'S OK) ###########
#         mu = mu * self.vehicle.muR / self.vehicle.muF; # encodes the steady state understeer / oversteer of the car
        
#         g = D/(1+beta);
#         h = m*g/(1+beta);
#         c = (a*m*K - dK*Iz)/L;
#         d = 0.
#         e = hcg*D/L
#         f = self.vehicle.FzR;

#         A = g**2 + np.multiply(c, c) - mu**2 * e ** 2 #array
#         B = 2*(g*h + c*d - mu**2 * e * f)
#         C = h**2 + d**2 - mu**2 * f ** 2

#         VmaxR = np.divide (np.sqrt( -B + np.sqrt( B ** 2 - 4*A*C)) , 2*A )
#         nonReal = np.isnan(VmaxR)
#         noFricLim = nonReal & (A < 0) & (B < 0)

#         VmaxR[noFricLim] = 999.

#         #limit to vMax
#         Ux = np.minimum(VmaxF, VmaxR)
#         Ux[Ux > self.vMax] = vMax

#         return Ux, dK

























