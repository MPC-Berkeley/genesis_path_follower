import numpy as np
import tiremodel_lib as tm
import vehicle_lib
import path_lib
from controllers import *
import time

#Defines a simulation class and a state class

class Simulation:
    def __init__(self, vehicle, controller, path = None, profile = None, mapMatchType = "euler", maxTime = None): 
        self.path = path
        self.vehicle = vehicle
        self.profile = profile
        self.controller = controller
        self.isRunning = True
        self.physics = "bicycle"
        self.logger = Logger()
        self.ts = 0.01 #simulation time in seconds
        self.mapMatchType = mapMatchType
        self.maxTime = maxTime
        
        
    def simulate(self):
        ##initialize states and instantiate objects

        if self.profile is None:
            Ux0 = 10.  #specify start state
        else:
            Ux0 = self.profile.Ux[0] #start car at the initial velocity specified in the path

        localState = LocalState(Ux0)
        globalState = GlobalState(self.path)
        controlInput = ControlInput()
        log = Logger()
        mapMatch = MapMatch(self.path, self.mapMatchType)


        ##Start the Counter
        counter = 0

        ##Run the simulation!
        while self.isRunning:
            
            #Perform localization
            mapMatch.localize(localState, globalState)

            #Check to see if we should terminate
            self.checkForTermination(localState, counter, log)

            #Calculate controller inputs
            auxVars = self.controller.updateInput(localState, controlInput)            

            #Update state
            self.updateState(controlInput, localState, globalState, auxVars)

            #Append counter and print to screen
            counter = counter + 1
            self.printStatus(localState, counter)


            #save signals needed

            UxDes = auxVars["UxDes"]                
            log.append('t',counter*self.ts)
            log.append('Ux',localState.Ux)
            log.append('s', localState.s)
            log.append('e', localState.e)
            log.append('dPsi',localState.dPsi)
            log.append('UxDes', UxDes) 
            log.append('posE', globalState.posE)
            log.append('posN', globalState.posN)
            log.append('psi', globalState.psi)
            log.append('r', localState.r)
            log.append('Uy', localState.Uy)
            log.append('deltaCmd', controlInput.delta)
            log.append('FxCmd', controlInput.Fx)
            log.incrementCounter()



        return log.getData()





    def checkForTermination(self, localState, counter, log):

        #Check if we have ended the simulation. If running open loop, stop after maximum time is specified. Otherwise stop once we reach the end of the track or if we go unstable

        
        if self.path is None:  
            assert (self.maxTime != None), "Must specify time to run"
            t = counter * self.ts
            if t > self.maxTime:
                self.isRunning = False

        else:
            if localState.s > (self.path.s[-1] - 0.55): #Stop simulation a little before end of path
                self.isRunning = False
                runTime = counter * self.ts
                print("Simulation complete - total time %.2f sec" %runTime)             

            #Check if we have gone off the track    
            if abs(localState.e) > 5.0:
                print("Car has left the track - terminating...")
                self.isRunning = False

    def printStatus(self, localState, counter):
        if self.path is None:
            t = counter * self.ts 
            pctComplete = np.ceil( 100 * t / self.maxTime)
        else:
            pctComplete = np.ceil( 100 * localState.s / self.path.s[-1] )

        if np.mod(counter, 100) == 0:
            print("Simulation is %02d percent done" % pctComplete)
            #print(pctComplete)
            #print("Distance Along Path is %04d meters") % localState.s


    def updateState(self, controlInput, localState, globalState, auxVars):
        K = auxVars["K"]
        UxDes = auxVars["UxDes"]
        if self.physics is "bicycle":
            localState, globalState = bicycleModel(self.vehicle, controlInput, localState, globalState, self.mapMatchType, self.ts, K)
            return localState, globalState

class LocalState:
    def __init__(self, Ux=0.0, Uy=0.0, r=0.0, e=0.0, deltaPsi=0.0, s=0.0):
        self.Ux = Ux
        self.Uy = Uy
        self.r = r
        self.e = e
        self.deltaPsi = deltaPsi
        self.s = s
        
    def update(self, Ux = 0, Uy = 0, r = 0, e = 0, deltaPsi = 0, s = 0, UxDes=0):
        self.Ux = Ux
        self.Uy = Uy
        self.r  = r
        self.e  = e
        self.deltaPsi = deltaPsi
        self.s = s
        self.UxDes=UxDes
    def updateMapMatchStates(self, e, dPsi, s):
        self.e = e
        self.deltaPsi = dPsi
        self.s = s
        
    def updateVelocityState(self, Ux, Uy, r):
        self.Ux = Ux
        self.Uy = Uy
        self.r  = r
        self.UxDes=UxDes
    def printState(self):
        print("Ux is %.2f" %self.Ux) 
        print("Uy is %.2f" %self.Uy)
        print("r is %.2f" %self.r)
        print("e is %.2f" %self.e)
        print("deltaPsi is %.2f" %self.deltaPsi)
        print("s is %.2f" %self.s)

        
class GlobalState:
    def __init__(self, path, offset = [0, 0, 0]):

        #Start at 0 IC's if path is None
        if path is None: 
            self.posE = 0
            self.posN = 0
            self.psi = 0

        else:
            self.posE = path.posE[1] + offset[0] #start at second element of array to avoid mapMatch issues
            self.posN = path.posN[1] + offset[1]
            self.psi  = path.roadPsi[1] + offset[2]

    def update(self, posE = 0, posN = 0, psi = 0):
        self.posE = posE
        self.posN = posN
        self.psi  = psi

class Logger:
    #Probably a better way to decide this
    def __init__(self, NUMBER_DATA_POINTS = 100000):
        self.data = {}
        self.counter = 0
        self.NUMBER_DATA_POINTS = NUMBER_DATA_POINTS

    def append(self, signalName, signalData):
        if signalName in self.data.keys():
            self.data[signalName][self.counter] = signalData

        else:
            #create array once and append
            self.data[signalName] = np.zeros( (self.NUMBER_DATA_POINTS, 1) )
            self.data[signalName][0] = signalData

    def incrementCounter(self):
        self.counter = self.counter + 1


    def getData(self):
        #remove trailing zeros
        for key in self.data.keys():
            object = self.data[key]
            self.data[key] = self.data[key][0:self.counter-1, :]

        #Add additional info
        self.data["N"] = self.counter

        #return the dictionary
        return self.data


class MapMatch:
    def __init__(self, path, matchType):
        self.path = path
        self.seed = 1 #index of map to take a guess
        self.firstSearch = True #first time running search
        self.matchType = matchType



    def localize(self, localState, globalState):
        if self.matchType is "euler":
            return

        elif self.matchType is "closest":
            e, s, dPsi = self.mapMatch(globalState.posE, globalState.posN, globalState.psi)
            localState.updateMapMatchStates(e, dPsi, s)
            return

        else:
            sys.exit("invalid mapMatch Type")

    def mapMatch(self, posE, posN, psi):
        pEN = [posE, posN]
        pSE = self.convertToLocalPath(pEN)

        #avoid small bug where we go to negative at start of path
        if pSE[0] < 0:
            s = 0
        else:
            s = pSE[0]

        e = pSE[1]
        psiDes = np.interp(s, np.squeeze(self.path.s), np.squeeze(self.path.roadPsi))
        dPsi = psi - psiDes - np.pi/2 #need pi/2 correction to account for 0 being due east, not due north

        while dPsi > np.pi:
            dPsi = dPsi - 2*np.pi

        while dPsi < -np.pi:
            dPsi = dPsi + 2*np.pi

        return e, s, dPsi

    def convertToLocalPath(self, pEN):
        #reshape to rank 0 arrays
        posE = np.squeeze(self.path.posE)
        posN = np.squeeze(self.path.posN)
        roadPsi = np.squeeze(self.path.roadPsi)
        s = np.squeeze(self.path.s)


        m = posE.size  #number of points in the map

        if self.firstSearch is True:
            dist = np.zeros([m, 1]) #array of distances

            #go through all points in the map
            for i in range(m):
                pMap = [ posE[i], posN[i] ]
                dist[i] = np.linalg.norm(np.array(pEN) - np.array(pMap))


            # Get closest point and the corresponding distance
            absE = min(dist)
            idx = np.argmin(dist)

            #Use cross product to get the signed error
            #To determine sign of e, cross heading vector with vector from point to road
            #Append vectors with 0 to get 3 dims for convenient use of cross function
                
            #some weird stuff here to get both in same format for cross product    
            headingVector  = [-np.sin(roadPsi[idx]) , np.cos(roadPsi[idx]) , 0]
            pENaugmented = np.array([pEN[0], pEN[1], 0])    
            pathVector = [posE[idx], posN[idx] , 0]

            positionVector = pENaugmented -  pathVector
            crss = np.cross(headingVector, positionVector)

            pSE = np.zeros(2)
            pSE[0] = s[idx]    
            pSE[1] = np.sign(crss[2]) * absE

            self.firstSearch = False #next search use the seed
            self.seed = idx
            return pSE

        if self.firstSearch is False:
            #Go forward

            lastPair = 9999999.0 #Inf
            forwardInd = self.seed

            stillDecreasing = True

            while stillDecreasing:

                if forwardInd + 1 <= m-2:
                    pMap1 = [ posE[forwardInd], posN[forwardInd] ]
                    pMap2 = [ posE[forwardInd+1], posN[forwardInd+1] ]

                    currentPair = np.linalg.norm( np.array(pEN) - np.array(pMap1) ) + np.linalg.norm( np.array(pEN) - np.array(pMap2)) 
                else:
                    currentPair = 999999.0 #Inf

                stillDecreasing = currentPair < lastPair
                if stillDecreasing:
                    lastPair = currentPair
                    forwardInd = forwardInd + 1

            smallestForward = lastPair

            #Go back
            lastPair = 9999999.0 #inf
            backwardInd = self.seed
            stillDecreasing = True

            while stillDecreasing:
                if (backwardInd - 1) >= 1:
                    pMap1 = [ posE[backwardInd], posN[backwardInd] ]
                    pMap2 = [ posE[backwardInd -1], posN[backwardInd -1] ]

                    currentPair = np.linalg.norm(np.array(pEN) - np.array(pMap1)) + np.linalg.norm(np.array(pEN) - np.array(pMap2))

                else:
                    currentPair = 999999.0 #Inf

                stillDecreasing = currentPair < lastPair
                if stillDecreasing:
                    lastPair = currentPair
                    backwardInd = backwardInd - 1

            smallestBackward = lastPair

            if smallestBackward < smallestForward:
                lowSind = backwardInd - 1
                highSind = backwardInd

            else:
                lowSind = forwardInd
                highSind = forwardInd + 1

            #do not understand this math - need to think about further

            a = np.linalg.norm( np.array(pEN) - np.array([posE[lowSind] , posN[lowSind]]) )
            b = np.linalg.norm( np.array(pEN) - np.array([posE[highSind], posN[highSind]]))
            c = np.linalg.norm( np.array([posE[lowSind], posN[lowSind] ])- np.array([posE[highSind], posN[highSind]]) );
         
            deltaS = (a**2+c**2-b**2)/(2*c)
            absE = np.sqrt(np.abs(a**2-deltaS**2))
        

            headingVector = [ -np.sin(roadPsi[lowSind]), np.cos(roadPsi[lowSind]), 0]
            pENaugmented = np.array([pEN[0], pEN[1], 0])
            pathVector = np.array([posE[lowSind], posN[lowSind], 0])

            positionVector = pENaugmented - pathVector
            crss = np.cross(headingVector, positionVector)
            
            pSE = np.zeros(2)
            pSE[0] = s[lowSind] + deltaS
            pSE[1] = np.sign(crss[2]) * absE

            self.seed = lowSind

            return pSE

class Animation:
    def __init__(self, path, logFile):
    	self.path = path
    	self.logFile = logFile

    def animate(self, numFrames = 10):
    	return
    	#STOPPED HERE
    	# N = self.logFile["N"]

    	# for i in range(N):

    	# 	#Don't plot every frame
    	# 	if i%numFrames is 0:
    	# 		self.plotVehicle()







def  bicycleModel(vehicle, controlInput, localState, globalState, matchType, ts, K):
    #Implementation of bicycle model with force derating, but no longitudinal dynamics

    #Unpack variables for brevity
    FxDes = controlInput.Fx
    delta = controlInput.delta
    


    Ux = localState.Ux    
    r = localState.r
    Uy = localState.Uy
    e = localState.e
    deltaPsi = localState.deltaPsi
    s = localState.s

    psi = globalState.psi
    posN = globalState.posN
    posE = globalState.posE

    m = vehicle.m
    a = vehicle.a
    b = vehicle.b
    Iz = vehicle.Iz

    #calculate forces and tire slips
    FxF, FxR = getFx(FxDes, Ux, vehicle)
    alphaF, alphaR = getSlips(localState, vehicle, controlInput)
    FyF, FyR, zetaF, zetaR = tm.coupledTireForces(alphaF, alphaR,  FxF, FxR, vehicle)
    
    
    #Calculate state derivatives and update
    dUy = (FyF + FyR) / m - r*Ux
    dr  = (a*FyF - b*FyR) / Iz
    dUx = Uy * r + (FxF + FxR - FyF * delta) / m

    if matchType is "euler":
        de = Uy * np.cos(deltaPsi) + Ux * np.sin(deltaPsi)
        ds = Ux * np.cos(deltaPsi) - Uy * np.sin(deltaPsi)
        dDeltaPsi = r - K  * Ux

    dE = - Uy * np.cos(psi) - Ux * np.sin(psi)
    dN =   Ux * np.cos(psi) - Uy * np.sin(psi)
    dotPsi = r 

    #update states with Euler integration
    Uy = Uy + ts * dUy
    r  = r + ts * dr
    Ux = Ux + ts * dUx
    posE = posE + ts*dE
    posN = posN + ts*dN
    psi = psi + ts*dotPsi


    #For Euler integration, update states with ODEs 
    if matchType is "euler":
        e = e + ts*de 
        s = s + ts*ds
        deltaPsi = deltaPsi + ts * dDeltaPsi

    localState.update(Ux, Uy, r, e, deltaPsi, s)
    globalState.update(posE, posN, psi)
        
    return localState, globalState  
      











def getSlips(localState, veh, controlInput):
    Ux = localState.Ux
    Uy = localState.Uy
    r  = localState.r
    delta = controlInput.delta
    
    if Ux < 2.0:
        alphaF = 0 #speed too low to get slip estimate
        alphaR = 0

    else:
        alphaF = np.arctan( (Uy + veh.a * r) / Ux ) - delta
        alphaR = np.arctan( (Uy - veh.b * r) / Ux ) 

    return alphaF, alphaR


def getFx(FxDes, Ux, vehicle):

    #Implement engine and brake limits
    if FxDes > 0:
        if Ux == 0:
            Fx = FxDes #set to FxDes to avoid divide by zero
        else:
            Fx = min( vehicle.powerLimit / Ux - 0.7 * Ux ** 2 - 300, FxDes)
    else:
        Fx = FxDes

    #Distribute according to weight
    FxF = Fx * vehicle.b / vehicle.L
    FxR = Fx * vehicle.a / vehicle.L
    return FxF, FxR

