import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.optimize import fsolve
from matplotlib import animation
import pdb

class Road:
    def __init__(self, length = 80):
        
        x0 = 0
        y0 = 0 #start at origin
        self.numLanes = 2
        self.laneWidth = 3.7 #meters
        self.length = length #meters
        self.fig, self.ax = plt.subplots(1)
        self.fig.set_size_inches(11,8)
        self.startIC = (x0,y0)
        self.color = '#949392' #light grey
        self.plot()
        


    def plot(self):
        x, y = self.startIC

        for i in range(self.numLanes):
            rect = patches.Rectangle((x,y), self.laneWidth, self.length, facecolor = self.color, linewidth = 0, edgecolor = None, linestyle = '--')
            self.ax.add_patch(rect)
            x += self.laneWidth

            if i < self.numLanes - 1:
                if i == 1: #median
                    plt.plot([x, x], [y, y + self.length],color = 'y',LineStyle = '-', LineWidth = 3)
                else:
                    plt.plot([x, x], [y, y + self.length],color = 'w',LineStyle = '--', LineWidth = 3)

        
        x0,y0 = self.startIC
        self.ax.axis('equal') 
        #self.ax.set_ylim([y - 5, y + self.length + 5])
        #self.ax.set_xlim([x - 1, x + self.laneWidth * self.numLanes]) 
        
class Crosswalk:
    def __init__(self, road, roadFraction = 0.5):
        self.width = road.numLanes * road.laneWidth
        self.height = 3.0 #meters
        self.fig = road.fig
        self.ax  = road.ax
        self.numBars = road.numLanes * 4 #number of bars on crosswalk for graphical purposes
        self.roadFraction = roadFraction

        x,y = road.startIC
        #self.start = (x, y + road.length * self.roadFraction)
        self.start  = (x, 68.5)
        self.plot()

    def getAxis(self):
        return self.fig, self.ax

    def plot(self):
        x1,y1 = self.start
        self.ax.plot([x1, x1 + self.width], [y1, y1], LineWidth = 3, color = 'w')
        self.ax.plot([x1, x1 + self.width], [y1 + self.height, y1+self.height], LineWidth = 3, color = 'w')

        barWidth = self.width / self.numBars

        for i in range(self.numBars / 2): #numbars over 2 because half are the same color as the road
            rect = patches.Rectangle((x1, y1), barWidth, self.height, facecolor = 'w', linewidth = 0, edgecolor = None)
            self.ax.add_patch(rect)
            x1 += 2 * barWidth

class Sidewalk:
    def __init__(self, road, width = 1.2):
        self.fig = road.fig
        self.ax = road.ax
        self.length = road.length
        self.width = width
        self.start = road.startIC
        self.gap = road.numLanes * road.laneWidth
        self.color = '#dedcdb' #light grey
        self.plot()


    def plot(self):
        x0,y0 = self.start
        rect1 = patches.Rectangle((x0 - self.width,y0), self.width, self.length, facecolor = self.color, linewidth = 0, edgecolor = None)
        rect2 = patches.Rectangle((x0 + self.gap,y0), self.width, self.length, facecolor = self.color, linewidth = 0, edgecolor = None)
        self.ax.add_patch(rect1)  
        self.ax.add_patch(rect2)      

class Trajectory:
    def __init__(self, A, J, v0):
        self.N = 1000 #number of points in trajectory
        self.s = np.zeros((self.N,1))
        self.UxDesired = np.zeros(self.s.size)
        self.AxDesired = np.zeros(self.s.size)
        self.J = J #desired jerk limit in m/s^3
        self.A = A #desired maximum decel in m/s
        self.v0 = v0

    def getAxUx(self, dist2crosswalk):
        currS = -dist2crosswalk
        UxDes = np.interp(currS, self.s, self.UxDesired)
        AxDes = np.interp(currS, self.s, self.AxDesired)
        return AxDes, UxDes


    def getAccel(self, t, t1, t2, t3, t4, J, A):
        accel = np.zeros(t.size)

        for i in range(len(t)):
            if t[i] < t1:
                accel[i] = 0
            elif t[i] < t2:
                accel[i] = -J*(t[i] - t1)
            elif t[i] < t3:
                accel[i] = -A
            elif t[i] < t4:
                accel[i] = -A + J*(t[i] - t3)
            else:
                accel[i] = 0

        return accel

    ## function for fsolve
    def trapezoidalFunc(self, x, s0):
        t1 = x[0]
        t3 = x[1]
        t2 = t1 + self.A / self.J
        t4 = t3 + self.A / self.J
        t = np.linspace(0, t4, self.N)

        accel = self.getAccel(t, t1, t2, t3, t4, self.J, self.A)
        velocity = np.zeros(accel.size)
        position = np.zeros(accel.size)

        velocity[0] = self.v0
        position[0] = s0

        dt = t[1] - t[0]

        for i in range(0, len(t)-1):
            velocity[i+1] = velocity[i] + dt * accel[i]
            if velocity[i+1] < 0:
                velocity[i+1] = 0

            position[i+1] = position[i] + dt * velocity[i]


        vF = velocity[-1]
        sF = position[-1]

        F = np.array([vF,sF])

        return F   

    def initialize(self, dist2crosswalk):
        args = (-dist2crosswalk)
        x0 = np.array([0,3])
        x = fsolve(self.trapezoidalFunc, x0, args)
        t1 = x[0]
        t2 = t1 + self.A / self.J
        t3 = x[1]
        t4 = t3 + self.A / self.J

        t = np.linspace(0, t4, self.N)
        dt = t[1] - t[0]
        self.AxDesired = self.getAccel(t, t1, t2, t3, t4, self.J, self.A)

        print(t1)
        print(t3)

        velocity = np.zeros(self.AxDesired.size)
        velocity[0] = self.v0
        position = np.zeros(velocity.size)
        position[0] = -dist2crosswalk

        for i in range(0, len(t)-1):
            velocity[i+1] = velocity[i] + dt * self.AxDesired[i]
            if velocity[i+1] < 0:
                velocity[i+1] = 0

            position[i+1] = position[i] + dt * velocity[i]

        self.UxDesired = velocity
        self.s = position

        # plt.figure()
        # plt.plot(self.s, self.UxDesired);
        # plt.show()


class Vehicle:
    #cY = y coordinate of crosswalk
    def __init__(self, crosswalk, sidewalk, road, v0 = 20, lane = 3):
        self.v0 = v0
        self.lane = lane # lane 2 corresponds to third lane from the left
        self.width = 1.5 #meters
        self.height = 2.5 #meters
        self.state = "driving"
        self.kSpeed = 2.
        self.xStop = 63 #desired stop position
        self.stopBuffer = 2. #meters, give some room for vehicle to stop
        self.accelLim = 3.0 # m / s^2 comfortable braking acceleration
        self.brakeEmergencyLim = 9.0 # m/s^2 - allow car to brake at up to 9 m/s^2 if pedestrian gets too close
        self.maxPedestrianVelocity = 1.5 #be conservative
        self.maxTimeAdvantage = 4.0
        self.brakeDelay = 0.0 #estimated brake delay time, in seconds
        self.maxAccel = 3.0  #m/s2
        self.jerkLimit = 1.5 #m/s3
        self.sidewalk = sidewalk
        self.crosswalk = crosswalk
        self.road = road
        self.speedTrajectory = Trajectory(self.accelLim, self.jerkLimit, self.v0)

        self.brakeDistance = 0. #to be updated in planning step
        self.s0 = 0 #to be updated for emergency braking or normal braking states

    def getAccel(self, xP, dxP, xV, dxV, t, pedStart):
        accel, state, dxVdes = self.getAccelStateMachine(xP, dxP, xV, dxV, t, pedStart)
        accel = self.limitAccel(accel)
        return accel, state, dxVdes

    def resetState(self):
        self.state = "driving"

    def limitAccel(self, accel):
        if accel > self.accelLim:
            accel = self.accelLim

        if accel < -self.brakeEmergencyLim:
            accel = -self.brakeEmergencyLim

        return accel

    def calculateTimeAdvantage(self, xP, dxP, xV, dxV):

        dist2crosswalk = self.xStop - xV
        vehicleX = self.lane * self.road.laneWidth + self.road.laneWidth / 2 - self.width / 2

        pedDist2Vehicle = abs(vehicleX - xP)


        tc = dist2crosswalk  / dxV
        tp = pedDist2Vehicle / max(abs(dxP), self.maxPedestrianVelocity)

        return tc - tp

    def checkTransition(self, xP, xV, pedStart):
        if pedStart == "left":
            return (xP > self.crosswalk.width) or (xV > (self.xStop + self.stopBuffer))

        elif pedStart =="right":
            return (xP < self.crosswalk.width / 2) or (xV > (self.xStop + self.stopBuffer))
        
        else:
            print("Error - must select left or right")    

    def getAccelStateMachine(self, xP, dxP, xV, dxV, t, pedStart):
        if self.state == "driving":
            dxVdes = self.v0
            accel = self.kSpeed*(dxVdes - dxV)
            state = 0
            self.brakeDistance = dxV**2 / (2* self.accelLim)    #minimum distance required for braking, from kinematic equations
            self.eBrakeDistance = dxV**2 / (2 * self.brakeEmergencyLim)
            dist2crosswalk = self.xStop - xV

            if abs(dxP) > 0 and xV < self.xStop and not self.checkTransition(xP, xV, pedStart):
 
                if self.calculateTimeAdvantage(xP, dxP, xV, dxV) < -self.maxTimeAdvantage:
                    pass #continue driving at current speed because there is no need to stop in practice

                elif dist2crosswalk > self.brakeDistance:
                    self.state = "braking"
                    self.speedTrajectory.initialize(dist2crosswalk)

                elif (dist2crosswalk < self.brakeDistance) and (dist2crosswalk > self.eBrakeDistance):
                    self.state = "emergencyBraking"
                    self.s0 = xV

                else:
                    self.state = "speedup"


        elif self.state == "emergencyBraking":
            state = 2
            #print("emergencyBraking")

            #calculate distance to crosswalk:
            dist2crosswalk = self.xStop - xV

            dxVdes = self.v0 / np.sqrt(self.xStop-self.s0)*np.sqrt(max(dist2crosswalk, 0))
            accel = -dxV **2 / (2 * dist2crosswalk) + self.kSpeed*(dxVdes - dxV)

            #return to driving once past crosswalk or once pedestrian crosses
            if self.checkTransition(xP, xV, pedStart):
                self.state = "driving"
                #pdb.set_trace()

        elif self.state == "speedup":
            state = 3
            accel = self.maxAccel 
            dxVdes = dxV #not used

            #return to driving once past crosswalk
            if (xV > (self.xStop + self.stopBuffer)):
                self.state = "driving"
                #pdb.set_trace()            


        elif self.state == "braking":

            #don't brake until we need to
            dist2crosswalk = self.xStop - xV
            accelDesired, dxVdes = self.speedTrajectory.getAxUx(dist2crosswalk)

            accel = accelDesired + self.kSpeed*(dxVdes - dxV)
            state = 1

            #return to driving once past crosswalk or once pedestrian crosses
            if self.checkTransition(xP, xV, pedStart):
                self.state = "driving"
                #pdb.set_trace()

        #print(self.state)
        return accel, state, dxVdes



class Pedestrian:
    def __init__(self, crosswalk, sidewalk, v0 = 0., acceptedGap = 2.0, start = "left"):
        self.m = 50. #kg
        self.v0 = v0
        self.radius = 1. #m
        self.state = "waiting" #possible states are "waiting"
        self.kSpeed = 10 #m/s2 per m/s of error
        self.acceptedGap = acceptedGap #seconds
        self.vDes = 1.2 #m/s
        self.prevGap = 0. #keep track of gap at last iteration
        self.gap = 0.

        #start on left side of crosswalk
        if start == "left":
            self.xP0 = -sidewalk.width #meters
            

        #start on right side of crosswalk
        else:
            self.xP0 = crosswalk.width + sidewalk.width
            self.vDes = -self.vDes

    def getAccel(self, xP, dxP, xV, dxV, t, crosswalk):
        accel = self.getAccelStateMachine(xP, dxP, xV, dxV, t, crosswalk)
        return accel

    def updateGap(self, newGap):
        self.acceptedGap = newGap

    def resetState(self):
        self.state = "waiting"
        self.gap = 0.0
        self.prevGap = 0.0

    def getAccelStateMachine(self, xP, dxP, xV, dxV, t, crosswalk):
        self.prevGap = self.gap
        self.gap = self.calculateGap(xV, dxV, crosswalk)
        if self.state == "waiting":
            if ((self.gap < self.acceptedGap) and (self.prevGap > self.acceptedGap)) or self.gap > 1000:
                if dxV > 1:
                    self.state = "walking"

            state = 0
            accel = 0
        elif self.state == "walking":
            accel = self.kSpeed * (self.vDes - dxP)
            state = 1

        return accel, state, self.gap

    def calculateGap(self,xV,dxV, crosswalk):
        gap = (crosswalk.start[1] - xV) / (dxV + .0000000001) #time based gap
        #car is past the crosswalk
        if gap < 0: #make gap large to trigger a walking state once car crosses crosswalk
            gap = 99999.
        return gap


#Simulate with double euler integration
class Simulation:
    def __init__(self, road, crosswalk, vehicle, pedestrian, ts = 0.1, N = 100):
        self.ts = ts #seconds
        self.N = N #number of time steps to simulate
        self.vehicle = vehicle
        self.road = road
        self.pedestrian = pedestrian
        self.crosswalk = crosswalk
        self.out = {}
        self.totalTime = 0.

    def run(self):

        #Initialize arrays

        #vehicle motion
        xV = np.zeros((self.N, 1))
        dxV = np.zeros(xV.shape); dxV[0] = self.vehicle.v0
        ddxV = np.zeros(dxV.shape)

        #pedestrian motion
        xP = np.zeros((self.N, 1)); xP[0] = self.pedestrian.xP0
        dxP = np.zeros(xP.shape); dxP[0]  = self.pedestrian.v0
        ddxP = np.zeros(xP.shape)

        pedestrianState =  np.zeros(xP.shape)
        vehicleState = np.zeros(xP.shape)
        gap = np.zeros(xP.shape)
        dxVdes = np.zeros(xP.shape)


        #other arrays
        t = np.zeros((self.N, 1))

        #main loop

        for i in range(1, self.N):
            t[i] = t[i-1] + self.ts

            ddxV[i], vehicleState[i], dxVdes[i] = self.vehicle.getAccel(xP[i-1], dxP[i-1], xV[i-1], dxV[i-1], t[i-1], self.crosswalk)
            ddxP[i], pedestrianState[i], gap[i] = self.pedestrian.getAccel(xP[i-1], dxP[i-1], xV[i-1], dxV[i-1], t[i-1], self.crosswalk)

            dxV[i] = self.ts * ddxV[i] + dxV[i-1]
            dxP[i] = self.ts * ddxP[i] + dxP[i-1]

            if dxV[i] < 0:
                dxV[i] = 0 #car cannot go backward

            xV[i] = self.ts * dxV[i] + xV[i-1]
            xP[i] = self.ts * dxP[i] + xP[i-1]

            #check for termination
            if xV[i] > self.road.length:
                self.totalTime = t[i]
                print("Simulation terminated after %d seconds." %t[i])
                break



        self.out = {'t': t, 'xV': xV, 'dxV': dxV, 'ddxV': ddxV, 'xP': xP, 'dxP': dxP, 'ddxP': ddxP,
        'pedestrianState': pedestrianState, 'vehicleState': vehicleState, 'dxVdes': dxVdes}


        return self.out


    def animate(self):
        #get up to date axis with road and crosswalk plotted
        fig,ax  = self.crosswalk.getAxis()

        #unpack arrays
        xP = self.out["xP"]
        xV = self.out["xV"]

        #plot vehicle
        vehicle    = patches.Rectangle((0, 0), 0, 0, fc='k')
        pedestrian = patches.Rectangle((0, 0), 0, 0, fc='r')
        
        vehicleX = self.vehicle.lane * self.road.laneWidth + self.road.laneWidth / 2 - self.vehicle.width/2
        pedestrianY = self.crosswalk.start[1]


        def init():
            ax.add_patch(vehicle)
            ax.add_patch(pedestrian)
            return vehicle, pedestrian

        def animate(i):

            vehicle.set_width(self.vehicle.width)
            vehicle.set_height(self.vehicle.height)
            vehicle.set_xy([vehicleX, xV[i]]) #Note that xV is vertical for the car

            pedestrian.set_width(self.pedestrian.radius)
            pedestrian.set_height(self.pedestrian.radius)
            pedestrian.set_xy([xP[i] ,pedestrianY]) 

            return vehicle, pedestrian

        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(xV), interval=100, blit=True)
        plt.show()        






# def initAnimation():
#     ax.add_patch(vehiclePatch)
#     return vehiclePatch

# def animate(i):
#     patch.set_width(1.5) #hardcoded
        













    












