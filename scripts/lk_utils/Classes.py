import numpy as np

class LMPCprediction():
    """Object collecting the predictions and SS at eath time step
    """
    def __init__(self, N, n, d, TimeLMPC, numSS_Points, Laps):
        """
        Initialization:
            N: horizon length
            n, d: input and state dimensions
            TimeLMPC: maximum simulation time length [s]
            num_SSpoints: number used to buils SS at each time step
        """
        self.oneStepPredictionError = np.zeros((n, TimeLMPC, Laps))
        self.PredictedStates = np.zeros((N+1, n, TimeLMPC, Laps))
        self.PredictedInputs = np.zeros((N, d, TimeLMPC, Laps))

        self.SSused   = np.zeros((n , numSS_Points, TimeLMPC, Laps))
        self.Qfunused = np.zeros((numSS_Points, TimeLMPC, Laps))


class ClosedLoopData():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self, dt, Time, v0, n):
        """Initialization
        Arguments:
            dt: discretization time
            Time: maximum time [s] which can be recorded
            v0: velocity initial condition
        """
        self.dt = dt
        self.Points = int(Time / dt)  # Number of points in the simulation
        self.measSteering = np.zeros((self.Points, 1))  # Initialize the input vector
        self.u = np.zeros((self.Points, 2))  # Initialize the input vector
        self.x = np.zeros((self.Points + 1, n))  # Initialize state vector (In curvilinear abscissas)
        self.x_glob = np.zeros((self.Points + 1, n))  # Initialize the state vector in absolute reference frame
        self.solverTime = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.sysIDTime  = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.contrTime  = np.zeros((self.Points + 1, 1))  # Initialize state vector (In curvilinear abscissas)
        self.SimTime = -1
        self.x[0,0] = v0
        self.x_glob[0,0] = v0
        self.CurrentState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.n = n

    def updateInitialConditions(self, x, x_glob):
        """Clears memory and resets initial condition
        x: initial condition is the curvilinear reference frame
        x_glob: initial condition in the inertial reference frame
        """
        self.x[0, :] = x
        self.x_glob[0, :] = x_glob

        self.x[1:, :] = np.zeros((self.x.shape[0]-1, self.n))
        self.x_glob[1:, :] = np.zeros((self.x.shape[0]-1, self.n))
        self.SimTime = -1

    def addMeasurement(self, xMeasuredGlob, xMeasuredLoc, uApplied, solverTime, sysIDTime, contrTime, measSteering):
        """Add point to the object ClosedLoopData
        xMeasuredGlob: measured state in the inerial reference frame
        xMeasuredLoc: measured state in the curvilinear reference frame
        uApplied: input applied to the system
        """
        self.SimTime = self.SimTime + 1
        self.x[self.SimTime, :]      = xMeasuredLoc
        self.x_glob[self.SimTime, :] = xMeasuredGlob
        self.u[self.SimTime, :]      = uApplied
        self.solverTime[self.SimTime, :]  = solverTime
        self.sysIDTime[self.SimTime, :]   = sysIDTime
        self.contrTime[self.SimTime, :]   = contrTime
        self.measSteering[self.SimTime, :]      = measSteering