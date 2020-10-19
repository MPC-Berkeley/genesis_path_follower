# genesis_path_follower
Code to track a GPS-specified set of waypoints using MPC.  In the master branch, the nonlinear kinematic module is currently used.
---
There are three major launch files to use:
  * path_record.launch
    * This is used to record the vehicle state in a demonstrated trajectory.
    * The launch file asks for a matfile of waypoints: this is just to aid in the visualization and isn't used for control.
  * sim_path_follow.launch
    * This uses a dynamic bicycle vehicle model to simulate following the demonstrated trajectory (matfile of waypoints).
    * Initial vehicle pose is required.
  * path_follow.launch
    * This is used to publish commands to the actual vehicle.
  * Common settings:
    * Track path using time (i.e. varying velocity) or with a fixed speed.
    * Reference system is defined based on (lat0, lon0) as the global origin.
    * Use heading (cw from North) or psi/yaw (ccw from East).  For the Genesis, we can use the latter one.
---
The launch files use the following scripts:
  * mpc_cmd_pub.py
    * Subscribes to the state from the vehicle (or vehicle simulator).
    * Queries a reference generator (ref_gps_traj.py) to get the desired open-loop trajectory.  A full global trajectory is provided ahead of time in mat format from the launch file.
    * Uses a nonlinear kinematic MPC model with Casadi/IPOPT to generate the optimal acceleration and steering angle (tire) commands.
    * Publishes the commands to the vehicle's (or simulated vehicle) low level controller, along with a message capturing the MPC reference and solution.

  * vehicle_simulator.py
    * This is for sake of tuning algorithms.  It uses a dynamic bicycle model for high model fidelity.
    * Low-level P controller used to simulate the fact that acceleration/steering angle control inputs are tracked with some delay.  This P gains can be tuned to simulate timing delays on the real vehicle.
    * The topics used for control match the Genesis Interface exactly.  Thus this module can be swapped with the real vehicle interface with no issues.

  * gps_plotter.py / gps_vehicle_plotter..py
    * This is a simple visualizer of the path following behavior.
    * The global trajectory is plotted in black and kept fixed.
    * The vehicle's current state (blue or with a vehicle plotted), target path (open-loop reference, in red), and predicted MPC path (green) are updated via subscriber callbacks.

  * state_publisher.py
    * Used for the real vehicle, in place of vehicle_simulator.py.
    * Subscribes to GPS/IMU topics and provides a roughly synchronized version of the vehicle state for convenience.
---
Mat Format for Reference Path (used by ref_gps_traj.py):
  * Keys: 't', 'x', 'y', 'v', 'psi', 'a', 'df', 'lat', 'lon' (i.e. the state information and ROS time)
  * Values: each key is associated with a N-long array, where N is the number of state samples recorded
  * There is also a 'mode' entry (Real, Sim, or Follow) to aid with plotting/analysis and keeping track of data.
---
Other notable branches and collaborators:
* Nitin Kapania started the lanekeeping branch which handles tracking in a road-aligned/Frenet frame using a feedback-feedforward controller.
  * Github: https://github.com/nkapania
  * A good reference can be found at: https://ddl.stanford.edu/publications/design-feedback-feedforward-steering-controller-accurate-path-tracking-and-stability
* George Xiaojing Zhang started the GXZ_kin branch, which has a decoupled MPC implementation that solves two quadratic programs (QPs) for acceleration and steering, respectively, at each time step.
  * Github: https://github.com/XiaojingGeorgeZhang
  * The key idea is to pose the problem as solving for the acceleration profile given the trajectory and then solving for the steering profile.  By decoupling in this way and using small angle approximations, two quadratic program problems can be solved (using Gurobi, for example) efficiently.
---
Contact Vijay Govindarajan (govvijay@berkeley.edu, https://github.com/govvijaycal) with any questions or comments.
