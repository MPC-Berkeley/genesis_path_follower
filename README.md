# gps_path_follower
Code to track a GPS-specified set of waypoints using a kinematic model of MPC.

There are three major components referenced in the launch file (for simulation, at present):
  * mpc_cmd_pub.jl
    * Subscribes to the state from the vehicle (or vehicle simulator).
    * Queries a reference generator (ref_gps_traj.py) to get the desired open-loop trajectory.  A full global trajectory is provided ahead of time in csv format.
    * Uses a nonlinear kinematic MPC model with IPOPT to generate the optimal acceleration and steering angle (tire) commands
    * Publishes the commands to the vehicle's low level controller, along with the desired open-loop trajectory (target_path)
      and the predicted open-loop trajectory MPC solution.
  * vehicle_simulator.py
    * This is for sake of tuning algorithms.  It uses a dynamic bicycle model that captures vehicle motion better under higher lateral acceleration.
    * Low-level P controller used to simulate the fact that acceleration/steering angle control inputs are tracked with some delay.
    * The hope is that this can be simply replaced by the vehicle's actual command interface once algorithms are fully tested.
  * plot_gps_traj.py
    * This is a simple visualizer of the path following behavior.
    * The global trajectory is plotted in black and kept fixed.
    * The vehicle's current state, target path (open-loop reference), and predicted MPC path are updated via subscriber callbacks.
---
CSV Format for Reference Path (used by ref_gps_traj.py):
  * Each row is formatted as [time, latitude, longitude, heading]
  * heading is the angle, in degrees) clockwise from North (i.e. N = 0, E = 90)
  * **TODO:** allow user to switch between heading and yaw cleanly.
  
