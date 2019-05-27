# ---------------- plotting_state.msg -------------------
# Custom ROS message with plotting states
# Sent from simulator

Header     header           # ROS defined header containing timestamp and sequence id
float64    xP     # pedestrian X position [m]
float64    xV     # vehicle Y position    [m]
float64    ddxV   # vehicle accel         [m/s2]
float64    yP     # pedestrian Y position [m]