# ---------------- cur_state.msg -------------------
# Custom ROS message with observation/measurements
# Sent from raptor with full observation

Header     header           # ROS defined header containing timestamp and sequence id
float64    velocity_mps     # vehicle velocity [m/s]
float64    distance_m       # vehicle distance to crosswalk [m]
bool       inCrosswalk_bool # pedestrian location [bool]
int64      posture_int      # pedestrian posture [enum]
float64    prevAx_mps2      # previous acceleration command [m/s^2]
	   
