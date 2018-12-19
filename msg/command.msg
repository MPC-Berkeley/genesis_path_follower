# ---------------- action.msg -------------------
# Custom ROS message for action command
# Sent to raptor for data logging and to send to vehicle

Header       header # ROS defined header containing timestamp and sequence id
float64      action # Ax_mps2 command
float64[]    belief # belief state for logging
