Header header
string solv_status_long
string solv_status_lat
float64 solv_time_long
float64 solv_time_lat

# MPC Prediction reconstructed from Frenet predictions
float64[] xs
float64[] ys
float64[] psis
float64[] vxs
float64[] vys
float64[] wzs
# Frenet predictions
float64[] ss_fren
float64[] vs_fren
float64[] eys_fren
float64[] epsis_fren
float64[] vys_fren
float64[] wzs_fren
float64[] curv

#Input prediction
float64[] df
float64[] acc

# Reference before Frenet (in XY-coord)
float64[] xr
float64[] yr
float64[] psir
float64[] vxr
float64[] vyr
float64[] wzr

# Reference reconstructed using curvature from Frenet
float64[] xr_recon
float64[] yr_recon
float64[] psir_recon
# float64[] vxr_recon
# float64[] vyr_recon
# float64[] wzr_recon


