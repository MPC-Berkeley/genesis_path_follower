#!/usr/bin/env python
import argparse
import scipy.io as sio
from gmplot import gmplot # pip install gmplot
import numpy as np
import pdb

''' Code to convert matfile to Google Map GPS HTML Plot '''
def plot(r_matfile, f_matfile, save_name):

	data = sio.loadmat(f_matfile)
	assert(data['mode'] != 'Sim')
	lats = np.ravel(data['lat']).tolist()
	lons = np.ravel(data['lon']).tolist()

	if 't_en' in data.keys():
		t_en = data['t_en']
		t_data = data['t']
		st_ind = np.argmin(np.square(t_data - t_en))
	else:
		st_ind = 0

	data_record = sio.loadmat(r_matfile)
	assert(data_record['mode'] != 'Sim')	
	
	lats_r = np.ravel(data_record['lat']).tolist()
	lons_r = np.ravel(data_record['lon']).tolist()

	gmap = gmplot.GoogleMapPlotter(lats[0], lons[0], 18) # lat, lon, zoom_level
	gmap.scatter([lats[st_ind]], [lons[st_ind]], color="b", size=10, symbol='x') # starting point in blue
	gmap.scatter([lats[-1]],[lons[-1]], color="g", size=10, symbol='x') # ending point in green	
	gmap.plot(lats_r, lons_r, color="k", size=10, symbol='x', ls=None) # black line: path recorded
	gmap.plot(lats, lons, color="r", size=10, symbol='x', ls=None) # red line: path followed	
	gmap.draw(save_name)

if __name__=='__main__':
	parser = argparse.ArgumentParser('Plot GPS map from 2 processed matfiles containing lat/lon history from a path recording and following experiment.')
	parser.add_argument('--pr', type=str, required=True, help='Matfile of recorded path.')
	parser.add_argument('--pf', type=str, required=True, help='Matfile of followed path.')
	parser.add_argument('-o', '--out', type=str, required=True, help='Map HTML save location.')
	args = parser.parse_args()
	plot(args.pr, args.pf, args.out)
